"""Fleet view aggregated by the dashboard bridge from what it *overhears*.

Pure and passive: it ingests copies of robot beacons/events and the world's ground-truth feed
and derives what the dashboard shows. It has no way to send anything to a robot.
"""
from __future__ import annotations

import json
import math
from collections import deque
from pathlib import Path

from amr.agent.deadlock import all_cycles
from amr.core.grid_map import GridMap

MODE_NAMES = {"idle": "idle", "moving": "moving", "waiting": "waiting for lock", "yielding": "yielding",
              "charging": "charging", "working": "picking/dropping"}
ALERT_CAP = 200


class FleetView:
    def __init__(self, gmap: GridMap, scenario: dict, rules_file: str | Path | None = None,
                 stale_s: float = 0.45, lost_s: float = 3.0):
        self.map = gmap
        self.scenario = scenario
        self.rules_file = Path(rules_file) if rules_file else None
        self.stale_s = stale_s
        self.lost_s = lost_s
        self.robots: dict[int, dict] = {}         # rid -> {"b": beacon, "rt": recv time}
        self.truth: dict = {}
        self.truth_t = 0.0
        self.alerts: deque = deque(maxlen=ALERT_CAP)
        self.alert_seq = 0
        self._recent: dict[tuple, float] = {}
        self.done: dict[int, float] = {}
        self.blocks: dict[tuple, dict] = {}       # cell -> {"exp": t, "by": rid}
        self.unknown: dict[int, float] = {}       # sid -> since
        self.deadlocks_resolved = 0
        self._broken_keys: dict[frozenset, float] = {}
        self.n_tasks = len(scenario.get("tasks", []))
        self.msg_window: deque = deque()          # (t, bytes)
        self.req_count = 0
        self.wait_total = 0.0
        self.partition_state: list | None = None
        self._part_candidate = None
        self._part_since = 0.0
        self.t_start = None
        self.cycles: list[list[int]] = []
        self.robot_ids = [r["id"] for r in scenario.get("robots", [])]
        self.rules: dict = {}
        self._rules_mtime = None

    # ------------------------------------------------------------------ ingest
    def ingest(self, pkt: dict, nbytes: int, now: float) -> None:
        if self.t_start is None:
            self.t_start = now
        t = pkt.get("t")
        if t == "B":
            b = pkt["b"]
            rid = b["id"]
            prev = self.robots.get(rid)
            if prev and prev["b"].get("m") == "waiting":
                self.wait_total += min(0.5, now - prev["rt"])
            self.robots[rid] = {"b": b, "rt": now}
            self.msg_window.append((now, nbytes))
        elif t == "O":
            self.msg_window.append((now, nbytes))
            self._event(pkt["src"], pkt["b"], now)
        elif t == "W":
            self.truth = pkt["b"]
            self.truth_t = now

    def _event(self, src: int, m: dict, now: float) -> None:
        k = m.get("k")
        if k == "DONE":
            self.done.setdefault(int(m["j"]), now)
        elif k == "REQ":
            self.req_count += 1
        elif k == "BLOCK":
            for c in m.get("cells", []):
                self.blocks[tuple(c)] = {"exp": now + float(m.get("ttl", 30.0)), "by": src}
        elif k == "ALERT":
            typ = m.get("type")
            if typ == "deadlock_broken":
                key = frozenset(m.get("cycle", []))
                if now - self._broken_keys.get(key, -1e9) > 5.0:
                    self.deadlocks_resolved += 1
                self._broken_keys[key] = now
            if typ == "lease_expired" and m.get("sid") is not None:
                self.unknown.setdefault(int(m["sid"]), now)
            if typ == "clear_confirmed" and m.get("sid") is not None:
                self.unknown.pop(int(m["sid"]), None)
            self._add_alert(typ, m.get("sev", "info"), m.get("msg", typ), m.get("robots", [src]),
                            m.get("x"), m.get("y"), now, by=src)

    def _add_alert(self, typ, sev, msg, robots, x, y, now, by=None) -> None:
        # dedupe: several robots report the same thing (e.g. every peer notices R3 is lost)
        key = (typ, tuple(sorted(robots or [])))
        if typ in ("deadlock_detected", "deadlock_broken", "yield_impossible"):
            key = (typ, tuple(sorted(robots or [])), by if typ == "deadlock_broken" else None)
        if now - self._recent.get(key, -1e9) < 5.0:
            return
        self._recent[key] = now
        self.alert_seq += 1
        self.alerts.append({"id": self.alert_seq, "type": typ, "sev": sev, "msg": msg, "robots": list(robots or []),
                            "x": x, "y": y, "t": round(now - (self.t_start or now), 2)})

    # ------------------------------------------------------------------ derived state
    def _poll_rules(self) -> None:
        if not self.rules_file:
            return
        try:
            m = self.rules_file.stat().st_mtime
            if m != self._rules_mtime:
                self._rules_mtime = m
                self.rules = json.loads(self.rules_file.read_text(encoding="utf-8") or "{}")
        except (OSError, ValueError):
            pass

    def snapshot(self, now: float) -> dict:
        self._poll_rules()
        while self.msg_window and now - self.msg_window[0][0] > 2.0:
            self.msg_window.popleft()
        for c in [c for c, v in self.blocks.items() if v["exp"] <= now]:
            del self.blocks[c]
        robots = []
        live = set()
        edges = {}
        prios = {}
        holders: dict[int, tuple] = {}
        waiters: dict[int, list] = {}
        for rid, r in sorted(self.robots.items()):
            b, age = r["b"], now - r["rt"]
            stale = age > self.stale_s
            lost = age > self.lost_s
            if not stale:
                live.add(rid)
                edges[rid] = b.get("wf")
                prios[rid] = b.get("pr", 0.0)
            for sid, remaining in b.get("h", []):
                rem = remaining - age
                if not stale:
                    holders[sid] = (rid, rem)
                elif rem < 0 and sid not in holders:
                    self.unknown.setdefault(sid, now)      # dead holder: lease expired
                elif sid not in holders:
                    holders[sid] = (rid, rem)
            if b.get("wq") is not None and not stale:
                waiters.setdefault(b["wq"], []).append(rid)
            path = [[c % self.map.width, c // self.map.width] for c in b.get("pp", [])]
            robots.append({
                "id": rid, "x": b["p"][0], "y": b["p"][1], "th": b["p"][2], "v": b.get("v", [0, 0]),
                "mode": "lost" if lost else b.get("m", "idle"), "stale": stale, "age": round(age, 2),
                "battery": b.get("bat"), "task": b.get("task"), "stage": b.get("st", ""),
                "held": [h[0] for h in b.get("h", [])], "wf": b.get("wf"), "wq": b.get("wq"),
                "prio": b.get("pr"), "path": path, "degraded": bool(b.get("dg")), "epoch": b.get("ep"),
                "live": [i for i in range(1, 63) if b.get("lv", 0) >> i & 1],
            })
        for sid, rid_rem in holders.items():
            if rid_rem[0] in live:
                self.unknown.pop(sid, None)
        self.cycles = [c for c in all_cycles(edges)]
        sections = []
        for s in self.map.sections:
            h = holders.get(s.sid)
            blocked = any(tuple(c) in self.blocks for c in s.cells)
            if s.sid in self.unknown and (h is None or h[0] not in live):
                state = "unknown"
            elif h is not None:
                state = "held"
            else:
                state = "free"
            sections.append({"sid": s.sid, "label": s.label, "state": state, "blocked": blocked,
                             "holder": h[0] if h else None,
                             "lease": round(max(0.0, h[1]), 1) if h else None,
                             "waiters": sorted(waiters.get(s.sid, []), key=lambda r: -(prios.get(r) or 0))})
        self._partition(live, now)
        win = 2.0
        nbytes = sum(b for _, b in self.msg_window)
        n_live = max(1, len(live))
        tr = self.truth
        return {
            "type": "state",
            "t": round(now - (self.t_start or now), 2),
            "robots": robots,
            "sections": sections,
            "cycles": self.cycles,
            "blocks": [{"x": c[0], "y": c[1], "ttl": round(v["exp"] - now, 1), "by": v["by"]}
                       for c, v in self.blocks.items()],
            "pallets": tr.get("blockages", []),
            "lidar": tr.get("lidar", {}),
            "world_ok": (now - self.truth_t) < 2.0,
            "kpi": {
                "tasks_done": len(self.done), "tasks_total": self.n_tasks,
                "collisions": tr.get("collisions", 0),
                "deadlocks_resolved": self.deadlocks_resolved,
                "mean_wait_s": round(self.wait_total / max(1, self.req_count), 2),
                "msgs_per_s": round(len(self.msg_window) / win, 1),
                "bytes_per_robot_s": round(nbytes / win / n_live, 0),
                "robots_live": len(live), "robots_total": len(self.robot_ids) or len(self.robots),
                "min_robot_dist": tr.get("min_robot_dist"),
            },
            "faults": self.rules,
        }

    def _partition(self, live: set[int], now: float) -> None:
        """Partition detection from the robots' own live-peer sets (lv bitmasks)."""
        if len(live) < 2:
            return
        adj = {r: set() for r in live}
        for r in live:
            lv = self.robots[r]["b"].get("lv", 0)
            for q in live:
                if q != r and (lv >> q & 1):
                    adj[r].add(q)
                    adj[q].add(r)
        comps, seen = [], set()
        for r in sorted(live):
            if r in seen:
                continue
            comp, st = [], [r]
            seen.add(r)
            while st:
                u = st.pop()
                comp.append(u)
                for v in adj[u]:
                    if v not in seen:
                        seen.add(v)
                        st.append(v)
            comps.append(sorted(comp))
        if len(comps) > 1:
            key = sorted(comps)
            if self._part_candidate != key:
                self._part_candidate, self._part_since = key, now     # must persist 1.5 s
            elif self.partition_state != key and now - self._part_since >= 1.5:
                self.partition_state = key
                single = [c[0] for c in key if len(c) == 1]
                txt = " | ".join("{" + ",".join(f"R{i}" for i in c) + "}" for c in key)
                if len(key) == 2 and single:
                    msg = f"R{single[0]} isolated from the fleet (link down / dead zone)"
                else:
                    msg = f"Network partition detected: {txt}"
                self._add_alert("partition_detected", "crit", msg, sorted(live), None, None, now)
            return
        self._part_candidate = None
        if self.partition_state is not None:
            self.partition_state = None
            self._add_alert("partition_healed", "info", "Network partition healed: all robots hear each other",
                            sorted(live), None, None, now)

    def alerts_since(self, last_id: int) -> list[dict]:
        return [a for a in self.alerts if a["id"] > last_id]
