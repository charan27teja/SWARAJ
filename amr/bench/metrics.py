"""Metrics recorder: a passive test instrument. It observes the copies of robot messages that
robots send to observers, plus the world's ground truth (collisions). Robots never read it."""
from __future__ import annotations

import json
import math
from collections import defaultdict

from amr.agent.deadlock import all_cycles


class MetricsRecorder:
    def __init__(self, n_tasks: int, robots: list[int]):
        self.n_tasks = n_tasks
        self.robots = list(robots)
        self.done: dict[int, list[tuple[float, int]]] = defaultdict(list)   # tid -> [(t, robot)]
        self.picks: dict[int, set[int]] = defaultdict(set)
        self.claims: list[tuple[float, int, int]] = []
        self.alerts: list[dict] = []
        self.bytes_by_robot: dict[int, int] = defaultdict(int)
        self.msgs_by_robot: dict[int, int] = defaultdict(int)
        self.beacon_bytes: list[int] = []
        self.wait_samples: dict[int, float] = defaultdict(float)
        self.req_count = 0
        self.last_mode: dict[int, str] = {}
        self.last_beacon_t: dict[int, float] = {}
        self.wf: dict[int, tuple] = {}                # robot -> (wait_for, beacon time)
        self.active_cycles: dict[frozenset, float] = {}
        self.cycle_durations: list[float] = []
        self._next_cycle_check = 0.0
        self.first_t = None
        self.faults: list[dict] = []
        self.live_masks: dict[int, int] = {}
        self.full_membership_t: float | None = None

    # ------------------------------------------------------------------ taps
    def on_sent(self, src: int, nbytes: int) -> None:
        self.bytes_by_robot[src] += nbytes
        self.msgs_by_robot[src] += 1

    def on_message(self, src: int, body: dict, now: float) -> None:
        k = body.get("k")
        if k == "B":
            prev = self.last_beacon_t.get(src)
            self.last_beacon_t[src] = now
            if self.last_mode.get(src) == "waiting" and prev is not None:
                self.wait_samples[src] += min(0.5, now - prev)
            self.last_mode[src] = body.get("m", "")
            self.live_masks[src] = body.get("lv", 0)
            self.wf[src] = (body.get("wf"), now)
        elif k == "DONE":
            self.done[int(body["j"])].append((now, src))
        elif k == "PICK":
            self.picks[int(body["j"])].add(src)
        elif k == "CLAIM":
            self.claims.append((now, src, int(body["j"])))
        elif k == "REQ":
            self.req_count += 1
        elif k == "ALERT":
            a = dict(body)
            a["obs_t"] = now
            self.alerts.append(a)

    def tick(self, now: float) -> None:
        """Ground-truth deadlock tracking: every 0.5 s rebuild the fleet-wide wait-for graph from
        the latest beacons and time every cycle from appearance to disappearance."""
        if now < self._next_cycle_check:
            return
        self._next_cycle_check = now + 0.5
        edges = {r: wf for r, (wf, t) in self.wf.items() if now - t <= 0.6}
        cur = {frozenset(c) for c in all_cycles(edges)}
        for key in cur:
            self.active_cycles.setdefault(key, now)
        for key in [k for k in self.active_cycles if k not in cur]:
            dur = now - self.active_cycles.pop(key)
            if dur >= 1.0:                     # ignore sub-second view transients
                self.cycle_durations.append(dur)

    def on_fault(self, f: dict, now: float) -> None:
        self.faults.append({**f, "applied": now})

    # ------------------------------------------------------------------ results
    def completed(self) -> int:
        return len(self.done)

    def all_done(self) -> bool:
        return len(self.done) >= self.n_tasks

    def summary(self, now: float, world_truth: dict, alive: list[int]) -> dict:
        done_t = [min(t for t, _ in v) for v in self.done.values()]
        makespan = max(done_t) if (done_t and self.all_done()) else None
        dups = sum(1 for v in self.done.values() if len({r for _, r in v}) > 1)
        dup_picks = sum(1 for s in self.picks.values() if len(s) > 1)
        total_wait = sum(self.wait_samples.values())
        n = max(1, len(self.robots))
        horizon = makespan if makespan else now
        ttr = list(self.cycle_durations)
        unresolved = [k for k, t0 in self.active_cycles.items() if now - t0 >= 10.0]
        return {
            "makespan": makespan,
            "completed": len(self.done),
            "n_tasks": self.n_tasks,
            "all_done": self.all_done(),
            "sim_time": round(now, 2),
            "throughput_per_h": (len(self.done) / horizon * 3600.0) if horizon > 0 else 0.0,
            "collisions": world_truth["collisions"],
            "wall_contacts": world_truth["wall_contacts"],
            "obstacle_contacts": world_truth["obstacle_contacts"],
            "min_robot_dist": world_truth["min_robot_dist"],
            "deadlocks": len(ttr) + len(unresolved),
            "deadlocks_resolved": len(ttr),
            "unresolved_deadlocks": len(unresolved),
            "mean_ttr_s": (sum(ttr) / len(ttr)) if ttr else None,
            "max_ttr_s": max(ttr) if ttr else None,
            "mean_wait_s": total_wait / max(1, self.req_count),
            "total_wait_s": total_wait,
            "lock_requests": self.req_count,
            "duplicates": max(dups, dup_picks),
            "bytes_per_robot_s": sum(self.bytes_by_robot.values()) / n / max(horizon, 1e-6),
            "msgs_per_robot_s": sum(self.msgs_by_robot.values()) / n / max(horizon, 1e-6),
            "beacon_bytes_mean": (sum(self.beacon_bytes) / len(self.beacon_bytes)) if self.beacon_bytes else None,
            "alerts": len(self.alerts),
            "yields": sum(1 for a in self.alerts if a.get("type") == "deadlock_broken"),
            "blockage_events": sum(1 for a in self.alerts if a.get("type") == "aisle_blocked"),
            "reauctions": sum(1 for a in self.alerts if a.get("type") == "task_reauctioned"),
            "recovery_s": self._recovery(),
        }

    def _recovery(self) -> float | None:
        for f in self.faults:
            if f.get("action") == "kill":
                t0 = f["applied"]
                victim = f.get("robot")
                vt = [j for (t, r, j) in self.claims if r == victim and t <= t0]
                if not vt:
                    return None
                j = vt[-1]
                later = [t for (t, r, jj) in self.claims if jj == j and r != victim and t > t0]
                if j in self.done and all(t <= t0 for t, _ in self.done[j]):
                    return None
                return (min(later) - t0) if later else None
            if f.get("action") == "heal":
                return None if self.full_membership_t is None else self.full_membership_t - f["applied"]
        return None

    def check_membership(self, now: float, alive: list[int]) -> None:
        """For partition recovery: first time after the last fault that every alive robot hears all
        other alive robots."""
        if not any(f.get("action") == "heal" for f in self.faults) or self.full_membership_t is not None:
            return
        want = 0
        for r in alive:
            want |= 1 << r
        for r in alive:
            if (self.live_masks.get(r, 0) | (1 << r)) & want != want:
                return
        self.full_membership_t = now


def beacon_size(b: dict) -> int:
    return len(json.dumps(b, separators=(",", ":")))
