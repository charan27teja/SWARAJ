"""The robot agent. Identical code on every robot; pure and runtime-agnostic:

    cmd, outbox = agent.step(now, dt, own_pose, lidar_scan, inbox, battery)

* `own_pose` (x, y, theta) and `battery` come from the robot's own sensors (world stand-in).
* `lidar_scan` is {"ranges": float32[K], "pose": (x, y, th), "t": capture time} or None.
* `inbox` is a list of (src, body) messages from peers delivered by the transport.
* `cmd` is (v, w) for the differential drive.
* `outbox` is a list of (cls, dst, body): cls "B" best-effort broadcast beacon, "E" reliable
  CoordEvent (dst None = every fleet peer), "O" observer-only (dashboard / metrics) alert.

No sockets, threads or wall clock in here. The agent knows only its own state plus what it hears.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass

import numpy as np

from amr.core.config import Config, DEFAULT
from amr.core.geometry import wrap, project_on_segment, point_box_dist
from amr.core.grid_map import Cell, GridMap
from amr.agent import orca
from amr.agent.deadlock import find_cycle, choose_victim, priority, prio_key
from amr.agent.locks import LockManager
from amr.agent.peers import PeerTable
from amr.agent.planner import RouteGraph, DStarLite, sipp, evaluate_path, static_dist_from, INF
from amr.agent.safety import Lidar, forward_clearance, forward_clearance_points, max_safe_speed
from amr.agent.tasks import CBBA, SequentialAuction, Task, TaskBook, better

MODES = ("idle", "moving", "waiting", "yielding", "charging", "working")

_NEAR_WALL: dict = {}


def _near_wall_grid(gm: GridMap) -> np.ndarray:
    """Boolean grid at 5 cm: True where a point lies within 6 cm of a wall/rack (static map
    data, identical for every robot, so it is computed once per map)."""
    g = _NEAR_WALL.get(id(gm))
    if g is None:
        res, e = 20, 0.06
        H, W = gm.occ.shape
        ys = (np.arange(H * res) + 0.5) / res
        xs = (np.arange(W * res) + 0.5) / res
        X, Y = np.meshgrid(xs, ys)
        g = np.zeros(X.shape, dtype=bool)
        for ox in (-e, 0.0, e):
            for oy in (-e, 0.0, e):
                ix = np.floor(X + ox).astype(int)
                iy = np.floor(Y + oy).astype(int)
                oob = (ix < 0) | (ix >= W) | (iy < 0) | (iy >= H)
                g |= oob | gm.occ[np.clip(iy, 0, H - 1), np.clip(ix, 0, W - 1)]
        _NEAR_WALL[id(gm)] = g
    return g


@dataclass
class Job:
    kind: str                     # task | charge | park
    tid: int | None = None
    stage: str = ""               # to_pick, pick, to_drop, drop, to_dock, charging, to_spot, parked
    target: Cell | None = None
    until: float = 0.0
    bid: float = 0.0


@dataclass
class Run:
    sid: int
    i0: int
    i1: int


class Agent:
    def __init__(self, rid: int, gmap: GridMap, cfg: Config = DEFAULT, fleet=(), tasks=(),
                 alloc: str = "cbba", seed: int = 0):
        self.id = rid
        self.map = gmap
        self.cfg = cfg
        self.fleet = set(fleet) - {rid}
        self.rng = random.Random(seed * 1009 + rid)
        self.peers = PeerTable(rid, cfg.stale_s)
        self.lm = LockManager(rid)
        self.graph = RouteGraph(gmap, cfg)
        self.lidar = Lidar(cfg.lidar_rays, cfg.lidar_range)
        self._dock_cache: dict[Cell, float] = {}
        self._rows: dict[Cell, dict] = {}
        self.book = TaskBook(rid, cfg, self._travel, self._travel, self._dock_dist)
        for t in tasks:
            self.book.add(Task(t.tid, tuple(t.pick), tuple(t.drop), t.release))
            if t.assign is not None:
                self.book.committed[t.tid] = (int(t.assign), 1e9)
        self.alloc = CBBA(self.book) if alloc == "cbba" else SequentialAuction(self.book)
        self.book.admissible = self._admissible
        self._inadmissible_since: dict[int, float] = {}
        self.alloc_kind = alloc
        self.pockets = set(gmap.pockets)
        self.spots = list(gmap.docks) + list(gmap.bays)
        # --- state
        self.now = 0.0
        self.t0: float | None = None
        self.x = self.y = self.th = 0.0
        self.vx = self.vy = 0.0
        self.battery = 100.0
        self.job: Job | None = None
        self.goal: Cell | None = None
        self.path: list[Cell] = []
        self.runs: list[Run] = []
        self.idx = 0
        self.s_now = 0.0
        self.plan_eta = INF
        self.plan_t = -1e9
        self.plan_version = -1
        self.dstar: dict[Cell, DStarLite] = {}
        self.blockages: dict[Cell, float] = {}      # cell -> expiry (local clock)
        self.occ_unknown: set[int] = set()
        self.alerted_lease: set[tuple[int, int]] = set()
        self.sensor_ok: set[int] = set()
        self.sensor_fail_since: dict[int, float] = {}
        self.dyn_pts = np.zeros((0, 2))
        self.unexplained = np.zeros((0, 2))
        self.scan: dict | None = None
        self.live: set[int] = set()
        self.lost_since: dict[int, float] = {}
        self.orphaned: set[tuple[int, str]] = set()
        self.epoch = 0
        self.degraded = False
        self.wait_for: int | None = None
        self.wait_sid: int | None = None
        self.wait_origin: float | None = None
        self.gate_wait_since: float | None = None
        self.yield_state: dict | None = None
        self.cycle_seen: dict[frozenset, int] = {}
        self.next_deadlock_check = 0.0
        self.mode = "idle"
        self.seq = 0
        self.next_beacon = 0.0
        self.next_bid = 0.0
        self.next_build = 0.0
        self.progress_mark = (0.0, 0.0)
        self.pk: Cell | None = None
        self.alerts: list[dict] = []
        self.stats = {"yields": 0, "replans": 0, "blockages_sent": 0, "safety_stops": 0,
                      "deadlocks_detected": 0, "commits": 0}
        self.last_cmd = (0.0, 0.0)
        self.safety_limited = False
        self._odometer = 0.0
        self._respot_t = -1e9
        self.boost_until = -1e9
        self._pos_hist: list = []
        self.phys_block: int | None = None
        self.phys_since: float | None = None
        self._cong_t = -1e9

    # ================================================================== helpers
    def _travel(self, a: Cell, b: Cell) -> float:
        if a == b:
            return 0.0
        row = self._rows.get(a)
        if row is None:
            row = self._rows[a] = static_dist_from(self.graph, a)
        return row.get(b, 1e6)

    def _dock_dist(self, c: Cell) -> float:
        d = self._dock_cache.get(c)
        if d is None:
            d = min(static_dist_from(self.graph, c).get(k, 1e6) for k in self.map.docks)
            self._dock_cache[c] = d
        return d

    def _ci(self, c: Cell) -> int:
        return c[1] * self.map.width + c[0]

    def _cell(self, i: int) -> Cell:
        return (i % self.map.width, i // self.map.width)

    def _here(self) -> Cell:
        c = self.map.cell_of(self.x, self.y)
        return c if c in self.map.free else self.map.nearest_free(self.x, self.y)

    def _alert(self, typ: str, sev: str, msg: str, robots=None, x=None, y=None, **extra):
        a = {"k": "ALERT", "type": typ, "sev": sev, "msg": msg, "robots": robots or [self.id],
             "x": round(self.x if x is None else x, 2), "y": round(self.y if y is None else y, 2),
             "t": round(self.now, 2), "by": self.id}
        a.update(extra)
        self.alerts.append(a)

    def _dist_to_section(self, sid: int) -> float:
        best = INF
        for (cx, cy) in self.map.sections[sid].cells:
            d, _, _ = point_box_dist(self.x, self.y, cx, cy, cx + 1, cy + 1)
            if d < best:
                best = d
        return best

    def _prio(self) -> float:
        base = 1.0 if (self.job and self.job.kind == "task" and self.job.stage in ("to_drop", "drop")) else 0.0
        if self.job and self.job.kind == "charge":
            base += 0.5
        tw = 0.0 if self.wait_origin is None else self.now - self.wait_origin
        if self.now < self.boost_until:
            base += 100.0           # boxed in: cannot be the victim, someone else must yield
        return priority(base, tw, self.cfg.alpha_age)

    # ================================================================== main step
    def step(self, now: float, dt: float, own_pose, lidar_scan, inbox, battery: float = 100.0):
        self.now = now
        if self.t0 is None:
            self.t0 = now
            self.x, self.y, self.th = own_pose
            self.progress_mark = (now, 0.0)
        px, py = self.x, self.y
        self.x, self.y, self.th = own_pose
        if dt > 0:
            a = 0.5
            self.vx = (1 - a) * self.vx + a * (self.x - px) / dt
            self.vy = (1 - a) * self.vy + a * (self.y - py) / dt
        self.battery = battery
        out: list = []
        for src, body in inbox:
            self._on_msg(src, body)
        self._membership()
        self._expire_blockages()
        if lidar_scan is not None:
            self._on_scan(lidar_scan)
        self._leases()
        self._tasks(out)
        self._route()
        stop_s = self._gates()
        self._deadlock()
        cmd = self._motion(stop_s, dt)
        self._send_bids(out)
        out.extend(self.lm.drain())
        if now >= self.next_beacon:
            self.next_beacon = now + 1.0 / self.cfg.beacon_hz
            out.append(("B", None, self._beacon()))
        for a in self.alerts:
            out.append(("O", None, a))
        self.alerts = []
        self.last_cmd = cmd
        return cmd, out

    # ================================================================== messages
    def _on_msg(self, src: int, m: dict) -> None:
        k = m.get("k")
        if k == "B":
            if self.peers.on_beacon(m, self.now):
                self.lm.observe(m.get("lc", 0))
        elif k in ("REQ", "REP", "REL"):
            self.lm.on_message(src, m)
        elif k == "BID" or k == "SBID":
            if isinstance(self.alloc, CBBA) and k == "BID":
                self.alloc.on_bid_message(src, m, self.now)
            elif isinstance(self.alloc, SequentialAuction) and k == "SBID":
                self.alloc.on_bid_message(src, m, self.now)
        elif k == "CLAIM":
            self._on_claim(src, int(m["j"]), float(m["y"]))
        elif k == "DONE":
            j = int(m["j"])
            self.book.done.add(j)
            self.book.committed.pop(j, None)
            self.alloc.forget(j)
        elif k == "UNCLAIM":
            j = int(m["j"])
            if self.book.committed.get(j, (None,))[0] == src:
                del self.book.committed[j]
        elif k == "BLOCK":
            self._cong_t = -1e9                      # re-price bids at the next round
            ttl = float(m.get("ttl", self.cfg.blockage_ttl_s))
            for c in m["cells"]:
                c = tuple(c)
                if c != self._here():
                    self.blockages[c] = max(self.blockages.get(c, 0.0), self.now + ttl)

    def _on_claim(self, k: int, j: int, y: float) -> None:
        mine = self.job and self.job.kind == "task" and self.job.tid == j
        if mine:
            if j in self.book.picked:
                return                     # I already have the item: keep (possible duplicate)
            if better(y, k, self.job.bid, self.id):
                self._alert("task_conflict", "info", f"R{self.id} yields task {j} to R{k}", [self.id, k])
                self.book.committed[j] = (k, y)
                self.alloc.forget(j)
                self._set_job(None)
            return
        cur = self.book.committed.get(j)
        if cur is None or cur[0] == k or better(y, k, cur[1], cur[0]):
            self.book.committed[j] = (k, y)
        self.alloc.forget(j)

    # ================================================================== membership
    def _membership(self) -> None:
        now = self.now
        live = self.peers.live(now)
        if live != self.live:
            self.epoch += 1
            for p in live - self.live:
                self.lm.on_peer_joined(p)
                if isinstance(self.alloc, CBBA):
                    self.alloc.dirty = True
                if p in self.lost_since:
                    self._alert("peer_back", "info", f"R{self.id} hears R{p} again", [p])
                self.lost_since.pop(p, None)
                self.orphaned = {o for o in self.orphaned if o[0] != p}
            for p in self.live - live:
                self.lm.on_peer_lost(p)
                self.lost_since[p] = now
                self._alert("robot_lost", "warn", f"R{self.id} lost contact with R{p}", [p])
            self.live = live
        for p, t in self.lost_since.items():
            lost_for = now - t
            if lost_for >= self.cfg.orphan_bundle_s and (p, "b") not in self.orphaned:
                self.orphaned.add((p, "b"))
                self.alloc.orphan(p, now)
            if lost_for >= self.cfg.orphan_commit_s and (p, "c") not in self.orphaned:
                self.orphaned.add((p, "c"))
                gone = [j for j, (r, _) in self.book.committed.items() if r == p and j not in self.book.done]
                for j in gone:
                    del self.book.committed[j]
                if gone:
                    self._alert("task_reauctioned", "warn", f"tasks {gone} of lost R{p} re-auctioned", [p],
                                tasks=gone)
                    if isinstance(self.alloc, CBBA):
                        self.alloc.dirty = True
        never_heard = self.fleet - self.peers.ever()
        startup = (now - (self.t0 or now)) < 2.0 and bool(never_heard)
        degraded = bool(self.lost_since) or startup
        if degraded != self.degraded:
            self.degraded = degraded
            self.graph.set_degraded(degraded)
            self.sensor_ok.clear()

    # ================================================================== blockages
    def _expire_blockages(self) -> None:
        now = self.now
        for c in [c for c, t in self.blockages.items() if t <= now]:
            del self.blockages[c]
        cells = set(self.blockages)
        changed = self.graph.set_blocked(cells)
        if changed:
            for ds in self.dstar.values():
                ds.notify(changed)
            if self.path and any(c in changed for c in self.path[self.idx:]):
                self.plan_t = -1e9      # force replan

    def _publish_blockage(self, cells: list[Cell], reason: str) -> None:
        self._cong_t = -1e9
        cells = [c for c in cells if c in self.map.free and c != self._here()]
        if not cells:
            return
        ttl = self.cfg.blockage_ttl_s
        for c in cells:
            self.blockages[c] = self.now + ttl
        self._pending_events.append(("E", None, {"k": "BLOCK", "cells": [list(c) for c in cells],
                                                 "ts": round(self.now, 2), "conf": 0.9, "ttl": ttl}))
        self.stats["blockages_sent"] += 1
        cx, cy = cells[0]
        self._alert("aisle_blocked", "warn", f"R{self.id}: {reason} at {cells[0]}", [self.id],
                    x=cx + 0.5, y=cy + 0.5, cells=[list(c) for c in cells])
        self.plan_t = -1e9

    @property
    def _pending_events(self) -> list:
        # blockage events ride on the lock manager's reliable queue
        return self.lm.out

    # ================================================================== perception
    def _on_scan(self, scan: dict) -> None:
        self.scan = scan
        r = scan["ranges"]
        sx, sy, sth = scan["pose"]
        valid = r < self.lidar.max_range - 1e-3
        ang = sth + self.lidar.angles[valid]
        rr = r[valid].astype(float)
        px = sx + rr * np.cos(ang)
        py = sy + rr * np.sin(ang)
        nw = _near_wall_grid(self.map)
        gx = np.clip((px * 20.0).astype(int), 0, nw.shape[1] - 1)
        gy = np.clip((py * 20.0).astype(int), 0, nw.shape[0] - 1)
        near = nw[gy, gx]
        dyn = np.stack([px[~near], py[~near]], axis=1) if (~near).any() else np.zeros((0, 2))
        self.dyn_pts = dyn
        if len(dyn) and self.live:
            peers = [self.peers.predicted(p, self.now) for p in self.live]
            pp = np.array([(q[0], q[1]) for q in peers if q is not None])
            if len(pp):
                d = np.sqrt(((dyn[:, None, :] - pp[None, :, :]) ** 2).sum(-1)).min(1)
                dyn = dyn[d > self.cfg.radius + 0.25]
        self.unexplained = dyn

    def _section_clear(self, sid: int, cells_needed: set[Cell], vantage: Cell) -> tuple[bool, list[Cell]]:
        """Own-lidar check that the section is clear. Normal mode: the cells of the section this
        robot will traverse. Degraded mode: the whole section plus the mouth zone."""
        pts = self.dyn_pts
        if len(pts) == 0:
            return True, []
        sec_cells = set(self.map.sections[sid].cells)
        check = sec_cells if self.degraded else (cells_needed & sec_cells or sec_cells)
        hits: list[Cell] = []
        ix = np.floor(pts[:, 0]).astype(int)
        iy = np.floor(pts[:, 1]).astype(int)
        for k in range(len(pts)):
            c = (int(ix[k]), int(iy[k]))
            if c in check and c not in hits:
                hits.append(c)
        if self.degraded and not hits and len(self.unexplained):
            vx, vy = vantage[0] + 0.5, vantage[1] + 0.5
            d = np.hypot(self.unexplained[:, 0] - vx, self.unexplained[:, 1] - vy)
            me = np.hypot(self.unexplained[:, 0] - self.x, self.unexplained[:, 1] - self.y)
            if ((d < 1.0) & (me > 0.05)).any():
                return False, []
        return (not hits), hits

    # ================================================================== leases / occupied-unknown
    def _leases(self) -> None:
        now = self.now
        margin = self.cfg.clock_error_s
        held_live: set[int] = set()
        for rid, p in self.peers.peers.items():
            b = p.beacon
            is_live = rid in self.live
            for sid, remaining in b.get("h", []):
                lease = p.recv_t + remaining          # anchored to *my* clock at reception
                if is_live:
                    held_live.add(sid)
                elif now > lease + margin:
                    if sid not in self.occ_unknown and (rid, sid) not in self.alerted_lease:
                        self.alerted_lease.add((rid, sid))
                        self._alert("lease_expired", "warn",
                                    f"lease of R{rid} on {self.map.sections[sid].label} expired: occupied-unknown",
                                    [rid], sid=sid)
                    self.occ_unknown.add(sid)
        self.occ_unknown -= held_live

    def _holders(self) -> dict[int, int]:
        out = {}
        for rid in self.live:
            b = self.peers.get(rid)
            for sid, lease in b.get("h", []):
                out[sid] = rid
        return out

    # ================================================================== L4 tasks / jobs
    def _set_job(self, job: Job | None) -> None:
        self.job = job
        self.goal = None
        self.path = []
        self.runs = []
        self.plan_t = -1e9

    def _need_charge(self) -> bool:
        c = self.cfg
        here = self._here()
        need = self._dock_dist(here) * c.battery_per_m + c.battery_reserve + 3.0
        return self.battery < need

    def _tasks(self, out: list) -> None:
        now, c = self.now, self.cfg
        bk = self.book
        job = self.job
        # scripted assignments (demo geometry) take effect at release
        if job is None or job.kind == "park":
            for j, (r, y) in list(bk.committed.items()):
                if r == self.id and j not in bk.done and bk.tasks[j].release <= now and j not in bk.picked:
                    self._start_task(j, y, out, announce=False)
                    return
        # allocation (re)build
        if now >= self.next_build and not (self.job and self.job.kind == "charge"):
            self.next_build = now + 0.25
            if now - self._cong_t >= 5.0:
                self._freeze_congestion()
            start, t_free = self._free_point()
            if isinstance(self.alloc, SequentialAuction):
                self.alloc.build(now, start, t_free, self.battery,
                                 idle=(self.job is None or self.job.kind == "park"))
            else:
                self.alloc.build(now, start, t_free, self.battery)
        job = self.job
        self._respot()
        if job is None or job.kind == "park":
            if self._need_charge():
                if job is None or job.kind != "charge":
                    self._start_charge()
                return
            path = self.alloc.path
            if path:
                j = path[0]
                adm = self._admissible(j)
                if bk.available(j, now) and self.alloc.stable(j, now, c.commit_stable_s) and adm:
                    y = self.alloc.y.get(j, 0.0) if isinstance(self.alloc, CBBA) else 1.0
                    self._start_task(j, y, out, announce=True)
                    return
                if not adm:
                    # its aisle / drop is at capacity: don't sit on the bid, let the task go back
                    t0 = self._inadmissible_since.setdefault(j, now)
                    if now - t0 > 1.0 and isinstance(self.alloc, CBBA):
                        self._inadmissible_since.pop(j, None)
                        for jj in list(self.alloc.bundle):
                            self.alloc._reset(jj, now)
                            self.alloc.forget(jj)
                        self.next_build = now
                else:
                    self._inadmissible_since.pop(j, None)
            if job is None:
                self._start_park()
            return
        # progress the current job
        if job.kind == "task":
            self._progress_task(out)
        elif job.kind == "charge":
            if job.stage == "to_dock" and self._at(job.target):
                job.stage = "charging"
            if job.stage == "charging" and self.battery >= c.battery_full:
                self._set_job(None)

    def _freeze_congestion(self) -> None:
        """Congestion enters bids as travel-time cost, frozen for the auction round: expected
        queueing at the destination's aisle / pocket from what peers currently hold, wait for
        or head to."""
        self._cong_t = self.now
        sec_load: dict[int, int] = {}
        pk_load: dict[int, int] = {}
        for rid in self.live:
            b = self.peers.get(rid)
            for sid, _ in b.get("h", []):
                sec_load[sid] = sec_load.get(sid, 0) + 1
            if b.get("wq") is not None:
                sec_load[b["wq"]] = sec_load.get(b["wq"], 0) + 1
            if b.get("pk") is not None:
                pk_load[b["pk"]] = pk_load.get(b["pk"], 0) + 1
        m = self.map
        ci = self._ci
        blocked_secs: dict[int, float] = {}
        for c, exp in self.blockages.items():
            sid = m.section_of(c)
            if sid is not None:
                blocked_secs[sid] = max(blocked_secs.get(sid, 0.0), exp - self.now)

        def cong(a: Cell, b: Cell) -> float:
            sid = m.section_of(b)
            d = 0.0
            if sid is not None:
                d += 6.0 * sec_load.get(sid, 0) + blocked_secs.get(sid, 0.0)
            if b in self.pockets:
                d += 4.0 * pk_load.get(ci(b), 0)
            return d

        self.book.congestion = cong

    def _respot(self) -> None:
        """Two robots chose the same pocket for parking/charging: the loser picks another."""
        job, now = self.job, self.now
        if job is None or job.kind not in ("park", "charge") or job.target not in self.pockets:
            return
        if self._at(job.target, 0.6) or now - self._respot_t < 1.0:
            return
        owner = self._pocket_owner(job.target)
        if owner is None or owner == self.id:
            return
        self._respot_t = now
        cands = [s for s in (self.map.docks if job.kind == "charge" else self.spots) if s != job.target]
        taken = self._claimed_spots()
        free = [s for s in cands if s not in taken]
        if free:
            job.target = min(free, key=lambda s: (self._travel(self._here(), s), s))
            self.goal = None

    def _admissible(self, j: int) -> bool:
        """Admission control: don't head for an aisle (or drop pocket) that already has
        `admit_max_queue` robots committed to it and not yet past it. Counts CLAIMs (reliable,
        immediate) rather than beacon stage, so a burst of simultaneous commits can't exceed it
        by more than the message latency."""
        bk = self.book
        t = bk.tasks[j]
        sid = self.map.section_of(t.pick)
        n_pick = n_drop = 0
        for jj, (r, _) in bk.committed.items():
            if r == self.id or jj in bk.done or r not in self.live:
                continue
            tt = bk.tasks.get(jj)
            if tt is None:
                continue
            b = self.peers.get(r)
            st = b.get("st") if b.get("task") == jj else "to_pick"   # not yet in its beacon
            if st in ("to_pick", "pick") and sid is not None and self.map.section_of(tt.pick) == sid:
                n_pick += 1
            if tt.drop == t.drop and st in ("to_drop", "drop"):
                n_drop += 1                       # only robots already carrying to that drop crowd it
        lim = self.cfg.admit_max_queue
        return n_pick < lim and n_drop < lim

    def _free_point(self) -> tuple[Cell, float]:
        j = self.job
        if j and j.kind == "task":
            t = self.book.tasks[j.tid]
            here = self._here()
            if j.stage in ("to_pick", "pick"):
                eta = self._travel(here, t.pick) + self.cfg.dwell_pick_s + self._travel(t.pick, t.drop)
            else:
                eta = self._travel(here, t.drop)
            return t.drop, self.now + eta + self.cfg.dwell_drop_s
        return self._here(), self.now

    def _start_task(self, j: int, y: float, out: list, announce: bool) -> None:
        self.book.committed[j] = (self.id, y)
        self.alloc.forget(j)
        self._set_job(Job("task", j, "to_pick", self.book.tasks[j].pick, bid=y))
        self.stats["commits"] += 1
        if announce:
            out.append(("E", None, {"k": "CLAIM", "j": j, "y": y}))

    def _unclaim(self, j: int, why: str) -> None:
        """Hand a committed (not yet picked) task back to the fleet for re-auction."""
        self.book.committed.pop(j, None)
        self.lm.out.append(("E", None, {"k": "UNCLAIM", "j": j}))
        self._alert("task_reauctioned", "warn", f"R{self.id} releases task {j}: {why}", tasks=[j])
        self._unreachable_since = None
        if isinstance(self.alloc, CBBA):
            self.alloc.dirty = True
        self._set_job(None)

    def _start_charge(self) -> None:
        if isinstance(self.alloc, CBBA):
            for j in list(self.alloc.bundle):
                self.alloc._reset(j, self.now)
                self.alloc.forget(j)
        dock = self._choose_spot(self.map.docks)
        self._set_job(Job("charge", None, "to_dock", dock))
        self._alert("charging", "info", f"R{self.id} battery {self.battery:.0f}% -> dock {dock}")

    def _start_park(self) -> None:
        here = self._here()
        taken = self._claimed_spots()
        bays_free = [b for b in self.map.bays if b not in taken or b == here]
        spot = self._choose_spot(self.spots if (here in self.map.docks or not bays_free) else self.map.bays)
        self._set_job(Job("park", None, "to_spot", spot))

    def _claimed_spots(self) -> dict[Cell, int]:
        out = {}
        for rid, p in self.peers.peers.items():
            pk = p.beacon.get("pk")
            if pk is not None:
                out[self._cell(pk)] = rid
        return out

    def _choose_spot(self, cands) -> Cell:
        here = self._here()
        taken = self._claimed_spots()
        free = [s for s in cands if s not in taken or s == here]
        pool = free or list(cands)
        return min(pool, key=lambda s: (0 if s == here else 1, self._travel(here, s), s))

    def _at(self, c: Cell | None, tol: float = 0.2) -> bool:
        return c is not None and math.hypot(self.x - c[0] - 0.5, self.y - c[1] - 0.5) < tol

    def _progress_task(self, out: list) -> None:
        job, c, now = self.job, self.cfg, self.now
        t = self.book.tasks[job.tid]
        if job.tid in self.book.done and job.tid not in self.book.picked:
            self._set_job(None)
            return
        if job.stage == "to_pick" and self._at(t.pick) and abs(self.last_cmd[0]) < 0.2:
            job.stage, job.until = "pick", now + c.dwell_pick_s
        elif job.stage == "pick" and now >= job.until:
            self.book.picked.add(job.tid)
            out.append(("O", None, {"k": "PICK", "j": job.tid, "by": self.id, "t": round(now, 2)}))
            job.stage, job.target = "to_drop", t.drop
            self.goal = None
        elif job.stage == "to_drop" and self._at(t.drop) and abs(self.last_cmd[0]) < 0.2:
            job.stage, job.until = "drop", now + c.dwell_drop_s
        elif job.stage == "drop" and now >= job.until:
            out.append(("E", None, {"k": "DONE", "j": job.tid, "t": round(now, 2)}))
            self.book.done.add(job.tid)
            self.book.committed.pop(job.tid, None)
            self._set_job(None)

    # ================================================================== L3 route
    def _goal_cell(self) -> Cell | None:
        if self.yield_state is not None:
            return self.yield_state["target"]
        j = self.job
        if j is None:
            return None
        return j.target

    def _dstar_for(self, goal: Cell) -> DStarLite:
        ds = self.dstar.get(goal)
        if ds is None:
            if len(self.dstar) > 12:
                self.dstar.pop(next(iter(self.dstar)))
            ds = DStarLite.from_template(self.graph, goal)
            self.dstar[goal] = ds
        elif getattr(ds, "_deg", None) != self.graph.degraded:
            ds = DStarLite.from_template(self.graph, goal)   # degraded rule flips many edges: rebuild
            self.dstar[goal] = ds
        ds._deg = self.graph.degraded
        return ds

    def _can_wait(self, c: Cell) -> bool:
        return c in self.map.wide or c in self.pockets

    def _reservations(self) -> dict[Cell, list[tuple[float, float]]]:
        now, cfg = self.now, self.cfg
        m = self.map
        res: dict[Cell, list] = {}
        pad = cfg.clock_error_s
        if not cfg.use_reservations:
            return res

        def add(sid, a, b):
            for c in m.sections[sid].cells:
                res.setdefault(c, []).append((a - pad, b + pad))

        my_key = prio_key(self._prio(), self.id)
        for rid, p in self.peers.peers.items():
            b = p.beacon
            is_live = rid in self.live
            for sid, remaining in b.get("h", []):
                lease = p.recv_t + remaining
                if is_live or lease + pad > now:
                    add(sid, now, max(now + 0.5, lease))
            if not is_live:
                continue
            wq = b.get("wq")
            if wq is not None and rid in self.lm.queue_ahead(wq, self.live):
                add(wq, now, now + 2.0 * (len(m.sections[wq].cells) / cfg.v_max + 1.0))
            if prio_key(b.get("pr", 0.0), rid) > my_key:
                pp = b.get("pp", [])
                x0, y0 = b["p"][0], b["p"][1]
                t = now
                last = (x0, y0)
                cur = None
                t_in = 0.0
                for ci in pp:
                    c = self._cell(ci)
                    cx, cy = c[0] + 0.5, c[1] + 0.5
                    t += math.hypot(cx - last[0], cy - last[1]) / cfg.v_max
                    last = (cx, cy)
                    sid = m.section_of(c)
                    if sid != cur:
                        if cur is not None:
                            add(cur, t_in - 1.0, t + 1.0)
                        cur, t_in = sid, t
                if cur is not None:
                    add(cur, t_in - 1.0, t + 1.0 + len(m.sections[cur].cells))
        for sid in self.occ_unknown:
            add(sid, now, now + 15.0)
        return res

    def _route(self) -> None:
        now = self.now
        goal = self._goal_cell()
        if goal is None:
            self.path, self.runs, self.goal = [], [], None
            return
        waiting_long = (self.gate_wait_since is not None and now - self.gate_wait_since >= self.cfg.replan_wait_s
                        and now - self.plan_t >= self.cfg.replan_wait_s)
        stuck = self._stuck()
        if goal == self.goal and self.path and self.plan_version == self.graph.version \
                and not waiting_long and not stuck and self.plan_t > -1e8:
            return
        here = self._here()
        ds = self._dstar_for(goal)
        if ds.version != self.graph.version:
            ds.notify(set(self.graph.blocked) | set(self.blockages))
        ds.compute(here)
        res = self._reservations()
        r = sipp(self.graph, here, goal, now, res, ds.cost_to_go, self._can_wait, max_expansions=6000)
        if r is None:
            p = ds.path(here)
            eta = INF if p is None else now + ds.cost_to_go(here)
        else:
            p, eta = [c for c, _ in r[0]], r[1]
        self.plan_t = now
        self.plan_version = self.graph.version
        if p is None:
            # unreachable for now (blocked / degraded): keep still and retry shortly
            self._unreachable_since = getattr(self, "_unreachable_since", None) or now
            j = self.job
            if (j is not None and j.kind == "task" and j.stage == "to_pick" and j.tid not in self.book.picked
                    and now - self._unreachable_since > 5.0):
                self._unclaim(j.tid, "pick unreachable (blocked)")
                return
            if self.path:
                self._cancel_unentered()
            self.path, self.runs, self.goal = [], [], goal
            self.plan_t = now - self.cfg.replan_wait_s + 2.0
            return
        self._unreachable_since = None
        if waiting_long and goal == self.goal and self.path and not stuck:
            cur = evaluate_path(self.graph, self.path[self.idx:], now, res, self._can_wait)
            if not (eta < cur - self.cfg.replan_gain_s):
                return                                  # keep current route (hysteresis)
        if stuck:
            self.progress_mark = (now, self._odometer)
        self.stats["replans"] += 1
        self._set_path(p, goal, eta)

    def _set_path(self, p: list[Cell], goal: Cell, eta: float) -> None:
        self.path = p
        self.goal = goal
        self.plan_eta = eta
        self.idx = 0
        self.s_now = 0.0
        runs: list[Run] = []
        for i, c in enumerate(p):
            sid = self.map.section_of(c)
            if sid is None:
                continue
            if runs and runs[-1].sid == sid and runs[-1].i1 == i - 1:
                runs[-1].i1 = i
            else:
                runs.append(Run(sid, i, i))
        self.runs = runs
        wanted = {r.sid for r in runs}
        for sid, r in list(self.lm.mine.items()):
            if not r.entered and sid not in wanted:
                self.lm.release(sid)
        self.sensor_ok &= wanted
        self._project()

    def _cancel_unentered(self) -> None:
        for sid, r in list(self.lm.mine.items()):
            if not r.entered:
                self.lm.release(sid)
        self.sensor_ok.clear()

    def _stuck(self) -> bool:
        """No progress for stuck_s while supposed to be moving (not waiting at a gate/dwelling)."""
        t, mark = self.progress_mark
        odo = self._odometer
        if odo - mark > 0.3:
            self.progress_mark = (self.now, odo)
            return False
        intentional = (self.gate_wait_since is not None or not self.path
                       or (self.job and self.job.stage in ("pick", "drop", "charging", "parked"))
                       or self._at(self.goal, 0.3))
        if intentional:
            self.progress_mark = (self.now, odo)
            return False
        return self.now - t > self.cfg.stuck_s

    # ================================================================== L2 gates
    def _project(self) -> None:
        p = self.path
        if len(p) < 2:
            self.idx, self.s_now = 0, 0.0
            return
        best = (INF, self.idx, 0.0)
        lo = max(0, self.idx - 1)
        hi = min(len(p) - 1, self.idx + 4)
        for k in range(lo, hi):
            a, b = p[k], p[k + 1]
            t, _, _, d = project_on_segment(self.x, self.y, a[0] + .5, a[1] + .5, b[0] + .5, b[1] + .5)
            if d < best[0] - 1e-9:
                best = (d, k, t)
        _, k, t = best
        self.idx = k
        self.s_now = k + t

    def _gates(self) -> float:
        """Drive the L2 protocol along the path; return the arc-length where the robot must stop."""
        cfg, now = self.cfg, self.now
        self.wait_for = None
        self.wait_sid = None
        # release sections we have left
        for sid, r in list(self.lm.mine.items()):
            d = self._dist_to_section(sid)
            if not r.entered and r.granted and d < cfg.radius:
                r.entered = True
            if r.entered and d >= cfg.release_clearance:
                self.lm.release(sid)
                self.sensor_ok.discard(sid)
            elif r.entered:
                r.lease = now + self._lease_estimate(sid)
        if not self.path:
            self._gate_wait(False)
            return 0.0
        self._project()
        self.lm.update_grants(self.live, self._holders(), now)
        for sid, r in list(self.lm.mine.items()):
            if r.granted and not r.entered and now - r.t_grant > cfg.grant_timeout_s \
                    and self._dist_to_section(sid) > 0.6:
                self.lm.release(sid)          # stuck away from the entry: hand the grant back
                self.sensor_ok.discard(sid)
        p = self.path
        end_s = float(len(p) - 1)
        stop_s = end_s
        here_sid = self.map.section_of(self._here())
        blocked_at_gate = False
        for n, run in enumerate(self.runs):
            if self.s_now >= run.i1 + 0.5 and run.sid != here_sid:
                rb = self.lm.mine.get(run.sid)
                if rb is not None and not rb.entered:
                    self.lm.release(run.sid)          # passed it: never keep a stale grant
                continue
            r = self.lm.mine.get(run.sid)
            if r is not None and r.entered:
                continue
            if run.i0 == 0:
                # path starts inside this section (e.g. after re-plan inside an aisle)
                if r is None:
                    self.lm.request(run.sid, now)
                    r = self.lm.mine[run.sid]
                if not r.granted:
                    stop_s = min(stop_s, self.s_now)
                    blocked_at_gate = self._set_wait(run.sid)
                    break
                r.entered = True
                continue
            b = run.i0 - 0.5
            if b - self.s_now > cfg.lock_request_dist:
                break
            # the exit section is secured before the intersection ("don't block the box")
            nxt = self.runs[n + 1] if n + 1 < len(self.runs) else None
            needed = [run]
            if self.map.sections[run.sid].kind == "intersection" and nxt and nxt.i0 == run.i1 + 1:
                needed = [nxt, run]
            prev_inside = (n > 0 and self.runs[n - 1].i1 == run.i0 - 1
                           and self.lm.mine.get(self.runs[n - 1].sid) is not None
                           and self.lm.mine[self.runs[n - 1].sid].entered)
            ok = True
            for x in needed:
                rx = self.lm.mine.get(x.sid)
                if rx is None:
                    self.lm.request(x.sid, now)
                    ok = False
                    break
                if not rx.granted:
                    ok = False
                    break
            if not ok:
                if b < 0 or self.s_now > b - 0.05:
                    # already at/inside the boundary without a grant (e.g. after re-plan): hold here
                    stop_s = min(stop_s, self.s_now)
                elif prev_inside:
                    stop_s = min(stop_s, max(self.s_now, b - cfg.entry_stop_dist))
                else:
                    q = len(self.lm.queue_ahead(x.sid, self.live))
                    target = self._clear_of_mouths(b - cfg.hold_dist - min(3.0, 1.0 * q))
                    if target < self.s_now - 0.05:
                        target = max(self.s_now, min(target + 0.6, b - cfg.entry_stop_dist))
                    stop_s = min(stop_s, target)
                blocked_at_gate = self._set_wait(x.sid)
                break
            # granted: own-lidar confirmation from the vantage cell before the section
            if run.sid not in self.sensor_ok:
                vantage = p[run.i0 - 1]
                if math.hypot(self.x - vantage[0] - 0.5, self.y - vantage[1] - 0.5) <= cfg.vantage_tol:
                    need_cells = set(p[run.i0:run.i1 + 1])
                    clear, hits = self._section_clear(run.sid, need_cells, vantage)
                    if clear:
                        self.sensor_ok.add(run.sid)
                        self.sensor_fail_since.pop(run.sid, None)
                        if run.sid in self.occ_unknown:
                            self.occ_unknown.discard(run.sid)
                            self._alert("clear_confirmed", "info",
                                        f"R{self.id} lidar-confirmed {self.map.sections[run.sid].label} clear",
                                        sid=run.sid)
                    else:
                        t0 = self.sensor_fail_since.setdefault(run.sid, now)
                        if now - t0 >= cfg.sensor_block_s:
                            self.sensor_fail_since.pop(run.sid, None)
                            self._publish_blockage(hits or [p[run.i0]], "lidar sees aisle occupied")
                            self.lm.release(run.sid)
                            if run.sid != needed[0].sid:
                                self.lm.release(needed[0].sid)
                            stop_s = min(stop_s, max(self.s_now, b - cfg.entry_stop_dist))
                            blocked_at_gate = True
                            break
                if run.sid not in self.sensor_ok:
                    stop_s = min(stop_s, max(self.s_now, b - cfg.entry_stop_dist))
                    self.wait_sid = run.sid
                    blocked_at_gate = True
                    break
        # pocket (drop/dock/bay) claims: wait outside if a peer has the pocket
        goal = self.path[-1]
        if goal in self.pockets and not blocked_at_gate:
            owner = self._pocket_owner(goal)
            if owner is not None and owner != self.id:
                b = end_s - 0.5
                q = self._pocket_queue_pos(goal)
                target = self._clear_of_mouths(b - cfg.hold_dist - min(3.0, 1.0 * q))
                if target < self.s_now - 0.05:
                    target = self.s_now
                stop_s = min(stop_s, target)
                self.wait_for = owner
                blocked_at_gate = True
        self._gate_wait(blocked_at_gate and stop_s - self.s_now < 0.6)
        return stop_s

    def _path_point(self, s: float) -> tuple[float, float]:
        p = self.path
        if len(p) == 1:
            return p[0][0] + .5, p[0][1] + .5
        s = max(0.0, min(s, len(p) - 1.0))
        k = min(int(math.floor(s)), len(p) - 2)
        f = s - k
        a, b = p[k], p[k + 1]
        return a[0] + .5 + f * (b[0] - a[0]), a[1] + .5 + f * (b[1] - a[1])

    def _clear_of_mouths(self, s: float) -> float:
        """Latest arc-length <= s where a waiting robot does not sit on a mouth cell."""
        mouths = self.map.mouths
        lo = self.s_now
        while s > lo:
            px, py = self._path_point(s)
            cx, cy = int(math.floor(px)), int(math.floor(py))
            bad = False
            for mx in (cx - 1, cx, cx + 1):
                for my in (cy - 1, cy, cy + 1):
                    if (mx, my) in mouths and math.hypot(px - mx - .5, py - my - .5) < 0.8:
                        bad = True
                        break
                if bad:
                    break
            if not bad:
                return s
            s -= 0.1
        return s

    def _pocket_queue_pos(self, pocket: Cell) -> int:
        """Number of live claimants of the pocket that are closer to it than this robot."""
        me = math.hypot(self.x - pocket[0] - .5, self.y - pocket[1] - .5)
        n = 0
        for rid in self.live:
            b = self.peers.get(rid)
            if b.get("pk") == self._ci(pocket):
                d = math.hypot(b["p"][0] - pocket[0] - .5, b["p"][1] - pocket[1] - .5)
                if (d, rid) < (me, self.id):
                    n += 1
        return n

    def _pocket_owner(self, pocket: Cell) -> int | None:
        """Who gets the pocket: whoever is inside, else the closer claimant (tie: lower id)."""
        cands = [(math.hypot(self.x - pocket[0] - .5, self.y - pocket[1] - .5), self.id)]
        for rid in self.live:
            b = self.peers.get(rid)
            d = math.hypot(b["p"][0] - pocket[0] - .5, b["p"][1] - pocket[1] - .5)
            if b.get("pk") == self._ci(pocket) or d < 0.7:
                cands.append((d, rid))
        # stale robots physically in the pocket count too
        for rid, p in self.peers.peers.items():
            if rid not in self.live:
                bx, by = p.beacon["p"][0], p.beacon["p"][1]
                if math.hypot(bx - pocket[0] - .5, by - pocket[1] - .5) < 0.6:
                    return rid
        inside = [c for c in cands if c[0] < 0.6]
        pool = inside or cands
        return min(pool)[1]

    def _set_wait(self, sid: int) -> bool:
        self.wait_sid = sid
        missing = self.lm.missing(sid, self.live)
        holders = self._holders()
        h = holders.get(sid)
        if h is not None and h in missing:
            self.wait_for = h
        elif missing:
            pr = self.lm.peer_reqs.get(sid, {})
            ahead = [q for q in missing if q in pr]
            self.wait_for = min(ahead, key=lambda q: (pr[q], q)) if ahead else min(missing)
        return True

    def _wait_for_eff(self) -> int | None:
        return self.wait_for if self.wait_for is not None else self.phys_block

    def _gate_wait(self, waiting: bool) -> None:
        if waiting:
            if self.gate_wait_since is None:
                self.gate_wait_since = self.now
            if self.wait_origin is None:
                self.wait_origin = self.now
        else:
            if self.gate_wait_since is not None and self.yield_state is None:
                self.wait_origin = None
            self.gate_wait_since = None

    def _lease_estimate(self, sid: int) -> float:
        cfg = self.cfg
        rem = 0.0
        if self.path:
            cells = set(self.map.sections[sid].cells)
            rem = sum(1 for c in self.path[self.idx:] if c in cells) / cfg.v_max
            if self.job and self.job.kind == "task" and self.job.stage in ("to_pick", "pick") \
                    and self.book.tasks[self.job.tid].pick in cells:
                rem += cfg.dwell_pick_s
        return rem + cfg.lease_margin_s

    # ================================================================== L2 deadlock
    def _deadlock(self) -> None:
        now, cfg = self.now, self.cfg
        if self.yield_state is not None:
            self._continue_yield()
            return
        if now < self.next_deadlock_check:
            return
        self.next_deadlock_check = now + 1.0 / cfg.deadlock_check_hz
        edges = {self.id: self._wait_for_eff()}
        prios = {self.id: self._prio()}
        for rid in self.live:
            b = self.peers.get(rid)
            edges[rid] = b.get("wf")
            prios[rid] = b.get("pr", 0.0)
        cyc = find_cycle(edges, self.id)
        if not cyc or self.id not in cyc:
            self.cycle_seen.clear()
            return
        key = frozenset(cyc)
        n = self.cycle_seen.get(key, 0) + 1
        self.cycle_seen = {key: n}
        if n == 2:
            self.stats["deadlocks_detected"] += 1
            self._alert("deadlock_detected", "crit", "wait-for cycle " + " -> ".join(f"R{r}" for r in cyc),
                        list(cyc), cycle=list(cyc))
        if n < 3:                       # persisted across 3 checks (~0.4 s): not a transient view
            return
        victim = choose_victim(cyc, prios)
        if victim == self.id:
            self._start_yield(cyc)

    def _start_yield(self, cyc: list[int]) -> None:
        cfg = self.cfg
        self.stats["yields"] += 1
        self.cycle_seen.clear()
        entered = {sid for sid, r in self.lm.mine.items() if r.entered}
        self._cancel_unentered()
        allowed_sections = entered
        target, path = self._backoff_path(allowed_sections)
        if not path or len(path) < 2 or target == self._here():
            # boxed in (dead-end pocket, crowd): yielding is physically impossible, so step out of
            # the victim role; the next-lowest member of the cycle will yield instead
            self.boost_until = self.now + 15.0
            self._alert("yield_impossible", "info", f"R{self.id} is boxed in; another robot must yield",
                        list(cyc), cycle=list(cyc))
            return
        self._alert("deadlock_broken", "crit",
                    f"deadlock {' -> '.join(f'R{r}' for r in cyc)}: R{self.id} (lowest priority) yields",
                    list(cyc), cycle=list(cyc), yielded=self.id)
        j = cfg.backoff_jitter_s
        self.yield_state = {"target": target, "until": self.now + self.rng.uniform(j[0], j[1]),
                            "started": self.now, "x0": self.x, "y0": self.y}
        self.phys_block = None
        self.phys_since = None
        if path:
            self._set_path(path, target, INF)
            self.plan_version = self.graph.version
            self.plan_t = self.now

    def _backoff_path(self, allowed_sections: set[int]) -> tuple[Cell, list[Cell] | None]:
        """BFS to the nearest passing bay through wide cells and sections we already occupy."""
        m = self.map
        start = self._here()
        bays = set(m.bays)
        occupied = set()
        for rid, pi in self.peers.peers.items():
            bx, by = pi.beacon["p"][0], pi.beacon["p"][1]
            occupied.add(m.cell_of(bx, by))
        occupied.discard(start)
        prev = {start: None}
        q = [start]
        found = None
        fallback = None
        qi = 0
        while qi < len(q):
            c = q[qi]
            qi += 1
            if c in bays and c not in self._claimed_spots():
                found = c
                break
            if fallback is None and c in m.wide and len(q) > 3:
                fallback = c
            for n in m.neighbors(c):
                if n in prev or n in self.graph.blocked or n in occupied:
                    continue
                sid = m.section_of(n)
                if sid is not None and sid not in allowed_sections:
                    continue
                if n in self.pockets and n not in bays:
                    continue
                prev[n] = c
                q.append(n)
        tgt = found or fallback or start
        path = [tgt]
        while prev.get(path[-1]) is not None:
            path.append(prev[path[-1]])
        return tgt, path[::-1]

    def _continue_yield(self) -> None:
        ys = self.yield_state
        now = self.now
        in_section = any(r.entered for r in self.lm.mine.values())
        away = self.map.section_of(self._here()) is None and not in_section
        moved = math.hypot(self.x - ys["x0"], self.y - ys["y0"])
        if now - ys["started"] > 4.0 and moved < 0.3:
            # could not back off (boxed in): end the yield and let another member yield next
            self.boost_until = now + 15.0
            self.yield_state = None
            self.plan_t = -1e9
            self.goal = None
            return
        if away and now >= ys["until"]:
            if moved >= 2.0 or self._at(ys["target"], 0.4) or now - ys["started"] > 12.0:
                self.yield_state = None
                self.plan_t = -1e9
                self.goal = None

    # ================================================================== L1 motion
    def _motion(self, stop_s: float, dt: float):
        cfg = self.cfg
        self._odometer += math.hypot(self.vx, self.vy) * dt
        p = self.path
        self._update_mode()
        # preferred velocity along the path
        pref = (0.0, 0.0)
        face = None
        if len(p) >= 2:
            s_c = min(self.s_now + 0.6, len(p) - 1.0)
            k = int(math.floor(s_c))
            f = s_c - k
            if k >= len(p) - 1:
                cx, cy = p[-1][0] + .5, p[-1][1] + .5
            else:
                a, b = p[k], p[k + 1]
                cx, cy = a[0] + .5 + f * (b[0] - a[0]), a[1] + .5 + f * (b[1] - a[1])
            remaining = stop_s - self.s_now
            if stop_s >= len(p) - 1 - 1e-6:
                g = p[-1]
                remaining = max(remaining, math.hypot(self.x - g[0] - .5, self.y - g[1] - .5))
            dxc, dyc = cx - self.x, cy - self.y
            dn = math.hypot(dxc, dyc)
            if dn > 1e-6:
                face = math.atan2(dyc, dxc)
            if remaining > 0.03 and dn > 1e-6:
                v = min(cfg.v_max, math.sqrt(2 * cfg.a_max * 0.8 * max(0.0, remaining - 0.02)))
                if stop_s >= len(p) - 1 - 1e-6:
                    v = min(v, max(0.15, remaining * 1.5))
                pref = (dxc / dn * v, dyc / dn * v)
        elif len(p) == 1:
            g = p[0]
            dxc, dyc = g[0] + .5 - self.x, g[1] + .5 - self.y
            dn = math.hypot(dxc, dyc)
            if dn > 0.03:
                v = min(cfg.v_max, max(0.15, dn * 1.5))
                pref = (dxc / dn * v, dyc / dn * v)
        vx, vy = self._orca(pref, dt)
        cmd = self._to_diff_drive(vx, vy, face, dt)
        self._physical_block(pref, vx, vy, cmd)
        return cmd

    def _physical_block(self, pref, vx, vy, cmd) -> None:
        """Wait-for edge for *physical* blocking (a peer in the way), so the wait-for graph also
        sees deadlocks that no lock explains (e.g. a crowd at an aisle mouth)."""
        want = math.hypot(pref[0], pref[1])
        # actual progress over a 2 s window (robust to rotate/jitter), not the command
        hist = self._pos_hist
        if not hist or self.now - hist[-1][0] >= 0.25:
            hist.append((self.now, self.x, self.y))
            while hist and self.now - hist[0][0] > 2.0:
                hist.pop(0)
        t0, x0, y0 = hist[0]
        got = math.hypot(self.x - x0, self.y - y0) / max(0.5, self.now - t0)
        if want > 0.15 and got < 0.08:
            if self.phys_since is None:
                self.phys_since = self.now
            if self.now - self.phys_since >= 1.5:
                ux, uy = pref[0] / want, pref[1] / want
                best, who = 1.8, None
                for rid, pi in self.peers.peers.items():
                    if self.now - pi.recv_t > 5.0:
                        continue
                    q = self.peers.predicted(rid, self.now)
                    dx, dy = q[0] - self.x, q[1] - self.y
                    d = math.hypot(dx, dy)
                    if d < best and (dx * ux + dy * uy) > 0.3 * d:
                        best, who = d, rid
                self.phys_block = who
                if self.wait_origin is None:
                    self.wait_origin = self.phys_since
                if who is None and self.now - self.phys_since >= 2.0:
                    self._blocked_by_object(ux, uy)
        else:
            self.phys_since = None
            self.phys_block = None

    def _orca(self, pref, dt):
        cfg = self.cfg
        x, y = self.x, self.y
        r = cfg.radius
        obst = []
        rw = r + cfg.wall_margin
        reach = rw + cfg.v_max * cfg.orca_tau_obst + 0.2
        for (cx, cy) in self.map.wall_cells_near(x, y, reach) + self._virtual_walls(x, y, reach):
            d, qx, qy = point_box_dist(x, y, cx, cy, cx + 1, cy + 1)
            if d > reach or d < 1e-9:
                continue
            nx, ny = (x - qx) / d, (y - qy) / d
            if d > rw:
                c = -(d - rw) / cfg.orca_tau_obst
            else:
                c = min(0.5 * cfg.v_max, (rw - d) / max(dt, 0.05))
            obst.append(orca.halfplane_line(nx, ny, c))
        # unknown objects from lidar (network independent): nearest few points
        U = self.unexplained
        if len(U):
            d = np.hypot(U[:, 0] - x, U[:, 1] - y)
            order = np.argsort(d)[:6]
            rp = r + 0.08
            for k in order:
                dk = float(d[k])
                if dk > rp + cfg.v_max * cfg.orca_tau_obst + 0.3 or dk < 1e-6:
                    break
                nx, ny = (x - U[k, 0]) / dk, (y - U[k, 1]) / dk
                # unknown object may itself be moving towards us: take only half the closing budget
                c = -(dk - rp) / (2 * cfg.orca_tau_obst) if dk > rp else min(0.5 * cfg.v_max, (rp - dk) / max(dt, 0.05))
                obst.append(orca.halfplane_line(nx, ny, c))
        agents = []
        ra = r + cfg.orca_margin
        vx0, vy0 = self.vx, self.vy
        for rid, pinfo in self.peers.peers.items():
            age = self.now - pinfo.recv_t
            if age > 5.0:
                continue
            q = self.peers.predicted(rid, self.now)
            if q is None:
                continue
            ox, oy, ovx, ovy = q
            if math.hypot(ox - x, oy - y) > cfg.orca_neighbor_dist + 1.0:
                continue
            if rid in self.live:
                agents.append(orca.agent_line(x, y, vx0, vy0, ox, oy, ovx, ovy, 2 * ra, cfg.orca_tau, max(dt, 0.05)))
            else:
                infl = min(cfg.stale_inflate_max, cfg.stale_inflate_rate * max(0.0, age - cfg.stale_s))
                bx, by = pinfo.beacon["p"][0], pinfo.beacon["p"][1]
                agents.append(orca.agent_line(x, y, vx0, vy0, bx, by, 0.0, 0.0, 2 * ra + infl,
                                              cfg.orca_tau, max(dt, 0.05), responsibility=1.0))
        self._lines = (obst, agents)
        return orca.solve(obst, agents, cfg.v_max, pref[0], pref[1])

    def _blocked_by_object(self, ux: float, uy: float) -> None:
        """Stopped in front of something no peer explains (pallet, dead or partitioned robot):
        publish a BlockageEvent for the cells it occupies and re-plan around it."""
        U = self.unexplained
        if not len(U):
            return
        dx, dy = U[:, 0] - self.x, U[:, 1] - self.y
        d = np.hypot(dx, dy)
        ahead = (d < 1.6) & ((dx * ux + dy * uy) > 0.3 * d)
        if not ahead.any():
            return
        cells = sorted({self.map.cell_of(float(px), float(py)) for px, py in U[ahead]})
        cells = [c for c in cells if c in self.map.free and c not in self.blockages and c != self._here()]
        if cells:
            self._publish_blockage(cells, "obstacle ahead on path")
            self.phys_since = self.now

    def _virtual_walls(self, x: float, y: float, reach: float) -> list[Cell]:
        """Cells of critical sections this robot may not enter right now (not held, or held but
        not yet lidar-confirmed) act as walls for ORCA, so dodging can never push it inside."""
        cs = self.map.cell_section
        allowed = {sid for sid, r in self.lm.mine.items() if r.entered or (r.granted and sid in self.sensor_ok)}
        out = []
        for cy in range(int(math.floor(y - reach)), int(math.floor(y + reach)) + 1):
            for cx in range(int(math.floor(x - reach)), int(math.floor(x + reach)) + 1):
                sid = cs.get((cx, cy))
                if sid is not None and sid not in allowed:
                    out.append((cx, cy))
        return out

    def _to_diff_drive(self, vx, vy, face, dt):
        cfg = self.cfg
        sp = math.hypot(vx, vy)
        if sp < 0.04:
            v = 0.0
            w = 0.0
            if face is not None and self.path and len(self.path) > 1:
                err = wrap(face - self.th)
                if abs(err) > 0.3:
                    w = max(-cfg.w_max, min(cfg.w_max, 2.0 * err))
        else:
            head = math.atan2(vy, vx)
            err = wrap(head - self.th)
            w = max(-cfg.w_max, min(cfg.w_max, 3.0 * err))
            v = sp * math.cos(err) if abs(err) < 0.8 else 0.0
            v = max(0.0, v)
            if v > 0.0:
                # the robot moves along its heading, not along the holonomic ORCA vector: cap the
                # forward speed so the *actual* velocity also satisfies the half-planes
                obst, agents = getattr(self, "_lines", ((), ()))
                hx, hy = math.cos(self.th), math.sin(self.th)
                vo = orca.max_speed_along(obst, hx, hy, v)
                if vo is None:
                    v = 0.0
                else:
                    va = orca.max_speed_along(agents, hx, hy, vo)
                    v = vo if va is None else va
        # L1s protective stop: velocity-scaled field from own lidar only
        self.safety_limited = False
        if self.scan is not None and v > 0.0:
            hw = cfg.radius + cfg.safety_half_width_extra
            lag = max(0.0, self.now - self.scan.get("t", self.now)) * max(v, math.hypot(self.vx, self.vy))
            clr = forward_clearance(self.scan["ranges"], self.lidar, hw) - lag
            vs = max_safe_speed(clr, cfg.a_max, cfg.safety_t_react, cfg.safety_margin)
            # returns no live peer explains may be another robot we cannot hear, approaching as fast
            # as we are: stop within half the gap so that both stopping distances fit
            clu = forward_clearance_points(self.unexplained, self.x, self.y, self.th, hw) - 2 * lag
            vs = min(vs, max_safe_speed(0.5 * clu, cfg.a_max, cfg.safety_t_react, 0.5 * cfg.safety_margin))
            if vs < v:
                v = vs
                self.safety_limited = True
                self.stats["safety_stops"] += 1
        return (v, w)

    def _update_mode(self) -> None:
        j = self.job
        if self.yield_state is not None:
            self.mode = "yielding"
        elif self.gate_wait_since is not None:
            self.mode = "waiting"
        elif j is None:
            self.mode = "idle"
        elif j.kind == "charge" and j.stage == "charging":
            self.mode = "charging"
        elif j.kind == "task" and j.stage in ("pick", "drop"):
            self.mode = "working"
        elif j.kind == "park" and self._at(j.target, 0.3):
            self.mode = "idle"
        else:
            self.mode = "moving"

    # ================================================================== L4 messages
    def _send_bids(self, out: list) -> None:
        if self.now < self.next_bid or not self.alloc.dirty:
            return
        self.next_bid = self.now + 0.4
        if isinstance(self.alloc, CBBA):
            out.append(("E", None, self.alloc.message(self.now)))
        else:
            for m in self.alloc.message(self.now):
                out.append(("E", None, m))

    # ================================================================== beacon
    def _beacon(self) -> dict:
        self.seq += 1
        # leases travel as *remaining seconds*; receivers anchor them to their own clock, so no
        # cross-robot clock comparison is ever needed (clock_error_s pads what remains)
        held = [[sid, round(max(0.0, (r.lease if r.entered else self.now + self._lease_estimate(sid)) - self.now), 2)]
                for sid, r in self.lm.mine.items() if r.granted]
        pp = [self._ci(c) for c in self.path[self.idx:self.idx + 8]] if self.path else []
        live_mask = 0
        for p in self.live:
            if p < 62:
                live_mask |= 1 << p
        j = self.job
        pk = None
        here = self._here()
        g = self._goal_cell()
        if here in self.pockets and self._at(here, 0.7):
            pk = self._ci(here)                 # the pocket I physically occupy
        elif g is not None and g in self.pockets:
            pk = self._ci(g)                    # the pocket I am heading to
        return {
            "k": "B", "id": self.id, "q": self.seq, "lc": self.lm.tick(), "ep": self.epoch,
            "p": [round(self.x, 2), round(self.y, 2), round(self.th, 2)],
            "v": [round(self.vx, 2), round(self.vy, 2)],
            "pp": pp, "h": held, "wf": self._wait_for_eff(), "wq": self.wait_sid,
            "bat": round(self.battery, 1), "task": j.tid if j and j.kind == "task" else None,
            "st": j.stage if j else "", "m": self.mode, "pr": round(self._prio(), 2),
            "lv": live_mask, "pk": pk, "dg": int(self.degraded),
        }
