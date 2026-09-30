"""Baseline B1: centralised stop-and-wait without supervisor deadlock handling.

Simpler than B0: same setup but no active deadlock resolution. Robots just halt and wait;
if a deadlock forms, they wait indefinitely. This is a fairer baseline to compare against
our decentralised system for cases where our system's deadlock breaking adds overhead.

* one central server (network id 0) knows the fleet from 10 Hz robot status reports;
* tasks are dispatched FIFO (release order) to the nearest idle, battery-feasible robot;
* routes are static shortest paths (no time reservations, no reciprocal avoidance);
* traffic is zone control: a robot may only drive over cells the server has granted. A narrow
  aisle / intersection is one zone granted as a whole (with its exit cell). Conflicting requests
  are queued and released first-in-first-out; a robot whose next zone is not granted halts
  and waits (stop-and-wait);
* NO active deadlock supervisor: if a cycle forms, robots remain blocked. Recovery only via
  network link loss triggers re-routing.
* a robot that loses its link to the server halts (fail-safe); after 2 s it creeps along its last
  route at 0.3 m/s protected only by its lidar, until the link returns.
"""
from __future__ import annotations

import math
from collections import deque

import numpy as np

from amr.core.config import Config, DEFAULT
from amr.core.geometry import wrap, project_on_segment, point_box_dist
from amr.core.grid_map import Cell, GridMap
from amr.agent.planner import RouteGraph, DStarLite, static_dist_from, INF
from amr.agent.safety import Lidar, forward_clearance, max_safe_speed

CENTRAL = 0
LOOKAHEAD = 3            # cells granted ahead in wide areas


# =============================================================================== robot
class B1Robot:
    """Thin client: follows the path prefix the server has granted; stops at its end."""

    def __init__(self, rid: int, gmap: GridMap, cfg: Config = DEFAULT):
        self.id = rid
        self.map = gmap
        self.cfg = cfg
        self.lidar = Lidar(cfg.lidar_rays, cfg.lidar_range)
        self.path: list[Cell] = []
        self.granted = 0          # path[:granted] may be driven on
        self.idx = 0
        self.s_now = 0.0
        self.last_cmd_t = -1e9
        self.scan = None
        self.x = self.y = self.th = 0.0
        self.vx = self.vy = 0.0
        self.mode = "idle"
        self.wf = None
        self.seq = 0
        self.next_status = 0.0
        self.t0 = None
        self.blocked_report: list = []
        self.cmd_seq = -1

    def step(self, now, dt, own_pose, lidar_scan, inbox, battery=100.0):
        px, py = self.x, self.y
        self.x, self.y, self.th = own_pose
        if self.t0 is None:
            self.t0 = now
            px, py = self.x, self.y
        if dt > 0:
            self.vx = 0.5 * self.vx + 0.5 * (self.x - px) / dt
            self.vy = 0.5 * self.vy + 0.5 * (self.y - py) / dt
        if lidar_scan is not None:
            self.scan = lidar_scan
        for src, m in inbox:
            if src == CENTRAL and m.get("k") == "C" and m.get("n", 0) >= self.cmd_seq:
                self.cmd_seq = m.get("n", 0)
                self.last_cmd_t = now
                self.path = [tuple(c) for c in m["path"]]     # starts at the cell the robot is in
                self.idx = 0
                self.s_now = 0.0
                self.granted = int(m["g"])
                self.mode = m.get("mode", "moving")
                self.wf = m.get("wf")
        link = now - self.last_cmd_t
        creep = link > 2.0
        if link > 0.5:
            self.mode = "lost"
        cmd = self._drive(dt, creep=creep, stop=(0.5 < link <= 2.0))
        out = []
        if now >= self.next_status:
            self.next_status = now + 0.1
            self.seq += 1
            st = {"k": "S", "id": self.id, "q": self.seq, "p": [round(self.x, 2), round(self.y, 2), round(self.th, 2)],
                  "bat": round(battery, 1), "v": round(math.hypot(self.vx, self.vy), 2),
                  "blk": self.blocked_report}
            self.blocked_report = []
            out.append(("U", CENTRAL, st))
            # observer copy in the same shape the metrics recorder reads (mode / wait-for)
            out.append(("O", None, {"k": "B", "id": self.id, "m": self.mode, "wf": self.wf, "lv": 0,
                                    "p": st["p"]}))
        return cmd, out

    def _project(self):
        p = self.path
        if len(p) < 2:
            self.idx, self.s_now = 0, 0.0
            return
        best = (INF, self.idx, 0.0)
        for k in range(max(0, self.idx - 1), min(len(p) - 1, self.idx + 4)):
            a, b = p[k], p[k + 1]
            t, _, _, d = project_on_segment(self.x, self.y, a[0] + .5, a[1] + .5, b[0] + .5, b[1] + .5)
            if d < best[0] - 1e-9:
                best = (d, k, t)
        _, self.idx, t = best
        self.s_now = self.idx + t

    def _drive(self, dt, creep=False, stop=False):
        cfg = self.cfg
        p = self.path
        if stop or not p:
            return (0.0, 0.0)
        self._project()
        end = len(p) - 1 if creep else max(0, min(self.granted, len(p)) - 1)
        stop_s = float(end)
        off = INF
        if len(p) >= 2:
            k0 = min(self.idx, len(p) - 2)
            a0, b0 = p[k0], p[k0 + 1]
            _, qx, qy, off = project_on_segment(self.x, self.y, a0[0] + .5, a0[1] + .5, b0[0] + .5, b0[1] + .5)
        if len(p) >= 2 and off <= 0.3:
            s_c = min(self.s_now + 0.6, stop_s if stop_s > self.s_now else len(p) - 1.0)
            k = min(int(math.floor(s_c)), len(p) - 2)
            f = s_c - k
            a, b = p[k], p[min(k + 1, len(p) - 1)]
            cx, cy = a[0] + .5 + f * (b[0] - a[0]), a[1] + .5 + f * (b[1] - a[1])
        elif len(p) >= 2:
            cx, cy = qx, qy          # off the route (e.g. leaving a pocket): rejoin it, don't cut corners
        else:
            cx, cy = p[0][0] + .5, p[0][1] + .5
        remaining = stop_s - self.s_now
        g = p[end]
        remaining = max(remaining, math.hypot(self.x - g[0] - .5, self.y - g[1] - .5)) if end == len(p) - 1 else remaining
        dxc, dyc = cx - self.x, cy - self.y
        dn = math.hypot(dxc, dyc)
        if remaining < 0.03 or dn < 1e-6:
            return (0.0, 0.0)
        v = min(cfg.v_max, math.sqrt(2 * cfg.a_max * 0.8 * max(0.0, remaining - 0.02)))
        if end == len(p) - 1:
            v = min(v, max(0.15, remaining * 1.5))
        if creep:
            v = min(v, 0.3)
        head = math.atan2(dyc, dxc)
        err = wrap(head - self.th)
        w = max(-cfg.w_max, min(cfg.w_max, 3.0 * err))
        vf = v * math.cos(err) if abs(err) < 0.8 else 0.0
        if self.scan is not None and vf > 0:
            clr = forward_clearance(self.scan["ranges"], self.lidar, cfg.radius + cfg.safety_half_width_extra)
            vs = max_safe_speed(clr, cfg.a_max, cfg.safety_t_react, cfg.safety_margin)
            if vs < vf:
                vf = vs
                self._report_obstacle()
        return (max(0.0, vf), w)

    def _report_obstacle(self):
        """Lidar return inside the swept corridor that is not a known robot: tell the server."""
        if self.scan is None or not self.path:
            return
        r = self.scan["ranges"]
        sx, sy, sth = self.scan["pose"]
        k = int(np.argmin(np.where(np.abs(self.lidar.angles) < 0.4, r, np.inf)))
        if r[k] > 1.2:
            return
        a = sth + self.lidar.angles[k]
        c = self.map.cell_of(sx + (r[k] + 0.1) * math.cos(a), sy + (r[k] + 0.1) * math.sin(a))
        if c in self.map.free:
            self.blocked_report = [list(c)]


# =============================================================================== central
class _R:
    def __init__(self, rid):
        self.rid = rid
        self.pos = None
        self.cell = None
        self.battery = 100.0
        self.heard = -1e9
        self.v = 0.0
        self.path: list[Cell] = []
        self.idx = 0
        self.granted: set[Cell] = set()
        self.owned: set[Cell] = set()
        self.job = None            # dict(kind, tid, stage, goal, until)
        self.req = None            # resource currently requested (FIFO)
        self.req_t = None
        self.wait_for = None
        self.dead = False
        self.removed = False
        self.plan_blocked: set[Cell] = set()


class B1Central:
    def __init__(self, gmap: GridMap, cfg: Config, ids: list[int], tasks, seed: int = 0):
        self.map = gmap
        self.cfg = cfg
        self.graph = RouteGraph(gmap, cfg)
        self.r = {i: _R(i) for i in ids}
        self.tasks = {t.tid: t for t in tasks}
        self.assigned: dict[int, int] = {}
        self.done: set[int] = set()
        self.picked: set[int] = set()
        self.cell_owner: dict[Cell, int] = {}
        self.queue: dict[object, deque] = {}
        self.blocked: dict[Cell, float] = {}
        self.spot_owner: dict[Cell, int] = {}
        self.next_t = 0.0
        self.t = 0.0
        self.n = 0
        self.lost_since: dict[int, float] = {}
        self.spots = list(gmap.docks) + list(gmap.bays)
        self.obs_out: list = []
        for t in tasks:
            if t.assign is not None:
                self.assigned[t.tid] = int(t.assign)

    # ------------------------------------------------------------ hooks used by the runtime
    def robot_killed(self, rid):
        pass            # the server only learns about it by not hearing the robot any more

    def body_removed(self, rid):
        rr = self.r[rid]
        rr.removed = True          # operator tells the server the dead robot was pulled out
        for c in list(rr.owned):
            self._release(c, rid)

    # ------------------------------------------------------------ helpers
    def _dist(self, a: Cell, b: Cell) -> float:
        return static_dist_from(self.graph, a).get(b, 1e6)

    def _section_cells(self, c: Cell) -> list[Cell]:
        sid = self.map.section_of(c)
        if sid is not None:
            return self.map.sections[sid].cells
        return [c]

    def _res(self, c: Cell):
        sid = self.map.section_of(c)
        if sid is not None:
            return ("S", sid)
        return ("P", c) if c in self.map.pockets else c

    def _free_for(self, cells, rid) -> int | None:
        """Owner blocking these cells for rid, or None if all free."""
        for c in cells:
            o = self.cell_owner.get(c)
            if o is not None and o != rid:
                return o
        return None

    def _release(self, c: Cell, rid: int):
        if self.cell_owner.get(c) == rid:
            del self.cell_owner[c]
        self.r[rid].owned.discard(c)
        self.r[rid].granted.discard(c)

    def _take(self, cells, rid):
        for c in cells:
            self.cell_owner[c] = rid
            self.r[rid].owned.add(c)
            self.r[rid].granted.add(c)

    def _body_cells(self, rr: _R) -> set[Cell]:
        x, y = rr.pos
        rad = self.cfg.radius + 0.05
        out = set()
        for cx in range(int(math.floor(x - rad)), int(math.floor(x + rad)) + 1):
            for cy in range(int(math.floor(y - rad)), int(math.floor(y + rad)) + 1):
                if (cx, cy) in self.map.free:
                    d, _, _ = point_box_dist(x, y, cx, cy, cx + 1, cy + 1)
                    if d < rad:
                        out.add((cx, cy))
        return out

    # ------------------------------------------------------------ main loop (10 Hz)
    def step(self, now, dt, world, bus, alive):
        self.t = now
        inbox = bus.deliver(CENTRAL, now)
        for src, m in inbox:
            if m.get("k") == "S" and src in self.r:
                rr = self.r[src]
                rr.pos = (m["p"][0], m["p"][1])
                rr.cell = self.map.nearest_free(*rr.pos)
                rr.battery = m["bat"]
                rr.v = m.get("v", 0.0)
                rr.heard = now
                for c in m.get("blk", []):
                    c = tuple(c)
                    if c not in self.cell_owner or self.r[self.cell_owner[c]].dead:
                        self.blocked[c] = now + self.cfg.blockage_ttl_s
        if now < self.next_t:
            return
        self.next_t = now + 0.1
        self.n += 1
        out = []
        for c in [c for c, t in self.blocked.items() if t <= now]:
            del self.blocked[c]
        self.graph.set_blocked(set(self.blocked))
        self._liveness(now, out)
        self._occupancy()
        self._jobs(now, out)
        self._grants(now)
        # NOTE: NO _deadlocks() call here - B1 has no supervisor deadlock handling
        for rid, rr in self.r.items():
            if rr.pos is None or rr.dead:
                continue
            if now - rr.heard > 0.5:
                continue                      # link down: nothing reaches the robot anyway
            path = rr.path[rr.idx:rr.idx + 80] if rr.path else []      # full route: lets a robot that
            # loses the link creep on to its goal (blind, lidar-protected) instead of freezing
            g = 0
            for c in path:
                if c in rr.granted:
                    g += 1
                else:
                    break
            mode = self._mode(rr, g, len(path))
            out.append(("U", rid, {"k": "C", "n": self.n, "path": [list(c) for c in path], "g": g, "mode": mode,
                                   "wf": rr.wait_for}))
        out.extend(self.obs_out)
        self.obs_out = []
        if out:
            bus.send(CENTRAL, out, now)

    def _mode(self, rr, g, n):
        j = rr.job
        if j is None:
            return "idle"
        if j.get("until", 0) > self.t:
            return "working" if j["kind"] == "task" else "charging"
        if j["kind"] == "charge" and j["stage"] == "charging":
            return "charging"
        if n and g < n and rr.v < 0.05 and rr.req is not None:
            return "waiting"
        return "moving"

    # ------------------------------------------------------------ liveness of robots (server view)
    def _liveness(self, now, out):
        for rid, rr in self.r.items():
            if rr.pos is None:
                continue
            silent = now - rr.heard
            if silent > self.cfg.orphan_commit_s and not rr.dead:
                # treat as failed: hand its task back to the queue; keep its cells (body is there)
                rr.dead = True
                if rr.job and rr.job["kind"] == "task":
                    tid = rr.job["tid"]
                    rr.lost_job = dict(rr.job, was_picked=tid in self.picked)
                    self.assigned.pop(tid, None)      # back to the queue (re-picked if necessary)
                    self.picked.discard(tid)
                    out.append(("O", None, {"k": "ALERT", "type": "task_reauctioned", "sev": "warn",
                                            "msg": f"server reassigns task {tid} of silent R{rid}", "robots": [rid]}))
                rr.job = None
                rr.req = None
            elif silent <= 0.5 and rr.dead:
                rr.dead = False
                lj = getattr(rr, "lost_job", None)
                rr.lost_job = None
                if lj is not None and lj["tid"] not in self.done and lj["tid"] not in self.picked:
                    # it was only out of contact (dead zone / partition): give its job back unless
                    # another robot has already picked the item
                    other = self.assigned.get(lj["tid"])
                    if other is not None and other != rid:
                        ro = self.r[other]
                        ro.job, ro.path, ro.idx, ro.req = None, [], 0, None
                    self.assigned[lj["tid"]] = rid
                    if lj.pop("was_picked", False):
                        self.picked.add(lj["tid"])
                    rr.job = lj
                    rr.path, rr.idx, rr.req = [], 0, None

    def _occupancy(self):
        """Cells a robot's body covers are always owned by it; cells behind it are released."""
        for rid, rr in self.r.items():
            if rr.pos is None or rr.removed:
                continue
            body = self._body_cells(rr)
            silent = self.t - rr.heard > 0.5
            if rr.path:
                # progress index along its path
                best, bi = INF, rr.idx
                for k in range(max(0, rr.idx - 1), min(len(rr.path), rr.idx + 5)):
                    c = rr.path[k]
                    d = math.hypot(rr.pos[0] - c[0] - .5, rr.pos[1] - c[1] - .5)
                    if d < best:
                        best, bi = d, k
                rr.idx = bi
            if silent:
                # conservative: while out of contact it may creep along its remaining route
                creep = rr.path[rr.idx:rr.idx + 6] if (rr.path and not rr.dead) else []
                self._take([c for c in list(body) + creep if self.cell_owner.get(c) in (None, rid)], rid)
                continue
            for c in list(rr.owned):
                if c in body:
                    continue
                ahead = rr.path[rr.idx:] if rr.path else []
                if c not in ahead:
                    self._release(c, rid)
            self._take([c for c in body if self.cell_owner.get(c) in (None, rid)], rid)

    # ------------------------------------------------------------ dispatch (FIFO, nearest idle)
    def _need_charge(self, rr):
        c = self.cfg
        d = min(self._dist(rr.cell, k) for k in self.map.docks)
        return rr.battery < d * c.battery_per_m + c.battery_reserve + 3.0

    def _feasible(self, rr, t):
        c = self.cfg
        m = self._dist(rr.cell, t.pick) + self._dist(t.pick, t.drop) + min(self._dist(t.drop, k) for k in self.map.docks)
        return rr.battery >= m * c.battery_per_m + c.battery_reserve

    def _admissible(self, t):
        lim = self.cfg.admit_max_queue
        sid = self.map.section_of(t.pick)
        n_pick = n_drop = 0
        for tid, rid in self.assigned.items():
            if tid in self.done or tid == t.tid:
                continue
            tt = self.tasks[tid]
            if tid not in self.picked and sid is not None and self.map.section_of(tt.pick) == sid:
                n_pick += 1
            if tt.drop == t.drop and tid in self.picked:
                n_drop += 1                       # only robots already carrying to that drop crowd it
        return n_pick < lim and n_drop < lim

    def _set_goal(self, rr, goal):
        rr.job["goal"] = goal
        rr.path = []
        rr.idx = 0
        rr.req = None

    def _jobs(self, now, out):
        # progress running jobs
        for rid, rr in self.r.items():
            if rr.pos is None or rr.dead:
                continue
            j = rr.job
            if j is None:
                continue
            at = math.hypot(rr.pos[0] - j["goal"][0] - .5, rr.pos[1] - j["goal"][1] - .5) < 0.2 and rr.v < 0.2
            if j["kind"] == "task":
                t = self.tasks[j["tid"]]
                if j["stage"] == "to_pick" and at:
                    j["stage"], j["until"] = "pick", now + self.cfg.dwell_pick_s
                elif j["stage"] == "pick" and now >= j["until"]:
                    self.picked.add(t.tid)
                    out.append(("O", None, {"k": "PICK", "j": t.tid, "by": rid}))
                    j["stage"] = "to_drop"
                    self._set_goal(rr, t.drop)
                elif j["stage"] == "to_drop" and at:
                    j["stage"], j["until"] = "drop", now + self.cfg.dwell_drop_s
                elif j["stage"] == "drop" and now >= j["until"]:
                    self.done.add(t.tid)
                    out.append(("O", None, {"k": "DONE", "j": t.tid, "by": rid}))
                    rr.job = None
            elif j["kind"] == "charge":
                if j["stage"] == "to_dock" and at:
                    j["stage"] = "charging"
                if j["stage"] == "charging" and rr.battery >= self.cfg.battery_full:
                    self.spot_owner.pop(j["goal"], None)
                    rr.job = None
            elif j["kind"] == "park" and not at:
                pass
        # dispatch released, unassigned tasks FIFO to the nearest idle feasible robot
        idle = [rr for rr in self.r.values()
                if rr.pos is not None and not rr.dead and now - rr.heard <= 0.5
                and (rr.job is None or rr.job["kind"] == "park")]
        for rr in list(idle):
            if self._need_charge(rr):
                dock = self._free_spot(rr, self.map.docks)
                if dock is not None:
                    self._free_spot_of(rr.rid)
                    self.spot_owner[dock] = rr.rid
                    rr.job = {"kind": "charge", "tid": None, "stage": "to_dock", "goal": dock}
                    rr.path, rr.idx, rr.req = [], 0, None
                    idle.remove(rr)
        pending = sorted((t for t in self.tasks.values()
                          if t.release <= now and t.tid not in self.done and t.tid not in self.assigned),
                         key=lambda t: (t.release, t.tid))
        for t in pending:
            if not idle:
                break
            if not self._admissible(t):
                continue
            cands = [rr for rr in idle if self._feasible(rr, t)]
            if not cands:
                continue
            best = min(cands, key=lambda rr: (self._dist(rr.cell, t.pick), rr.rid))
            idle.remove(best)
            self.assigned[t.tid] = best.rid
            self._free_spot_of(best.rid)
            best.job = {"kind": "task", "tid": t.tid, "stage": "to_pick", "goal": t.pick}
            best.path, best.idx, best.req = [], 0, None
            out.append(("O", None, {"k": "CLAIM", "j": t.tid, "by": best.rid}))
        # scripted assignments
        for tid, rid in list(self.assigned.items()):
            rr = self.r[rid]
            t = self.tasks[tid]
            if (tid not in self.done and t.release <= now and rr.pos is not None and not rr.dead
                    and (rr.job is None or rr.job["kind"] == "park") and tid not in self.picked):
                self._free_spot_of(rid)
                rr.job = {"kind": "task", "tid": tid, "stage": "to_pick", "goal": t.pick}
                rr.path, rr.idx, rr.req = [], 0, None
        # park idle robots
        for rr in idle:
            if rr.job is None:
                spot = self._free_spot(rr, self.map.bays) or self._free_spot(rr, self.spots)
                if spot is not None:
                    self.spot_owner[spot] = rr.rid
                    rr.job = {"kind": "park", "tid": None, "stage": "to_spot", "goal": spot}
                    rr.path, rr.idx, rr.req = [], 0, None

    def _free_spot(self, rr, cands):
        free = [s for s in cands if self.spot_owner.get(s) in (None, rr.rid)]
        if not free:
            return None
        return min(free, key=lambda s: (0 if s == rr.cell else 1, self._dist(rr.cell, s), s))

    def _free_spot_of(self, rid):
        for s in [s for s, o in self.spot_owner.items() if o == rid]:
            del self.spot_owner[s]

    # ------------------------------------------------------------ routes + zone grants (FIFO)
    def _plan(self, rr, avoid: set[Cell] | None = None):
        goal = rr.job["goal"]
        saved = self.graph.blocked
        blk = set(saved) | (avoid or set())
        blk.discard(goal)
        blk.discard(rr.cell)
        self.graph.blocked = blk
        ds = DStarLite(self.graph, goal)
        ds.compute(rr.cell)
        p = ds.path(rr.cell)
        self.graph.blocked = saved
        return p

    def _grants(self, now):
        # drop queue entries of robots that moved on (new job / re-routed / failed)
        for res, q in self.queue.items():
            for rid in [r for r in q if self.r[r].dead or self.r[r].req != res]:
                q.remove(rid)
        # FIFO: serve robots in order of when they started waiting for their current resource
        order = sorted((rr for rr in self.r.values() if rr.job is not None and not rr.dead and rr.pos is not None
                        and now - rr.heard <= 0.5),
                       key=lambda rr: (rr.req_t if rr.req_t is not None else now, rr.rid))
        for rr in order:
            if not rr.path:
                p = self._plan(rr)
                if p is None:
                    continue
                rr.path, rr.idx = p, 0
            rr.wait_for = None
            k = rr.idx
            while k < len(rr.path) and k - rr.idx <= LOOKAHEAD:
                c = rr.path[k]
                if c in rr.granted:
                    k += 1
                    continue
                zone = self._section_cells(c)
                cells = list(zone)
                j = k + 1
                res = self._res(c)
                if k + 1 < len(rr.path) and rr.path[k + 1] in self.map.pockets and c not in self.map.pockets:
                    # the mouth cell is granted only together with the pocket behind it, so no robot
                    # ever waits on a mouth for its pocket (it would block the occupant's exit)
                    pk = rr.path[k + 1]
                    cells = [c, pk]
                    res = ("P", pk)
                    j = k + 2
                elif self.map.section_of(c) is not None:
                    # an aisle is one zone, granted with the cell after it (never stop in the box)
                    while j < len(rr.path) and rr.path[j] in zone:
                        j += 1
                    if j < len(rr.path):
                        cells += self._section_cells(rr.path[j])
                        j += 1
                q = self.queue.setdefault(res, deque())
                if rr.rid not in q:
                    q.append(rr.rid)
                blocker = self._free_for(cells, rr.rid)
                if blocker is None and q[0] == rr.rid:
                    self._take(cells, rr.rid)
                    q.popleft()
                    rr.req, rr.req_t = None, None
                    k = j
                    continue
                if rr.req != res:
                    rr.req, rr.req_t = res, now
                    if isinstance(res, tuple) and res[0] == "S":
                        self.obs_out.append(("O", None, {"k": "REQ", "s": res[1], "by": rr.rid}))
                rr.wait_for = blocker if blocker is not None else q[0]
                break
