"""L4 Task allocation: decentralised and battery-feasible.

* `CBBA`  Consensus-Based Bundle Algorithm (Choi, Brunet & How, IEEE T-RO 2009). Each robot
          greedily builds a bundle by marginal score, then resolves conflicts with peers using
          the CBBA consensus table on (winning bids y, winners z, timestamps s).
          Score = sum_j R * lambda^(t_j - now): time-discounted reward, which keeps CBBA's
          diminishing-marginal-gains property. Congestion enters as a travel-time cost frozen
          at the start of each auction round. Battery is a hard feasibility filter.
* `SequentialAuction`  the simpler first version: tasks auctioned one at a time in id order,
          highest bid wins, deterministic tie-break by robot id.

Execution: a robot commits to the first task of its path once its winning bid has been stable
for `commit_stable_s`, and announces CLAIM (reliable). Committed tasks leave the auction.
Two conflicting CLAIMs are resolved by (bid, lower id); the loser aborts if it has not picked
up yet. DONE retires a task. A lost peer's tasks are re-auctioned (bundle tasks after
`orphan_bundle_s`, a committed task after `orphan_commit_s`).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable

from amr.core.config import Config, DEFAULT
from amr.core.grid_map import Cell

EPS = 1e-7   # bids are compared with a tolerance, ties broken by lower robot id


@dataclass
class Task:
    tid: int
    pick: Cell
    drop: Cell
    release: float = 0.0


def better(y1: float, z1: int | None, y2: float, z2: int | None) -> bool:
    """Is bid (y1 by z1) strictly better than (y2 by z2)? Ties -> lower robot id."""
    if z1 is None:
        return False
    if z2 is None:
        return y1 > EPS
    if y1 > y2 + EPS:
        return True
    if abs(y1 - y2) <= EPS:
        return z1 < z2
    return False


class TaskBook:
    """Order book + execution state shared by both allocators (each robot has its own copy)."""

    def __init__(self, me: int, cfg: Config, travel: Callable[[Cell, Cell], float],
                 dist_m: Callable[[Cell, Cell], float], nearest_dock_m: Callable[[Cell], float]):
        self.me = me
        self.cfg = cfg
        self.travel = travel
        self.dist_m = dist_m
        self.nearest_dock_m = nearest_dock_m
        self.tasks: dict[int, Task] = {}
        self.done: set[int] = set()
        self.committed: dict[int, tuple[int, float]] = {}   # tid -> (robot, bid)
        self.picked: set[int] = set()                        # tasks I have picked up
        self.congestion: Callable[[Cell, Cell], float] = lambda a, b: 0.0
        self.admissible: Callable[[int], bool] = lambda j: True   # capacity filter (like battery)

    def add(self, t: Task) -> None:
        self.tasks.setdefault(t.tid, t)

    def available(self, tid: int, now: float) -> bool:
        t = self.tasks.get(tid)
        return (t is not None and t.release <= now and tid not in self.done
                and tid not in self.committed)

    def leg_time(self, a: Cell, b: Cell) -> float:
        return self.travel(a, b) + self.congestion(a, b)

    def path_eval(self, tids: list[int], start: Cell, t_start: float, now: float,
                  battery: float) -> tuple[float, bool]:
        """(time-discounted score, battery-feasible) of executing tids in order."""
        c = self.cfg
        t, loc, score, metres = t_start, start, 0.0, 0.0
        for tid in tids:
            tk = self.tasks[tid]
            t += self.leg_time(loc, tk.pick) + c.dwell_pick_s + self.leg_time(tk.pick, tk.drop) + c.dwell_drop_s
            metres += self.dist_m(loc, tk.pick) + self.dist_m(tk.pick, tk.drop)
            score += c.task_reward * (c.discount ** max(0.0, t - now))
            loc = tk.drop
        metres += self.nearest_dock_m(loc)
        need = metres * c.battery_per_m + (t - now) * c.battery_idle_per_s + c.battery_reserve
        return score, battery >= need


class CBBA:
    def __init__(self, book: TaskBook):
        self.book = book
        self.me = book.me
        self.y: dict[int, float] = {}
        self.z: dict[int, int | None] = {}
        self.s: dict[int, float] = {}
        self.bundle: list[int] = []
        self.path: list[int] = []
        self.last_change: dict[int, float] = {}
        self.dirty = True           # something changed -> broadcast
        self.rounds = 0

    # ------------------------------------------------------------------ helpers
    def _set(self, j: int, y: float, z: int | None, now: float) -> None:
        if self.y.get(j, 0.0) != y or self.z.get(j) != z:
            self.last_change[j] = now
            self.dirty = True
        self.y[j] = y
        self.z[j] = z

    def _reset(self, j: int, now: float) -> None:
        self._set(j, 0.0, None, now)

    def forget(self, j: int) -> None:
        """Task left the auction (committed/done)."""
        for d in (self.y, self.z, self.last_change):
            d.pop(j, None)
        if j in self.bundle:
            self.bundle.remove(j)
        if j in self.path:
            self.path.remove(j)

    # ------------------------------------------------------------------ phase 1: bundle
    def build(self, now: float, start: Cell, t_start: float, battery: float) -> None:
        bk, cfg = self.book, self.book.cfg
        # drop anything no longer available
        for j in [j for j in self.bundle if not bk.available(j, now)]:
            self.forget(j)
        # rescore current path (times move on); keep bids current
        base, _ = bk.path_eval(self.path, start, t_start, now, battery)
        while len(self.bundle) < cfg.bundle_max:
            best_gain, best_j, best_pos = 0.0, None, 0
            for j, t in bk.tasks.items():
                if j in self.bundle or not bk.available(j, now) or not bk.admissible(j):
                    continue
                for pos in range(len(self.path) + 1):
                    cand = self.path[:pos] + [j] + self.path[pos:]
                    sc, ok = bk.path_eval(cand, start, t_start, now, battery)
                    if not ok:
                        continue
                    gain = sc - base
                    if gain > best_gain + EPS and better(gain, self.me, self.y.get(j, 0.0), self.z.get(j)):
                        best_gain, best_j, best_pos = gain, j, pos
            if best_j is None:
                break
            self.bundle.append(best_j)
            self.path.insert(best_pos, best_j)
            self._set(best_j, best_gain, self.me, now)
            base += best_gain
        self.rounds += 1

    # ------------------------------------------------------------------ phase 2: consensus
    def receive(self, k: int, yk: dict[int, float], zk: dict[int, int | None], sk: dict[int, float],
                now: float) -> None:
        i = self.me
        bk = self.book
        s_i = self.s
        tasks = set(yk) | {j for j, z in self.z.items() if z is not None}
        for j in tasks:
            if not bk.available(j, now):
                continue
            ykj, zkj = yk.get(j, 0.0), zk.get(j)
            yij, zij = self.y.get(j, 0.0), self.z.get(j)
            act = self._decide(i, k, ykj, zkj, yij, zij, sk, s_i)
            if act == "update":
                self._set(j, ykj, zkj, now)
            elif act == "reset":
                self._reset(j, now)
        s_i[k] = now
        for m, t in sk.items():
            if m != i and t > s_i.get(m, -1.0):
                s_i[m] = t
        self._repair_bundle(now)

    @staticmethod
    def _decide(i, k, ykj, zkj, yij, zij, sk, si) -> str:
        def skm(m):
            return sk.get(m, -1.0)

        def sim(m):
            return si.get(m, -1.0)

        if zkj == k:
            if zij == i:
                return "update" if better(ykj, zkj, yij, zij) else "leave"
            if zij == k:
                return "update"
            if zij is None:
                return "update"
            m = zij
            return "update" if (skm(m) > sim(m) or better(ykj, zkj, yij, zij)) else "leave"
        if zkj == i:
            if zij == i or zij is None:
                return "leave"
            if zij == k:
                return "reset"
            return "reset" if skm(zij) > sim(zij) else "leave"
        if zkj is not None:            # sender thinks a third robot m wins
            m = zkj
            if zij == i:
                return "update" if (skm(m) > sim(m) and better(ykj, zkj, yij, zij)) else "leave"
            if zij == k:
                return "update" if skm(m) > sim(m) else "reset"
            if zij == m:
                return "update" if skm(m) > sim(m) else "leave"
            if zij is None:
                return "update" if skm(m) > sim(m) else "leave"
            n = zij
            if skm(m) > sim(m) and skm(n) > sim(n):
                return "update"
            if skm(m) > sim(m) and better(ykj, zkj, yij, zij):
                return "update"
            if skm(n) > sim(n) and sim(m) > skm(m):
                return "reset"
            return "leave"
        # sender has no winner
        if zij == i or zij is None:
            return "leave"
        if zij == k:
            return "update"
        return "update" if skm(zij) > sim(zij) else "leave"

    def _repair_bundle(self, now: float) -> None:
        for n, j in enumerate(self.bundle):
            if self.z.get(j) != self.me:
                for jj in self.bundle[n + 1:]:
                    if self.z.get(jj) == self.me:
                        self._reset(jj, now)
                for jj in self.bundle[n:]:
                    if jj in self.path:
                        self.path.remove(jj)
                self.bundle = self.bundle[:n]
                self.dirty = True
                return

    def orphan(self, peer: int, now: float) -> list[int]:
        out = []
        for j, z in list(self.z.items()):
            if z == peer:
                self._reset(j, now)
                out.append(j)
        return out

    def stable(self, j: int, now: float, hold: float) -> bool:
        return self.z.get(j) == self.me and now - self.last_change.get(j, now) >= hold

    def message(self, now: float) -> dict:
        self.dirty = False
        self.s[self.me] = now
        b = [[j, self.y[j], z] for j, z in self.z.items() if z is not None]
        return {"k": "BID", "b": b, "s": {str(r): round(t, 3) for r, t in self.s.items()}}

    def on_bid_message(self, k: int, m: dict, now: float) -> None:
        yk = {int(j): float(y) for j, y, _ in m["b"]}
        zk = {int(j): (None if z is None else int(z)) for j, _, z in m["b"]}
        sk = {int(r): float(t) for r, t in m.get("s", {}).items()}
        self.receive(k, yk, zk, sk, now)


class SequentialAuction:
    """Simpler allocator: auction the lowest-id available task; each idle robot bids its
    time-discounted score; after `window` the highest bid wins (tie: lowest id). Every robot
    computes the winner from the same (reliably delivered) bids, so no auctioneer is needed."""

    def __init__(self, book: TaskBook, window: float = 0.3):
        self.book = book
        self.me = book.me
        self.window = window
        self.bids: dict[int, dict[int, float]] = {}      # tid -> {robot: bid}
        self.opened: dict[int, float] = {}
        self.path: list[int] = []
        self.dirty = False
        self._pending_msg: list[dict] = []

    def current(self, now: float) -> int | None:
        avail = sorted(j for j in self.book.tasks if self.book.available(j, now))
        return avail[0] if avail else None

    def build(self, now: float, start: Cell, t_start: float, battery: float, idle: bool = True) -> None:
        j = self.current(now)
        if j is None:
            self.path = []
            return
        if j not in self.opened:
            self.opened[j] = now
            bid = 0.0
            if idle:
                sc, ok = self.book.path_eval([j], start, t_start, now, battery)
                bid = sc if ok else 0.0
            self.bids.setdefault(j, {})[self.me] = bid
            self._pending_msg.append({"k": "SBID", "j": j, "y": bid})
            self.dirty = True
        if now - self.opened[j] >= self.window:
            w = self.winner(j)
            self.path = [j] if w == self.me else []

    def winner(self, j: int) -> int | None:
        bids = self.bids.get(j, {})
        best = None
        for r, y in bids.items():
            if y > EPS and (best is None or better(y, r, bids[best], best)):
                best = r
        return best

    def on_bid_message(self, k: int, m: dict, now: float) -> None:
        self.bids.setdefault(int(m["j"]), {})[k] = float(m["y"])

    def stable(self, j: int, now: float, hold: float) -> bool:
        return j in self.path

    def message(self, now: float) -> list[dict]:
        self.dirty = False
        o, self._pending_msg = self._pending_msg, []
        return o

    def forget(self, j: int) -> None:
        if j in self.path:
            self.path.remove(j)
        self.bids.pop(j, None)
        self.opened.pop(j, None)

    def orphan(self, peer: int, now: float) -> list[int]:
        return []
