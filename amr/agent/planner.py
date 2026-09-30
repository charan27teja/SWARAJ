"""L3 Route planning.

* `RouteGraph`  static cell graph with travel-time costs (right-hand-traffic bias in wide lanes,
                small penalty on narrow cells), plus a per-robot overlay of blocked cells
                (BlockageEvents, TTL-expired locally) and the degraded-mode entry rule.
* `DStarLite`   Koenig & Likhachev (2002) incremental search, run backwards from the goal. It
                maintains the exact cost-to-go field and repairs it when the overlay changes
                (static-map repair) instead of re-planning from scratch.
* `sipp`        Safe Interval Path Planning (Phillips & Likhachev 2011) over the cell graph
                using a local reservation table built from what peers broadcast. The D* Lite
                cost-to-go is its (exact, admissible) heuristic.
* `st_astar`    classic space-time A* against a discrete reservation table (the simpler first
                version, kept for tests/ablation).
"""
from __future__ import annotations

import heapq
import math
from functools import lru_cache
from typing import Callable

from amr.core.config import Config, DEFAULT
from amr.core.grid_map import Cell, GridMap

INF = float("inf")


def _right_of(dx: int, dy: int) -> tuple[int, int]:
    # screen coordinates (y down): facing +x, the right-hand side is +y
    return (-dy, dx)


class RouteGraph:
    def __init__(self, gmap: GridMap, cfg: Config = DEFAULT):
        self.map = gmap
        self.cfg = cfg
        self.blocked: set[Cell] = set()
        self.degraded = False
        self.version = 0
        self._base: dict[Cell, list[tuple[Cell, float]]] = {}
        wide = gmap.wide
        for c in gmap.free:
            out = []
            for n in gmap.neighbors(c):
                cost = 1.0 / cfg.v_max
                dx, dy = n[0] - c[0], n[1] - c[1]
                if n in wide:
                    rx, ry = _right_of(dx, dy)
                    if (n[0] + rx, n[1] + ry) in wide:
                        cost += cfg.right_hand_penalty
                elif gmap.section_of(n) is not None:
                    cost += cfg.narrow_penalty
                    sec = gmap.sections[gmap.section_of(n)]
                    if sec.one_way:
                        if gmap.section_of(c) != sec.sid and c != sec.canonical_outside:
                            cost += cfg.wrong_way_penalty          # entering through the exit end
                        elif gmap.section_of(c) == sec.sid and dy < 0:
                            cost += cfg.wrong_way_penalty / 5.0    # reversing inside a one-way aisle
                out.append((n, cost))
            self._base[c] = out
        self._pred: dict[Cell, list[tuple[Cell, float]]] = {c: [] for c in gmap.free}
        for c, outs in self._base.items():
            for n, cost in outs:
                self._pred[n].append((c, cost))

    # overlay ---------------------------------------------------------------
    def set_blocked(self, cells: set[Cell]) -> set[Cell]:
        cells = set(cells)
        changed = cells ^ self.blocked
        if changed:
            self.blocked = cells
            self.version += 1
        return changed

    def set_degraded(self, on: bool) -> bool:
        if on != self.degraded:
            self.degraded = on
            self.version += 1
            return True
        return False

    def edge_ok(self, u: Cell, v: Cell) -> bool:
        if v in self.blocked:
            return False
        if self.degraded:
            m = self.map
            sv = m.section_of(v)
            if sv is not None and m.section_of(u) != sv:
                if m.sections[sv].canonical_outside != u:
                    return False
        return True

    def succ(self, u: Cell):
        for v, c in self._base.get(u, ()):
            if self.edge_ok(u, v):
                yield v, c

    def pred(self, v: Cell):
        for u, c in self._pred.get(v, ()):
            if self.edge_ok(u, v):
                yield u, c

    def static_cost(self, u: Cell, v: Cell) -> float:
        for n, c in self._base[u]:
            if n == v:
                return c
        return INF


# ---------------------------------------------------------------------------- D* Lite
class DStarLite:
    """Backward D* Lite from `goal`; g(s) = cost-to-go from s. `compute(start)` repairs the
    field incrementally after `notify(changed_cells)`."""

    def __init__(self, graph: RouteGraph, goal: Cell):
        self.G = graph
        self.goal = goal
        self.g: dict[Cell, float] = {}
        self.rhs: dict[Cell, float] = {goal: 0.0}
        self.km = 0.0
        self.start: Cell | None = None
        self.U: list = []
        self._in: dict[Cell, tuple] = {}
        self._push(goal)
        self.version = graph.version
        self.expansions = 0

    _templates: dict = {}

    @classmethod
    def from_template(cls, graph: "RouteGraph", goal: Cell) -> "DStarLite":
        """A converged D* Lite for `goal` on the static graph (+ degraded rule), then repaired
        incrementally for this robot's own blockage overlay. The converged static field is
        pure map data, so it is memoised per (map, goal, degraded)."""
        key = (id(graph.map), goal, graph.degraded)
        t = cls._templates.get(key)
        if t is None:
            saved = graph.blocked
            graph.blocked = set()
            t = cls(graph, goal)
            t.compute(goal)
            graph.blocked = saved
            cls._templates[key] = t
        ds = cls.__new__(cls)
        ds.G = graph
        ds.goal = goal
        ds.g = dict(t.g)
        ds.rhs = dict(t.rhs)
        ds.km = 0.0
        ds.start = t.start
        ds.U = []
        ds._in = {}
        ds.version = -1
        ds.expansions = 0
        if graph.blocked:
            ds.notify(set(graph.blocked))
        ds.version = graph.version
        return ds

    def _h(self, a: Cell, b: Cell | None) -> float:
        if b is None:
            return 0.0
        return (abs(a[0] - b[0]) + abs(a[1] - b[1])) / self.G.cfg.v_max

    def _key(self, s: Cell) -> tuple[float, float]:
        m = min(self.g.get(s, INF), self.rhs.get(s, INF))
        return (m + self._h(s, self.start) + self.km, m)

    def _push(self, s: Cell) -> None:
        k = self._key(s)
        self._in[s] = k
        heapq.heappush(self.U, (k, s))

    def _update_vertex(self, u: Cell) -> None:
        if u != self.goal:
            best = INF
            for v, c in self.G.succ(u):
                val = c + self.g.get(v, INF)
                if val < best:
                    best = val
            self.rhs[u] = best
        self._in.pop(u, None)
        if self.g.get(u, INF) != self.rhs.get(u, INF):
            self._push(u)

    def notify(self, changed: set[Cell]) -> None:
        affected = set()
        for v in changed:
            affected.add(v)
            for u, _ in self.G._pred.get(v, ()):
                affected.add(u)
        for u in affected:
            self._update_vertex(u)
        self.version = self.G.version

    def compute(self, start: Cell, full: bool = True) -> None:
        if self.start is not None and start != self.start:
            self.km += self._h(self.start, start)
        self.start = start
        while self.U:
            k_old, u = self.U[0]
            if self._in.get(u) != k_old:
                heapq.heappop(self.U)           # stale entry
                continue
            if not full:
                ks = self._key(start)
                if not (k_old < ks or self.rhs.get(start, INF) != self.g.get(start, INF)):
                    break
            heapq.heappop(self.U)
            del self._in[u]
            self.expansions += 1
            k_new = self._key(u)
            gu, ru = self.g.get(u, INF), self.rhs.get(u, INF)
            if k_old < k_new:
                self._push(u)
            elif gu > ru:
                self.g[u] = ru
                for p, _ in self.G.pred(u):
                    self._update_vertex(p)
            else:
                self.g[u] = INF
                self._update_vertex(u)
                for p, _ in self.G.pred(u):
                    self._update_vertex(p)

    def cost_to_go(self, s: Cell) -> float:
        return self.g.get(s, INF)

    def path(self, start: Cell, max_len: int = 2000) -> list[Cell] | None:
        if self.cost_to_go(start) == INF:
            return None
        p, s = [start], start
        while s != self.goal and len(p) < max_len:
            best, bn = INF, None
            for v, c in self.G.succ(s):
                val = c + self.g.get(v, INF)
                if val < best:
                    best, bn = val, v
            if bn is None or best == INF:
                return None
            s = bn
            p.append(s)
        return p if s == self.goal else None


# ---------------------------------------------------------------------------- SIPP
Reservations = dict[Cell, list[tuple[float, float]]]


def safe_intervals(unsafe: list[tuple[float, float]], t0: float) -> list[tuple[float, float]]:
    """Complement of merged unsafe intervals within [t0, inf)."""
    if not unsafe:
        return [(t0, INF)]
    iv = sorted(unsafe)
    merged = []
    for a, b in iv:
        if b <= t0:
            continue
        if merged and a <= merged[-1][1]:
            merged[-1] = (merged[-1][0], max(merged[-1][1], b))
        else:
            merged.append((a, b))
    out, cur = [], t0
    for a, b in merged:
        if a > cur:
            out.append((cur, a))
        cur = max(cur, b)
    out.append((cur, INF))
    return out


def sipp(graph: RouteGraph, start: Cell, goal: Cell, t0: float, res: Reservations,
         h: Callable[[Cell], float], can_wait: Callable[[Cell], bool],
         max_expansions: int = 20000) -> tuple[list[tuple[Cell, float]], float] | None:
    """Returns ([(cell, arrival_time), ...], goal_arrival) or None if unreachable."""
    si_cache: dict[Cell, list[tuple[float, float]]] = {}

    def sis(c: Cell):
        s = si_cache.get(c)
        if s is None:
            s = safe_intervals(res.get(c, []), t0)
            si_cache[c] = s
        return s

    h0 = h(start)
    if h0 == INF:
        return None
    start_iv = (t0, INF)
    for iv in sis(start):
        if iv[0] <= t0 < iv[1]:
            start_iv = iv
            break
    else:
        # already inside a reserved cell (e.g. our own section, stale view): treat as safe until next
        nxt = [iv for iv in sis(start) if iv[0] > t0]
        start_iv = (t0, nxt[0][0] if nxt else INF)
    start_key = (start, -1)
    best: dict = {start_key: t0}
    parent: dict = {start_key: None}
    ivs_of: dict = {start_key: start_iv}
    heap = [(t0 + h0, t0, start, -1)]
    exp = 0
    while heap:
        f, t, c, k = heapq.heappop(heap)
        key = (c, k)
        if best.get(key, INF) < t:
            continue
        if c == goal:
            path = []
            while key is not None:
                path.append((key[0], best[key]))
                key = parent[key]
            path.reverse()
            return path, t
        exp += 1
        if exp > max_expansions:
            return None
        a_k, b_k = ivs_of[key]
        wait_ok = can_wait(c) or k == -1
        for n, cost in graph.succ(c):
            lo = t + cost
            hi = (b_k + cost) if wait_ok else lo
            for m, (a_m, b_m) in enumerate(sis(n)):
                if a_m > hi:
                    break
                if b_m <= lo:
                    continue
                arr = max(lo, a_m)
                if arr >= b_m:
                    continue
                nk = (n, m)
                if arr < best.get(nk, INF):
                    hn = h(n)
                    if hn == INF:
                        continue
                    best[nk] = arr
                    parent[nk] = key
                    ivs_of[nk] = (a_m, b_m)
                    heapq.heappush(heap, (arr + hn, arr, n, m))
    return None


def evaluate_path(graph: RouteGraph, path: list[Cell], t0: float, res: Reservations,
                  can_wait: Callable[[Cell], bool]) -> float:
    """Arrival time at the end of a fixed path, waiting (where allowed) for reserved cells."""
    t = t0
    for i in range(1, len(path)):
        u, v = path[i - 1], path[i]
        c = graph.static_cost(u, v)
        if c == INF or not graph.edge_ok(u, v):
            return INF
        arr = t + c
        for a, b in sorted(res.get(v, [])):
            if a <= arr < b:
                if can_wait(u) or i == 1:
                    arr = b
                else:
                    arr = b + 5.0   # would stall inside a narrow cell: heavy penalty
        t = arr
    return t


# ---------------------------------------------------------------------------- space-time A*
def st_astar(graph: RouteGraph, start: Cell, goal: Cell, reserved: set[tuple[Cell, int]],
             h: Callable[[Cell], float], horizon: int = 300) -> list[Cell] | None:
    """Space-time A* on unit time steps (1 cell/step) against a vertex reservation table
    {(cell, t)}; also forbids edge swaps with reserved moves. Returns cells per time step."""
    start_state = (start, 0)
    heap = [(h(start), 0, start)]
    parent = {start_state: None}
    while heap:
        f, t, c = heapq.heappop(heap)
        if c == goal:
            out, s = [], (c, t)
            while s is not None:
                out.append(s[0])
                s = parent[s]
            return out[::-1]
        if t >= horizon:
            continue
        for n in [c] + [v for v, _ in graph.succ(c)]:
            s2 = (n, t + 1)
            if s2 in parent or (n, t + 1) in reserved:
                continue
            if n != c and (c, t + 1) in reserved and (n, t) in reserved:
                continue   # swap conflict
            parent[s2] = (c, t)
            heapq.heappush(heap, (t + 1 + h(n), t + 1, n))
    return None


# ---------------------------------------------------------------------------- static tables
_DIST_CACHE: dict = {}


def static_dist_from(graph: RouteGraph, src: Cell) -> dict[Cell, float]:
    """Forward Dijkstra from src on the *static* costs (no overlay). Cached per map: this is
    fixed map data every robot can precompute identically."""
    key = (id(graph.map), src)
    d = _DIST_CACHE.get(key)
    if d is not None:
        return d
    d = {src: 0.0}
    heap = [(0.0, src)]
    while heap:
        du, u = heapq.heappop(heap)
        if du > d.get(u, INF):
            continue
        for v, c in graph._base.get(u, ()):
            nd = du + c
            if nd < d.get(v, INF):
                d[v] = nd
                heapq.heappush(heap, (nd, v))
    _DIST_CACHE[key] = d
    return d


def static_path(graph: RouteGraph, src: Cell, dst: Cell) -> list[Cell] | None:
    """Shortest static path (ignores overlay); used for cost estimates."""
    ds = DStarLite(graph, dst)
    ds.compute(src)
    return ds.path(src)
