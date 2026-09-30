"""P5: route planning - D* Lite (vs Dijkstra, incremental repair), SIPP, space-time A*."""
import heapq

import pytest

from amr.core.config import DEFAULT
from amr.core.grid_map import GridMap
from amr.core.scenario import ROOT
from amr.agent.planner import RouteGraph, DStarLite, sipp, st_astar, safe_intervals, evaluate_path, INF


@pytest.fixture(scope="module")
def gm():
    return GridMap.load(ROOT / "maps/warehouse_a.yaml")


def dijkstra_to(graph, goal):
    d = {goal: 0.0}
    h = [(0.0, goal)]
    while h:
        dv, v = heapq.heappop(h)
        if dv > d.get(v, INF):
            continue
        for u, c in graph.pred(v):
            if dv + c < d.get(u, INF):
                d[u] = dv + c
                heapq.heappush(h, (dv + c, u))
    return d


def test_dstar_lite_matches_dijkstra(gm):
    g = RouteGraph(gm, DEFAULT)
    goal = (22, 23)
    ds = DStarLite(g, goal)
    ds.compute((2, 3))
    ref = dijkstra_to(g, goal)
    for c in list(gm.free)[::7]:
        assert ds.cost_to_go(c) == pytest.approx(ref.get(c, INF))


def test_dstar_lite_repairs_after_blockage(gm):
    g = RouteGraph(gm, DEFAULT)
    goal = (13, 18)                               # pick inside aisle A13
    ds = DStarLite(g, goal)
    ds.compute((13, 13))
    before = ds.cost_to_go((13, 13))
    changed = g.set_blocked({(13, 17)})           # pallet between the entry and... (not on this path)
    ds.notify(changed)
    ds.compute((13, 13))
    ref = dijkstra_to(g, goal)
    assert ds.cost_to_go((13, 13)) == pytest.approx(ref[(13, 13)])
    assert ds.cost_to_go((13, 13)) > before       # must now enter against the one-way flow
    p = ds.path((13, 13))
    assert (13, 17) not in p and p[-1] == goal
    ds.notify(g.set_blocked(set()))
    ds.compute((13, 13))
    assert ds.cost_to_go((13, 13)) == pytest.approx(before)


def test_one_way_aisles_are_preferred_in_flow_direction(gm):
    g = RouteGraph(gm, DEFAULT)
    ds = DStarLite(g, (5, 20))                    # below aisle A5 (block B)
    ds.compute((5, 14))
    p = ds.path((5, 14))
    assert p[1] == (5, 15)                        # straight down the aisle
    ds2 = DStarLite(g, (5, 14))
    ds2.compute((5, 20))
    p2 = ds2.path((5, 20))
    assert (5, 17) not in p2                      # going up: around, not against the flow


def test_degraded_mode_only_canonical_entry(gm):
    g = RouteGraph(gm, DEFAULT)
    g.set_degraded(True)
    ds = DStarLite(g, (15, 10))                   # inside the S arm of the cross
    ds.compute((20, 8))
    p = ds.path((20, 8))
    assert p is not None and (15, 8) not in p     # may not cross the intersection in degraded mode
    assert (15, 12) in p                          # enters the S arm from its outer (canonical) end


def test_safe_intervals():
    assert safe_intervals([], 0.0) == [(0.0, INF)]
    assert safe_intervals([(2, 4), (3, 6), (10, 11)], 0.0) == [(0.0, 2), (6, 10), (11, INF)]


def test_sipp_waits_or_detours_around_reserved_aisle(gm):
    g = RouteGraph(gm, DEFAULT)
    goal = (13, 20)
    ds = DStarLite(g, goal)
    ds.compute((13, 13))
    free = sipp(g, (13, 13), goal, 0.0, {}, ds.cost_to_go, lambda c: c in gm.wide)
    assert free is not None
    t_free = free[1]
    # aisle A13 reserved for the next 30 s: SIPP must either wait (in a wide cell) or detour
    sec = gm.sections[gm.section_of((13, 16))]
    res = {c: [(0.0, 30.0)] for c in sec.cells}
    r = sipp(g, (13, 13), goal, 0.0, res, ds.cost_to_go, lambda c: c in gm.wide)
    path, t = r
    for c, ta in path:
        for a, b in res.get(c, []):
            assert not (a <= ta < b)
    assert t > t_free
    assert t < 30.0 + t_free                       # took the faster option (detour beats waiting)


def test_evaluate_path_counts_waits(gm):
    g = RouteGraph(gm, DEFAULT)
    p = [(12, 14), (13, 14), (13, 15), (13, 16)]
    base = evaluate_path(g, p, 0.0, {}, lambda c: c in gm.wide)
    res = {(13, 15): [(0.0, 10.0)]}
    assert evaluate_path(g, p, 0.0, res, lambda c: c in gm.wide) >= 10.0 > base


def test_space_time_astar_respects_reservations(gm):
    g = RouteGraph(gm, DEFAULT)
    goal = (8, 3)
    reserved = {((6, 3), t) for t in range(0, 6)}  # someone sits on (6,3) for 6 steps
    p = st_astar(g, (4, 3), goal, reserved, lambda c: abs(c[0] - goal[0]) + abs(c[1] - goal[1]))
    assert p[-1] == goal
    for t, c in enumerate(p):
        assert (c, t) not in reserved
