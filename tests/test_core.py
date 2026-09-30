"""P0: map loading, critical-section / passing-bay extraction, scenario generation, world."""
import math

import numpy as np
import pytest

from amr.core.grid_map import GridMap
from amr.core.scenario import random_overlap, load_scenario
from amr.world.world import World
from amr.core.config import DEFAULT


@pytest.fixture(scope="module")
def gm():
    from amr.core.scenario import ROOT
    return GridMap.load(ROOT / "maps/warehouse_a.yaml")


def test_map_dimensions_and_stations(gm):
    assert (gm.width, gm.height) == (30, 25)
    assert len(gm.picks) == 20 and len(gm.drops) == 6 and len(gm.docks) == 6
    assert len(gm.docks) + len(gm.bays) >= 5           # a parking spot for every robot


def test_sections_are_narrow_and_extracted(gm):
    kinds = [s.kind for s in gm.sections]
    assert kinds.count("intersection") == 1
    assert kinds.count("corridor") == 14      # 10 one-way picking aisles + 4 cross arms
    for s in gm.sections:
        for c in s.cells:
            assert c not in gm.wide, "a critical section cell must be narrower than 2 diameters"
    x = [s for s in gm.sections if s.kind == "intersection"][0]
    assert x.cells == [(15, 8)] and len(x.ends) == 4        # 4-way intersection (circular wait)


def test_every_pick_is_inside_a_section(gm):
    for p in gm.picks:
        assert gm.section_of(p) is not None


def test_extraction_on_tiny_synthetic_map():
    rows = ["#######",
            "#.....#",
            "#.....#",
            "###.###",
            "###.###",
            "#.....#",
            "#.....#",
            "##B####"]
    m = GridMap(rows)
    assert len(m.sections) == 1
    s = m.sections[0]
    assert s.cells == [(3, 3), (3, 4)] and s.kind == "corridor"
    assert m.bays == [(2, 7)]


def test_one_way_aisles(gm):
    a = [s for s in gm.sections if s.label == "A5:15-19"][0]
    assert a.one_way and a.canonical_outside == (5, 14) and a.exit_outside == (5, 20)
    arm = [s for s in gm.sections if s.label == "A12-14:8"][0]
    assert not arm.one_way                                   # cross arms stay bidirectional


def test_scenario_generator_is_reproducible():
    a = random_overlap(5, "medium", seed=4)
    b = random_overlap(5, "medium", seed=4)
    c = random_overlap(5, "medium", seed=5)
    assert a.to_json() == b.to_json()
    assert a.to_json() != c.to_json()
    assert len(a.tasks) == 15 and len(a.robots) == 5
    assert len({r.cell for r in a.robots}) == 5


def test_named_scenarios_load():
    for name in ("demo", "head_on", "circular_wait", "blockage", "node_loss", "dead_zone", "partition"):
        sc = load_scenario(name)
        assert len(sc.robots) == 5, name


def test_world_kinematics_and_accel_limit(gm):
    w = World(gm, DEFAULT)
    w.add_robot(1, 2.5, 2.5, 0.0)
    w.set_cmd(1, 1.0, 0.0)
    w.step(0.05)
    assert w.bodies[1].v == pytest.approx(DEFAULT.a_max * 0.05)
    for _ in range(40):
        w.step(0.05)
    assert w.bodies[1].v == pytest.approx(1.0)
    assert w.bodies[1].x > 3.5 and w.bodies[1].y == pytest.approx(2.5)


def test_world_blocks_wall_penetration(gm):
    w = World(gm, DEFAULT)
    w.add_robot(1, 1.5, 2.5, math.pi)       # facing the left wall at x=1
    w.set_cmd(1, 1.0, 0.0)
    for _ in range(40):
        w.step(0.05)
    assert w.bodies[1].x >= 1.0 + DEFAULT.radius - 0.02
    assert any(e.kind == "wall" for e in w.events)


def test_world_detects_robot_collision(gm):
    w = World(gm, DEFAULT)
    w.add_robot(1, 5.0, 2.5, 0.0)
    w.add_robot(2, 7.0, 2.5, math.pi)
    for _ in range(60):
        w.set_cmd(1, 1.0, 0.0)
        w.set_cmd(2, 1.0, 0.0)
        w.step(0.05)
    assert w.robot_collisions == 1          # one contact event, not one per step


def test_lidar_sees_walls_robots_and_blockages(gm):
    w = World(gm, DEFAULT)
    w.add_robot(1, 5.5, 2.5, 0.0)           # top lane, facing +x
    w.add_robot(2, 8.5, 2.5, 0.0)
    scans = w.scans([1, 2])
    r = scans[1]
    k0 = int(np.argmin(np.abs(w.ray_offsets)))           # ray straight ahead
    assert r[k0] == pytest.approx(3.0 - DEFAULT.radius, abs=0.05)
    kup = int(np.argmin(np.abs(w.ray_offsets + math.pi / 2)))   # -y: wall row 1 at y=2 (pocket rows aside)
    assert r[kup] < 1.6
    w.add_blockage(5.5, 3.5, 0.4)
    kdown = int(np.argmin(np.abs(w.ray_offsets - math.pi / 2)))
    assert w.scans([1])[1][kdown] == pytest.approx(0.6, abs=0.05)


def test_battery_drains_and_charges(gm):
    w = World(gm, DEFAULT)
    dock = gm.docks[0]
    w.add_robot(1, dock[0] + 0.5, dock[1] + 0.5, 0.0, battery=50.0)
    for _ in range(100):
        w.step(0.05)
    assert w.bodies[1].battery > 50.0       # charging on the dock
    w.add_robot(2, 5.5, 2.5, 0.0, battery=50.0)
    w.set_cmd(2, 1.0, 0.0)
    for _ in range(100):
        w.step(0.05)
    assert w.bodies[2].battery < 50.0
