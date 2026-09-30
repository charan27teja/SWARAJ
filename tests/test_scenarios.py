"""Scenario tests in the fast runtime: the phase "done means" criteria.

These run full multi-robot simulations (a few seconds each)."""
import ast
from pathlib import Path

import pytest

from amr.core.scenario import load_scenario, random_overlap
from amr.runtime.fast import FastSim, NetParams

ROOT = Path(__file__).resolve().parents[1]


def run(name_or_sc, **kw):
    sc = load_scenario(name_or_sc) if isinstance(name_or_sc, str) else name_or_sc
    sim = FastSim(sc, **kw)
    return sim, sim.run()


def assert_safe(r):
    assert r["collisions"] == 0, r
    assert r["section_violations"] == 0, r
    assert r["unresolved_deadlocks"] == 0, r


# ------------------------------------------------------------------ P0
def test_p0_reproducible_from_seed():
    a = FastSim(random_overlap(5, "low", seed=5)).run(until=60)
    b = FastSim(random_overlap(5, "low", seed=5)).run(until=60)
    for k in ("completed", "collisions", "min_robot_dist", "lock_requests", "bytes_per_robot_s"):
        assert a[k] == b[k], k


def test_p0_metrics_logged():
    _, r = run(random_overlap(5, "low", seed=1))
    for k in ("makespan", "throughput_per_h", "collisions", "mean_wait_s", "bytes_per_robot_s",
              "deadlocks", "duplicates", "section_violations"):
        assert k in r
    assert r["all_done"] and r["makespan"] > 0


# ------------------------------------------------------------------ P3
@pytest.mark.parametrize("net", [NetParams(), NetParams(latency_s=0.08, jitter_s=0.04, loss=0.2)])
def test_p3_zero_collisions_open_floor_with_and_without_network_faults(net):
    _, r = run(random_overlap(5, "low", seed=3), net=net)
    assert_safe(r)
    assert r["all_done"]


# ------------------------------------------------------------------ P4
def test_p4_head_on_never_deadlocks():
    _, r = run("head_on")
    assert_safe(r)
    assert r["all_done"]


def test_p4_circular_wait_detected_and_broken_quickly():
    sim, r = run("circular_wait")
    assert_safe(r)
    assert r["all_done"]
    types = [a["type"] for a in sim.rec.alerts]
    assert "deadlock_detected" in types and "deadlock_broken" in types
    assert r["max_ttr_s"] is None or r["max_ttr_s"] < 15.0


def test_p4_node_loss_inside_aisle_does_not_block_fleet():
    sim, r = run("node_loss")
    assert_safe(r)
    assert r["all_done"]
    kill = [f for f in sim.rec.faults if f["action"] == "kill"][0]
    assert sim.map.section_of(sim.map.cell_of(sim.world.bodies[2].x, sim.world.bodies[2].y)) is not None \
        or sim.world.bodies[2].present is False            # it was killed inside an aisle
    types = {a["type"] for a in sim.rec.alerts}
    assert "robot_lost" in types


def test_p4_partition_never_two_robots_in_one_aisle():
    _, r = run("partition")
    assert_safe(r)
    assert r["all_done"]


def test_p4_dead_zone():
    _, r = run("dead_zone")
    assert_safe(r)
    assert r["all_done"]


# ------------------------------------------------------------------ P5
def test_p5_blockage_rerouted_without_central_input():
    sim, r = run("blockage")
    assert_safe(r)
    assert r["all_done"]
    assert r["blockage_events"] >= 1                   # a robot's own lidar published it


# ------------------------------------------------------------------ P6
def test_p6_killed_robot_tasks_are_reauctioned():
    sim, r = run("node_loss")
    kill = [f for f in sim.rec.faults if f["action"] == "kill"][0]
    victim = kill["robot"]
    had = {j for (t, rr, j) in sim.rec.claims if rr == victim and t <= kill["applied"]}
    done_by_victim = {j for j, v in sim.rec.done.items() if any(x == victim for _, x in v)}
    orphan = had - done_by_victim
    assert orphan, "victim should die holding a task"
    for j in orphan:
        assert j in sim.rec.done and all(x != victim for _, x in sim.rec.done[j])
    assert r["reauctions"] >= 1


def test_p6_high_congestion_fleet_completes():
    _, r = run(random_overlap(5, "high", seed=2))
    assert_safe(r)
    assert r["all_done"]


# ------------------------------------------------------------------ architecture rule
AGENT_DIR = ROOT / "amr" / "agent"


def test_agent_code_is_pure():
    """No sockets, threads, wall clock or I/O inside agent logic."""
    banned = {"socket", "threading", "time", "asyncio", "subprocess", "multiprocessing", "select"}
    for f in AGENT_DIR.glob("*.py"):
        tree = ast.parse(f.read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                names = [n.name.split(".")[0] for n in node.names]
            elif isinstance(node, ast.ImportFrom):
                names = [(node.module or "").split(".")[0]]
            else:
                continue
            assert not (set(names) & banned), f"{f.name} imports {names}"
        assert "open(" not in f.read_text(encoding="utf-8"), f.name


def test_agents_share_no_mutable_state_in_fast_runtime():
    sim = FastSim(random_overlap(5, "low", seed=1))
    a, b = sim.agents[1], sim.agents[2]
    for attr in ("peers", "lm", "graph", "book", "alloc", "blockages", "dstar"):
        assert getattr(a, attr) is not getattr(b, attr), attr
