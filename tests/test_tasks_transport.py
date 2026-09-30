"""P6/P1: auction conflict resolution, CBBA consensus, battery feasibility; reliable transport and
fault shim."""
import random

import pytest

from amr.core.config import DEFAULT
from amr.agent.tasks import CBBA, SequentialAuction, Task, TaskBook, better
from amr.transport.reliable import ReliableEndpoint
from amr.transport.faultshim import FaultShim


def manhattan(a, b):
    return float(abs(a[0] - b[0]) + abs(a[1] - b[1]))


def make_book(me, tasks, battery_dock=5.0):
    bk = TaskBook(me, DEFAULT, manhattan, manhattan, lambda c: battery_dock)
    for t in tasks:
        bk.add(t)
    return bk


TASKS = [Task(1, (2, 2), (2, 10)), Task(2, (10, 2), (10, 10)), Task(3, (20, 2), (20, 10)),
         Task(4, (6, 6), (6, 12))]


def test_better_tie_breaks_by_lower_id():
    assert better(5.0, 1, 5.0, 2) and not better(5.0, 2, 5.0, 1)
    assert better(5.1, 9, 5.0, 1)
    assert not better(0.0, None, 1.0, 2)


def run_cbba(starts, battery=100.0, rounds=20):
    agents = {i: CBBA(make_book(i, TASKS)) for i in starts}
    for _ in range(rounds):
        for i, a in agents.items():
            a.build(0.0, starts[i], 0.0, battery)
        msgs = {i: a.message(0.0) for i, a in agents.items()}
        for i, a in agents.items():
            for k, m in msgs.items():
                if k != i:
                    a.on_bid_message(k, m, 0.0)
    return agents


def test_cbba_converges_to_conflict_free_assignment():
    starts = {1: (2, 0), 2: (10, 0), 3: (20, 0)}
    agents = run_cbba(starts)
    owners = {}
    for i, a in agents.items():
        for j in a.path:
            assert j not in owners, f"task {j} in two bundles"
            owners[j] = i
    # consensus: every agent agrees on every winner
    for j in range(1, 5):
        zs = {a.z.get(j) for a in agents.values()}
        assert len(zs) == 1
    assert owners.get(1) == 1 and owners.get(2) == 2 and owners.get(3) == 3


def test_cbba_battery_filter_blocks_infeasible_bids():
    agents = run_cbba({1: (2, 0), 2: (10, 0)}, battery=DEFAULT.battery_reserve + 0.5)
    assert all(not a.path for a in agents.values())


def test_battery_feasibility_accounts_for_return_to_dock():
    bk = make_book(1, TASKS, battery_dock=50.0)
    _, ok_full = bk.path_eval([1], (2, 0), 0.0, 0.0, 100.0)
    _, ok_low = bk.path_eval([1], (2, 0), 0.0, 0.0, 12.0 + 50 * DEFAULT.battery_per_m)
    assert ok_full and not ok_low


def test_sequential_auction_deterministic_winner():
    books = {i: make_book(i, TASKS) for i in (1, 2)}
    auc = {i: SequentialAuction(books[i], window=0.3) for i in (1, 2)}
    starts = {1: (2, 0), 2: (2, 0)}                  # identical bids -> lower id wins
    for i in auc:
        auc[i].build(0.0, starts[i], 0.0, 100.0)
    for i in auc:
        for m in auc[i].message(0.0):
            auc[3 - i].on_bid_message(i, m, 0.0)
    for i in auc:
        auc[i].build(0.5, starts[i], 0.0, 100.0)
    assert auc[1].path == [1] and auc[2].path == []


def test_reliable_endpoint_retransmits_and_dedupes():
    a, b = ReliableEndpoint(1, retry_s=0.1), ReliableEndpoint(2)
    pkt = a.send(2, {"k": "REQ", "s": 1, "ts": 3}, 0.0)
    assert a.due(0.05) == [] and a.due(0.11) == [pkt]      # not acked -> retransmitted
    body, ack = b.on_packet(pkt)
    assert body["k"] == "REQ"
    body2, ack2 = b.on_packet(pkt)                          # duplicate delivery
    assert body2 is None and ack2 is not None
    a.on_packet(ack)
    assert a.backlog == 0 and a.due(1.0) == []


def test_reliable_delivery_over_lossy_link():
    rng = random.Random(1)
    a, b = ReliableEndpoint(1, retry_s=0.1), ReliableEndpoint(2)
    got = set()
    t = 0.0
    wire = [a.send(2, {"n": n}, t) for n in range(50)]
    while (a.backlog or wire) and t < 30:
        nxt = []
        for p in wire:
            if rng.random() < 0.4:                          # 40 % loss each way
                continue
            if p["t"] == "E":
                body, ack = b.on_packet(p)
                if body:
                    got.add(body["n"])
                nxt.append(ack)
            else:
                a.on_packet(p)
        t += 0.05
        wire = nxt + a.due(t)
    assert got == set(range(50))


def test_fault_shim_partition_deadzone_loss_delay():
    s = FaultShim({"partition": [[1, 2], [3]], "dead_zones": [[10, 10, 20, 13]]}, seed=0)
    assert s.link_up(1, 2) and not s.link_up(1, 3)
    assert not s.link_up(1, 2, pos_src=(15, 11)) and s.link_up(1, 2, pos_src=(5, 5))
    s.set_rules({"loss": 0.3, "delay_ms": 50, "jitter_ms": 10})
    res = [s.deliver(1, 2) for _ in range(4000)]
    lost = sum(r is None for r in res) / len(res)
    assert lost == pytest.approx(0.3, abs=0.03)
    lat = [r for r in res if r is not None]
    assert 0.040 <= min(lat) and max(lat) <= 0.060
    s.set_rules({})
    assert s.deliver(1, 2) == 0.0
