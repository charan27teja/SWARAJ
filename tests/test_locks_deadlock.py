"""P4: Ricart-Agrawala ordering, membership, leases; wait-for cycle detection and victim choice."""
from amr.agent.locks import LockManager
from amr.agent.deadlock import find_cycle, all_cycles, choose_victim, priority, prio_key


def deliver(src: LockManager, dsts: dict):
    """Deliver src's queued events to the other managers; returns count."""
    n = 0
    for cls, dst, body in src.drain():
        targets = [d for d in dsts if d != src.me] if dst is None else [dst]
        for d in targets:
            dsts[d].on_message(src.me, body)
            n += 1
    return n


def pump(mgrs):
    while sum(deliver(m, mgrs) for m in mgrs.values()):
        pass


def test_ra_lower_timestamp_wins_and_release_hands_over():
    m = {i: LockManager(i) for i in (1, 2, 3)}
    live = {1: {2, 3}, 2: {1, 3}, 3: {1, 2}}
    m[2].request(7, 0.0)
    pump(m)                       # robot 1 has now seen ts=1 ...
    m[1].request(7, 0.0)          # ... so its own request is causally later (higher Lamport ts)
    assert m[1].mine[7].ts > m[2].mine[7].ts
    pump(m)
    for i in m:
        m[i].update_grants(live[i], {})
    assert m[2].is_granted(7) and not m[1].is_granted(7)
    m[2].release(7)
    pump(m)
    m[1].update_grants(live[1], {})
    assert m[1].is_granted(7)


def test_ra_tie_on_timestamp_broken_by_robot_id():
    m = {i: LockManager(i) for i in (1, 2)}
    m[1].request(3, 0.0)
    m[2].request(3, 0.0)          # both Lamport ts == 1
    assert m[1].mine[3].ts == m[2].mine[3].ts
    pump(m)
    m[1].update_grants({2}, {})
    m[2].update_grants({1}, {})
    assert m[1].is_granted(3) and not m[2].is_granted(3)


def test_ra_only_live_peers_are_waited_for():
    m = {i: LockManager(i) for i in (1, 2, 3)}
    m[1].request(5, 0.0)
    deliver(m[1], {2: m[2]})       # robot 3 is dead: message never arrives
    deliver(m[2], m)
    m[1].update_grants({2}, {})    # 3 is not live -> not required
    assert m[1].is_granted(5)


def test_ra_rejoin_resends_request_and_revokes_unused_grant():
    m = {i: LockManager(i) for i in (1, 2)}
    m[1].request(4, 0.0)
    m[1].drain()                   # partitioned: nothing delivered
    m[1].update_grants(set(), {})  # alone in its island -> granted
    assert m[1].is_granted(4)
    # heal: robot 2's beacon says it holds section 4
    m[1].update_grants({2}, {4: 2})
    assert not m[1].is_granted(4)
    m[1].on_peer_joined(2)
    assert any(b["k"] == "REQ" and d == 2 for _, d, b in m[1].drain())


def test_ra_release_before_request_is_idempotent():
    m = LockManager(1)
    m.on_message(2, {"k": "REL", "s": 9, "ts": 5})
    m.on_message(2, {"k": "REQ", "s": 9, "ts": 5})      # stale, already released
    assert 2 not in m.peer_reqs.get(9, {})


def test_lamport_clock_monotone():
    m = LockManager(1)
    a = m.tick()
    m.observe(40)
    assert m.tick() > 41 > a


def test_find_cycle_and_victim_with_ageing():
    edges = {1: 2, 2: 3, 3: 1, 4: 1}
    assert set(find_cycle(edges, 4)) == {1, 2, 3}
    assert find_cycle({1: 2, 2: None}, 1) is None
    assert len(all_cycles(edges)) == 1
    prios = {1: priority(0.0, 1.0, 0.2), 2: priority(0.0, 10.0, 0.2), 3: priority(1.0, 0.0, 0.2)}
    assert choose_victim([1, 2, 3], prios) == 1          # waited least, carries nothing
    # equal priority -> higher id yields (tie-break by id)
    assert choose_victim([1, 2, 3], {1: 0.0, 2: 0.0, 3: 0.0}) == 3
    assert prio_key(1.0, 5) > prio_key(1.0, 6)
