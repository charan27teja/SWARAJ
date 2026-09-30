"""L2 Deadlock detect-and-break.

Every robot publishes its current wait-for edge (`wf`, a robot id or null) and its priority in
its beacon. Each robot builds the wait-for graph locally from the beacons it has heard and runs
cycle detection. The lowest-priority member of a cycle yields. Priority = base + alpha * t_wait
(ageing), ties broken by robot id (lower id = higher priority). The same function orders L3
planning. Views can briefly disagree; each robot only ever decides for itself, so the worst
case is an extra (safe) yield.
"""
from __future__ import annotations


def priority(base: float, t_wait: float, alpha: float) -> float:
    return base + alpha * max(0.0, t_wait)


def prio_key(prio: float, rid: int) -> tuple[float, int]:
    """Sort key: larger means higher priority."""
    return (round(prio, 3), -rid)


def find_cycle(edges: dict[int, int | None], start: int) -> list[int] | None:
    """Follow single out-edges from `start`; return the cycle reached (if any) as a list."""
    seen: dict[int, int] = {}
    order: list[int] = []
    node = start
    while node is not None and node not in seen:
        seen[node] = len(order)
        order.append(node)
        node = edges.get(node)
    if node is None:
        return None
    return order[seen[node]:]


def all_cycles(edges: dict[int, int | None]) -> list[list[int]]:
    out, done = [], set()
    for n in edges:
        c = find_cycle(edges, n)
        if c:
            key = frozenset(c)
            if key not in done:
                done.add(key)
                out.append(c)
    return out


def choose_victim(cycle: list[int], prios: dict[int, float]) -> int:
    return min(cycle, key=lambda r: prio_key(prios.get(r, 0.0), r))
