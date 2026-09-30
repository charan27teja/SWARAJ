"""L2 Contention: narrow aisles / intersections as mutual-exclusion resources.

Ricart & Agrawala (1981) distributed mutual exclusion with Lamport clocks, one RA instance per
critical section, extended for a fleet whose membership changes:

* **Live-peer membership.** A request is granted once every *live* peer (heard within
  `stale_s`) has replied. Peers that are not live are not waited for; when a peer (re)joins, all
  outstanding requests are re-sent to it and a grant that has not yet been used is revoked if the
  newcomer's beacon shows it holding the section.
* **Leased grants.** A held section carries a lease (expected traversal time + margin) that the
  holder renews in every beacon. A dead holder stops renewing, the lease expires, and peers mark
  the section *occupied-unknown* (see agent.py) - never *free*.
* **Sensor-confirmed entry** is enforced by the agent: RA grant **and** own-lidar clear check.

Messages (all reliable CoordEvents, idempotent, order-tolerant):
    REQ  {s, ts}     request section s with Lamport timestamp ts
    REP  {s, ts}     reply to the request (s, ts)
    REL  {s, ts}     release / cancel the request (s, ts)
"""
from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class Request:
    sid: int
    ts: int
    t_req: float
    replied: set = field(default_factory=set)
    granted: bool = False
    entered: bool = False
    lease: float = 0.0
    t_grant: float = 0.0


class LockManager:
    def __init__(self, me: int):
        self.me = me
        self.lamport = 0
        self.mine: dict[int, Request] = {}                  # sid -> my request/hold
        self.deferred: dict[int, set[tuple[int, int]]] = {}  # sid -> {(peer, ts)}
        self.peer_reqs: dict[int, dict[int, int]] = {}       # sid -> {peer: ts} outstanding requests seen
        self._released: set[tuple[int, int, int]] = set()    # (peer, sid, ts) released (REL before REQ)
        self.out: list[tuple[str, int | None, dict]] = []    # queued ("E", dst|None, body)

    # ------------------------------------------------------------------ clock
    def tick(self) -> int:
        self.lamport += 1
        return self.lamport

    def observe(self, ts: int) -> None:
        if ts > self.lamport:
            self.lamport = ts
        self.lamport += 1

    # ------------------------------------------------------------------ local API
    def request(self, sid: int, now: float) -> Request:
        r = self.mine.get(sid)
        if r is not None:
            return r
        ts = self.tick()
        r = Request(sid, ts, now)
        self.mine[sid] = r
        self.out.append(("E", None, {"k": "REQ", "s": sid, "ts": ts}))
        return r

    def release(self, sid: int) -> None:
        r = self.mine.pop(sid, None)
        if r is None:
            return
        self.out.append(("E", None, {"k": "REL", "s": sid, "ts": r.ts}))
        for peer, ts in self.deferred.pop(sid, set()):
            self.out.append(("E", peer, {"k": "REP", "s": sid, "ts": ts}))

    cancel = release

    def update_grants(self, live: set[int], holders: dict[int, int], now: float = 0.0) -> None:
        """Mark requests granted when every live peer replied. `holders` maps sid -> live peer
        whose beacon claims it holds sid (partition-heal reconciliation)."""
        for sid, r in self.mine.items():
            h = holders.get(sid)
            if r.granted and not r.entered and h is not None and h != self.me:
                r.granted = False                  # revoke unused grant: a live peer holds it
                r.replied.discard(h)
            if not r.granted and live <= r.replied and (h is None or h == self.me):
                r.granted = True
                r.t_grant = now

    def is_granted(self, sid: int) -> bool:
        r = self.mine.get(sid)
        return bool(r and r.granted)

    def missing(self, sid: int, live: set[int]) -> set[int]:
        r = self.mine.get(sid)
        return set() if r is None else live - r.replied

    def on_peer_joined(self, peer: int) -> None:
        for sid, r in self.mine.items():
            if not r.entered:
                self.out.append(("E", peer, {"k": "REQ", "s": sid, "ts": r.ts}))

    def on_peer_lost(self, peer: int) -> None:
        for sid in list(self.peer_reqs):
            self.peer_reqs[sid].pop(peer, None)
        for sid in list(self.deferred):
            self.deferred[sid] = {(p, ts) for p, ts in self.deferred[sid] if p != peer}

    # ------------------------------------------------------------------ messages
    def on_message(self, src: int, m: dict) -> None:
        k = m["k"]
        sid, ts = m["s"], m["ts"]
        if k == "REQ":
            self.observe(ts)
            if (src, sid, ts) in self._released:
                return
            self.peer_reqs.setdefault(sid, {})[src] = ts
            r = self.mine.get(sid)
            if r is not None and (r.granted or r.entered or (r.ts, self.me) < (ts, src)):
                self.deferred.setdefault(sid, set()).add((src, ts))
            else:
                self.out.append(("E", src, {"k": "REP", "s": sid, "ts": ts}))
        elif k == "REP":
            r = self.mine.get(sid)
            if r is not None and r.ts == ts:
                r.replied.add(src)
        elif k == "REL":
            self._released.add((src, sid, ts))
            if len(self._released) > 5000:
                self._released = set(list(self._released)[-2500:])
            pr = self.peer_reqs.get(sid)
            if pr and pr.get(src) == ts:
                del pr[src]

    def queue_ahead(self, sid: int, live: set[int]) -> list[int]:
        """Live peers requesting sid with RA precedence over my request (or all, if I have none)."""
        pr = self.peer_reqs.get(sid, {})
        r = self.mine.get(sid)
        out = []
        for p, ts in pr.items():
            if p in live and (r is None or (ts, p) < (r.ts, self.me)):
                out.append(p)
        return sorted(out, key=lambda p: (pr[p], p))

    def drain(self) -> list[tuple[str, int | None, dict]]:
        o, self.out = self.out, []
        return o
