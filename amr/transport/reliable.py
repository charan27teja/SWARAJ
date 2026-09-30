"""Reliable CoordEvent delivery over an unreliable datagram link: sequence numbers, ACK,
retransmit and receiver-side dedupe. Pure (no sockets, no clock): the caller passes `now`.

Delivery is at-least-once with dedupe, i.e. effectively exactly-once, but *not* ordered. The
coordination messages are designed to be idempotent and order-tolerant (e.g. a lock RELEASE
carries the timestamp of the request it releases), so no head-of-line blocking is needed.

Wire packets are plain dicts:
    {"t": "E", "src": i, "dst": j, "q": seq, "b": body}     reliable event
    {"t": "A", "src": j, "dst": i, "q": seq}                 acknowledgement
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass
class _Pending:
    pkt: dict
    first_sent: float
    next_try: float


class ReliableEndpoint:
    def __init__(self, me: int, retry_s: float = 0.15, give_up_s: float = 20.0, window: int = 4096):
        self.me = me
        self.retry_s = retry_s
        self.give_up_s = give_up_s
        self.window = window
        self._seq = 0
        self._pending: dict[tuple[int, int], _Pending] = {}
        self._seen: dict[int, set[int]] = {}
        self._seen_max: dict[int, int] = {}
        self.stats = {"sent": 0, "retx": 0, "acked": 0, "dup": 0, "gave_up": 0}

    def send(self, dst: int, body: dict, now: float) -> dict:
        self._seq += 1
        pkt = {"t": "E", "src": self.me, "dst": dst, "q": self._seq, "b": body}
        self._pending[(dst, self._seq)] = _Pending(pkt, now, now + self.retry_s)
        self.stats["sent"] += 1
        return pkt

    def on_packet(self, pkt: dict) -> tuple[dict | None, dict | None]:
        """Handle an incoming E or A packet. Returns (body to deliver or None, ack to send or None)."""
        if pkt.get("t") == "A":
            if self._pending.pop((pkt["src"], pkt["q"]), None) is not None:
                self.stats["acked"] += 1
            return None, None
        src, q = pkt["src"], pkt["q"]
        ack = {"t": "A", "src": self.me, "dst": src, "q": q}
        seen = self._seen.setdefault(src, set())
        mx = self._seen_max.get(src, 0)
        if q in seen or q <= mx - self.window:
            self.stats["dup"] += 1
            return None, ack
        seen.add(q)
        if q > mx:
            self._seen_max[src] = q
            if len(seen) > 2 * self.window:
                lo = q - self.window
                self._seen[src] = {s for s in seen if s > lo}
        return pkt["b"], ack

    def due(self, now: float) -> list[dict]:
        """Packets to retransmit now."""
        out = []
        dead = []
        for key, p in self._pending.items():
            if now - p.first_sent > self.give_up_s:
                dead.append(key)
            elif now >= p.next_try:
                p.next_try = now + self.retry_s
                out.append(p.pkt)
        for key in dead:
            del self._pending[key]
            self.stats["gave_up"] += 1
        self.stats["retx"] += len(out)
        return out

    def drop_dest(self, dst: int) -> None:
        for key in [k for k in self._pending if k[0] == dst]:
            del self._pending[key]

    @property
    def backlog(self) -> int:
        return len(self._pending)
