"""Neighbour table built only from received StateBeacons (newest-wins by sequence number)."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass
class Peer:
    rid: int
    beacon: dict
    recv_t: float
    first_t: float

    @property
    def seq(self) -> int:
        return self.beacon.get("q", -1)


class PeerTable:
    def __init__(self, me: int, stale_s: float):
        self.me = me
        self.stale_s = stale_s
        self.peers: dict[int, Peer] = {}

    def on_beacon(self, b: dict, now: float) -> bool:
        rid = b.get("id")
        if rid is None or rid == self.me:
            return False
        p = self.peers.get(rid)
        if p is None:
            self.peers[rid] = Peer(rid, b, now, now)
            return True
        if b.get("q", -1) > p.seq or b.get("q", 0) < p.seq - 1000:   # newest wins (or peer restarted)
            p.beacon = b
            p.recv_t = now
            return True
        return False

    def age(self, rid: int, now: float) -> float:
        p = self.peers.get(rid)
        return float("inf") if p is None else now - p.recv_t

    def live(self, now: float) -> set[int]:
        return {r for r, p in self.peers.items() if now - p.recv_t <= self.stale_s}

    def ever(self) -> set[int]:
        return set(self.peers)

    def get(self, rid: int) -> dict | None:
        p = self.peers.get(rid)
        return None if p is None else p.beacon

    def predicted(self, rid: int, now: float, max_extrap: float = 0.3) -> tuple[float, float, float, float] | None:
        p = self.peers.get(rid)
        if p is None:
            return None
        b = p.beacon
        x, y = b["p"][0], b["p"][1]
        vx, vy = b.get("v", (0.0, 0.0))
        a = min(max_extrap, now - p.recv_t)
        return (x + vx * a, y + vy * a, vx, vy)
