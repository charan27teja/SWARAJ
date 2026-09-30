"""UDP peer-to-peer transport for the live runtime. No broker, no relay.

* Every robot binds `port_base + id` on its host. Peers are discovered from the configured
  port range on the configured host list (default 127.0.0.1), so the same code works across
  real machines.
* Beacons are best-effort datagrams; CoordEvents go through `ReliableEndpoint` (seq/ACK/
  retransmit/dedupe).
* A copy of every message is sent to the observer ports (dashboard bridge, metrics recorder)
  if they exist. Sending to a closed UDP port never blocks; on Windows the resulting ICMP
  "port unreachable" surfaces as ConnectionResetError on a later recvfrom, which the receive
  loop skips (Python does not expose SIO_UDP_CONNRESET to switch it off).
* The FaultShim (rules file written by `python -m amr.faults`) is applied here, below the
  agent: each side enforces the rules for its own end (partition, loss, delay, dead zone of its
  own position). Robot logic never sees it.
"""
from __future__ import annotations

import heapq
import json
import socket
import time

from amr.transport.faultshim import FaultShim, FileRules
from amr.transport.reliable import ReliableEndpoint

_enc = json.JSONEncoder(separators=(",", ":")).encode
MAX_DGRAM = 60000


def make_socket(port: int, host: str = "127.0.0.1") -> socket.socket:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    if hasattr(socket, "SIO_UDP_CONNRESET"):
        try:
            s.ioctl(socket.SIO_UDP_CONNRESET, False)
        except OSError:
            pass
    s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
    s.bind((host, port))
    s.setblocking(False)
    return s


def safe_send(sock: socket.socket, data: bytes, addr) -> bool:
    try:
        sock.sendto(data, addr)
        return True
    except OSError:
        return False     # peer gone / buffer full: best effort, never block


class UdpTransport:
    def __init__(self, me: int, fleet: list[int], port_base: int = 9100, hosts: dict | None = None,
                 bind_host: str = "127.0.0.1", observers: list[tuple[str, int]] | None = None,
                 rules_file: str | None = None, seed: int = 0):
        self.me = me
        self.fleet = [r for r in fleet if r != me]
        self.port_base = port_base
        self.hosts = hosts or {}
        self.sock = make_socket(port_base + me, bind_host)
        self.addr = {r: (self.hosts.get(r, "127.0.0.1"), port_base + r) for r in self.fleet}
        self.observers = observers or []
        self.ep = ReliableEndpoint(me)
        self.shim = FaultShim({}, seed=seed + me)
        self.rules = FileRules(rules_file, self.shim) if rules_file else None
        self.pos: tuple[float, float] | None = None
        self._delayed: list = []
        self._n = 0
        self.bytes_sent = 0
        self.msgs_sent = 0
        self.beacon_bytes = 0
        self.beacons = 0

    # ------------------------------------------------------------------ send
    def _link_ok(self, peer: int) -> float | None:
        # this end only knows its own position; the peer enforces its own dead zone
        return self.shim.deliver(self.me, peer, self.pos, None)

    def _out(self, data: bytes, peer: int, now: float) -> None:
        lat = self._link_ok(peer)
        self.bytes_sent += len(data)
        self.msgs_sent += 1
        if lat is None:
            return
        if lat > 0:
            self._n += 1
            heapq.heappush(self._delayed, (now + lat, self._n, data, peer))
        else:
            safe_send(self.sock, data, self.addr[peer])

    def send(self, outbox: list, now: float) -> None:
        for cls, dst, body in outbox:
            if cls == "B":
                data = _enc({"t": "B", "b": body}).encode()
                self.beacon_bytes += len(data)
                self.beacons += 1
                for p in self.fleet:
                    self._out(data, p, now)
                self._observe(data)
            elif cls == "E":
                for p in (self.fleet if dst is None else [dst]):
                    pkt = self.ep.send(p, body, now)
                    self._out(_enc(pkt).encode(), p, now)
                self._observe(_enc({"t": "O", "src": self.me, "b": body}).encode())
            elif cls == "O":
                self._observe(_enc({"t": "O", "src": self.me, "b": body}).encode())

    def _observe(self, data: bytes) -> None:
        for a in self.observers:
            safe_send(self.sock, data, a)

    def tick(self, now: float) -> None:
        if self.rules:
            self.rules.poll(now)
        for pkt in self.ep.due(now):
            if pkt["dst"] in self.addr:
                self._out(_enc(pkt).encode(), pkt["dst"], now)
        while self._delayed and self._delayed[0][0] <= now:
            _, _, data, peer = heapq.heappop(self._delayed)
            safe_send(self.sock, data, self.addr[peer])

    # ------------------------------------------------------------------ receive
    def poll(self, now: float) -> list[tuple[int, dict]]:
        inbox = []
        while True:
            try:
                data, addr = self.sock.recvfrom(MAX_DGRAM)
            except BlockingIOError:
                break
            except ConnectionResetError:
                continue      # Windows: ICMP 'port unreachable' after sending to a dead peer; keep reading
            except OSError:
                break
            try:
                pkt = json.loads(data)
            except ValueError:
                continue
            t = pkt.get("t")
            if t == "B":
                src = pkt["b"].get("id")
            else:
                src = pkt.get("src")
            if src is None or src == self.me or src not in self.addr:
                continue
            # receiving end enforces partition / loss / its own dead zone
            if not self.shim.link_up(src, self.me, None, self.pos):
                continue
            if t == "B":
                inbox.append((src, pkt["b"]))
            elif t in ("E", "A"):
                body, ack = self.ep.on_packet(pkt)
                if ack is not None:
                    self._out(_enc(ack).encode(), src, now)
                if body is not None:
                    inbox.append((src, body))
        return inbox

    def close(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass


def now_s() -> float:
    return time.time()
