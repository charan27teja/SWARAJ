"""P1 (live runtime, 5 real robot processes over UDP): robots see each other, a killed robot is detected
within ~500 ms, and injected loss / latency behave as configured."""
import json
import os
import socket
import tempfile
import time
from pathlib import Path

import pytest

from amr import faults
from amr.core.config import DEFAULT
from amr.runtime.launcher import Fleet
from amr.transport.faultshim import FaultShim
from amr.transport.udp import UdpTransport

pytestmark = pytest.mark.live
ALL = {1, 2, 3, 4, 5}


class Observer:
    """Stands where the metrics recorder would: receives the copies robots send to observers."""

    def __init__(self, port=9051):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.bind(("127.0.0.1", port))
        self.s.settimeout(0.05)
        self.beacons = {}
        self.alerts = []

    def pump(self, secs):
        end = time.time() + secs
        while time.time() < end:
            try:
                d, _ = self.s.recvfrom(65536)
            except OSError:
                continue
            p = json.loads(d)
            if p.get("t") == "B":
                self.beacons[p["b"]["id"]] = (time.time(), p["b"])
            elif p.get("t") == "O" and p["b"].get("k") == "ALERT":
                self.alerts.append(p["b"])

    def close(self):
        self.s.close()


def live_ids(b):
    return {i for i in range(1, 63) if b.get("lv", 0) >> i & 1}


def test_five_processes_see_each_other_and_kill_is_detected():
    obs = Observer()
    fleet = Fleet("demo", bridge=False).start()
    try:
        deadline = time.time() + 15
        ok = False
        while time.time() < deadline and not ok:
            obs.pump(0.5)
            ok = len(obs.beacons) == 5 and all(live_ids(b) == ALL - {r} for r, (_, b) in obs.beacons.items())
        assert ok, {r: live_ids(b) for r, (_, b) in obs.beacons.items()}
        obs.pump(2.0)
        t_kill = time.time()
        assert faults.kill_proc("3")
        obs.pump(3.0)
        lost = [a for a in obs.alerts if a["type"] == "robot_lost" and 3 in a["robots"]]
        by = {a["by"]: a["t"] for a in lost}
        assert set(by) == ALL - {3}, obs.alerts
        latency = [t - (t_kill - fleet.t0) for t in by.values()]
        print("kill detection latency (s):", [round(x, 3) for x in latency])
        # detection = stale threshold after the last beacon (which precedes the kill by <= 100 ms)
        # + loop granularity; the requirement is <= 500 ms after the kill
        lo = DEFAULT.stale_s - 0.1 - 0.02
        assert all(lo <= x <= 0.50 for x in latency), latency
        # the rest of the fleet keeps working
        before = {r: obs.beacons[r][1]["q"] for r in ALL - {3}}
        obs.pump(1.0)
        assert all(obs.beacons[r][1]["q"] > before[r] + 5 for r in ALL - {3})
        assert all(live_ids(obs.beacons[r][1]) == ALL - {3, r} for r in ALL - {3})
    finally:
        fleet.stop()
        obs.close()


def test_injected_loss_and_latency_behave_as_configured(tmp_path):
    rules = tmp_path / "faults.json"
    rules.write_text("{}")
    a = UdpTransport(1, [1, 2], port_base=9300, rules_file=str(rules))
    b = UdpTransport(2, [1, 2], port_base=9300, rules_file=str(rules))
    try:
        def run(n=400):
            got, lat = 0, []
            for k in range(n):
                now = time.time()
                a.tick(now)
                a.send([("B", None, {"k": "B", "id": 1, "q": k, "ts": now})], now)
                time.sleep(0.002)
                for src, body in b.poll(time.time()):
                    got += 1
                    lat.append(time.time() - body["ts"])
            end = time.time() + 0.3
            while time.time() < end:
                a.tick(time.time())
                for src, body in b.poll(time.time()):
                    got += 1
                    lat.append(time.time() - body["ts"])
                time.sleep(0.002)
            return got / n, lat

        rate, lat = run()
        assert rate > 0.98 and max(lat) < 0.05
        rules.write_text(json.dumps({"loss": 0.3}))
        a.rules._next = 0
        a.tick(time.time())
        rate, _ = run()
        print("delivery with loss 0.3:", rate)
        assert 0.62 <= rate <= 0.78
        rules.write_text(json.dumps({"delay_ms": 80, "jitter_ms": 10}))
        a.rules._next = 0
        a.tick(time.time())
        rate, lat = run(200)
        lat.sort()
        med = lat[len(lat) // 2]
        print("median latency with delay 80 ms:", round(med * 1000, 1), "ms")
        assert rate > 0.98 and 0.075 <= med <= 0.100
    finally:
        a.close()
        b.close()
