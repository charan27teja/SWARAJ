"""Live metrics recorder: a passive test instrument on an observer port. Robots send it copies
of their messages (fire-and-forget); the world streams ground truth. It never sends anything to
robots, and killing it has no effect on the fleet.

    python -m amr.bench.recorder --scenario demo [--port 9051]

Writes runs/live/metrics.json (rolling summary) and runs/live/events.jsonl.
"""
from __future__ import annotations

import argparse
import json
import os
import time

from amr.bench.metrics import MetricsRecorder
from amr.core.scenario import load_scenario, ROOT
from amr.transport.udp import make_socket

RUN_DIR = ROOT / "runs" / "live"


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", default="demo")
    ap.add_argument("--port", type=int, default=9051)
    a = ap.parse_args(argv)
    sc = load_scenario(a.scenario)
    RUN_DIR.mkdir(parents=True, exist_ok=True)
    (RUN_DIR / "recorder.pid").write_text(str(os.getpid()))
    rec = MetricsRecorder(len(sc.tasks), [r.rid for r in sc.robots])
    sock = make_socket(a.port)
    truth = {"collisions": 0, "wall_contacts": 0, "obstacle_contacts": 0, "min_robot_dist": None}
    t0 = time.time()
    nxt = 0.0
    gm = sc.map
    sec_viol, sec_prev = 0, set()
    ev = open(RUN_DIR / "events.jsonl", "a", encoding="utf-8")
    print(f"[recorder] listening on {a.port}", flush=True)
    while True:
        now = time.time()
        while True:
            try:
                data, _ = sock.recvfrom(65536)
            except ConnectionResetError:
                continue      # Windows: ICMP 'port unreachable' from a dead peer; keep reading
            except (BlockingIOError, OSError):
                break
            try:
                pkt = json.loads(data)
            except ValueError:
                continue
            t = pkt.get("t")
            if t == "B":
                b = pkt["b"]
                rec.beacon_bytes.append(len(data))
                rec.on_sent(b["id"], len(data))
                rec.on_message(b["id"], b, now - t0)
            elif t == "O":
                body = pkt["b"]
                rec.on_message(pkt["src"], body, now - t0)
                if body.get("k") in ("ALERT", "DONE", "CLAIM", "BLOCK", "PICK"):
                    ev.write(json.dumps({"t": round(now - t0, 3), "src": pkt["src"], **body}) + "\n")
            elif t == "W":
                truth = pkt["b"]
                occ = {}
                for rid, b in truth.get("robots", {}).items():
                    if b.get("present", True):
                        sid = gm.section_of(gm.cell_of(b["x"], b["y"]))
                        if sid is not None:
                            occ.setdefault(sid, []).append(rid)
                now_v = {sid for sid, rs in occ.items() if len(rs) > 1}
                sec_viol += len(now_v - sec_prev)
                sec_prev = now_v
        rec.tick(now - t0)
        if now >= nxt:
            nxt = now + 1.0
            ev.flush()
            s = rec.summary(now - t0, truth, rec.robots)
            s["wall_clock"] = now
            s["section_violations"] = sec_viol
            tmp = RUN_DIR / "metrics.tmp"
            tmp.write_text(json.dumps(s, indent=1), encoding="utf-8")
            os.replace(tmp, RUN_DIR / "metrics.json")
        time.sleep(0.01)


if __name__ == "__main__":
    main()
