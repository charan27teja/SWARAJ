"""One robot = one OS process. Runs the identical pure Agent at 20 Hz.

    python -m amr.runtime.live_robot --id 1 --scenario demo

Inputs: its own pose/battery/scan from the world (private channel) and peer messages over UDP.
Outputs: velocity command to the world, beacons/events to peers, copies to observers.
"""
from __future__ import annotations

import argparse
import json
import os
import time
from pathlib import Path

import numpy as np

from amr.agent.agent import Agent
from amr.core.config import DEFAULT
from amr.core.scenario import load_scenario, ROOT
from amr.transport.udp import UdpTransport, make_socket, safe_send

_enc = json.JSONEncoder(separators=(",", ":")).encode
RUN_DIR = ROOT / "runs" / "live"


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--id", type=int, required=True)
    ap.add_argument("--scenario", default="demo")
    ap.add_argument("--port-base", type=int, default=9100)
    ap.add_argument("--world", default="127.0.0.1:9000")
    ap.add_argument("--observers", default="127.0.0.1:9050,127.0.0.1:9051")
    ap.add_argument("--rules", default=str(RUN_DIR / "faults.json"))
    ap.add_argument("--alloc", default="cbba", choices=["cbba", "seq"])
    ap.add_argument("--hosts", default="", help="id=host,... for multi-machine runs")
    ap.add_argument("--t0", type=float, default=None, help="shared scenario start (epoch s)")
    args = ap.parse_args(argv)

    sc = load_scenario(args.scenario)
    cfg = DEFAULT.with_(**sc.config) if sc.config else DEFAULT
    ids = [r.rid for r in sc.robots]
    me = args.id
    RUN_DIR.mkdir(parents=True, exist_ok=True)
    (RUN_DIR / f"robot_{me}.pid").write_text(str(os.getpid()))
    hosts = dict((int(k), v) for k, v in (h.split("=") for h in args.hosts.split(",") if h))
    observers = []
    for o in args.observers.split(","):
        if o:
            h, p = o.split(":")
            observers.append((h, int(p)))
    tr = UdpTransport(me, ids, port_base=args.port_base, hosts=hosts, observers=observers,
                      rules_file=args.rules, seed=sc.seed)
    wh, wp = args.world.split(":")
    world_addr = (wh, int(wp))
    wsock = make_socket(0)
    agent = Agent(me, sc.map, cfg, fleet=ids, tasks=sc.tasks, alloc=args.alloc, seed=sc.seed)
    dt = 1.0 / cfg.control_hz
    pose = None
    battery = 100.0
    scan = None
    last_t = None
    print(f"[robot {me}] up, port {args.port_base + me}", flush=True)
    t_next = time.perf_counter()
    steps = 0
    status_t = 0.0
    safe_send(wsock, _enc({"t": "cmd", "id": me, "v": 0.0, "w": 0.0}).encode(), world_addr)
    t0 = args.t0 if args.t0 is not None else time.time()
    while True:
        now = time.time() - t0          # scenario clock shared by all processes
        # own sensors (private world channel): keep the newest pose, newest scan
        while True:
            try:
                data, _ = wsock.recvfrom(65536)
            except ConnectionResetError:
                continue      # Windows: ICMP 'port unreachable' from a dead peer; keep reading
            except (BlockingIOError, OSError):
                break
            m = json.loads(data)
            pose = tuple(m["p"])
            battery = m.get("bat", battery)
            if "scan" in m:
                scan = {"ranges": np.asarray(m["scan"], dtype=np.float32), "pose": pose, "t": now}
        if pose is None:
            safe_send(wsock, _enc({"t": "cmd", "id": me, "v": 0.0, "w": 0.0}).encode(), world_addr)
            time.sleep(0.05)
            continue
        tr.pos = (pose[0], pose[1])
        tr.tick(now)
        inbox = tr.poll(now)
        step_dt = dt if last_t is None else max(1e-3, now - last_t)
        last_t = now
        cmd, out = agent.step(now, step_dt, pose, scan, inbox, battery)
        scan = None
        safe_send(wsock, _enc({"t": "cmd", "id": me, "v": round(cmd[0], 4), "w": round(cmd[1], 4)}).encode(),
                  world_addr)
        if out:
            tr.send(out, now)
        steps += 1
        if now >= status_t:
            status_t = now + 5.0
            print(f"[robot {me}] steps={steps} mode={agent.mode} live={sorted(agent.live)} "
                  f"job={agent.job.kind if agent.job else None} bat={battery:.0f}", flush=True)
        t_next += dt
        sl = t_next - time.perf_counter()
        if sl > 0:
            time.sleep(sl)
        else:
            t_next = time.perf_counter()


if __name__ == "__main__":
    main()
