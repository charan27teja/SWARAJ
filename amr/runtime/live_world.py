"""Live world process (physics stand-in, replaced by Gazebo in P8).

Private per-robot channel on UDP `world_port`: a robot sends {"t":"cmd","id","v","w"}; the world
replies *only to that robot* with its own pose, battery and (10 Hz) lidar scan. The world never
forwards anything between robots and never makes a coordination decision.

A separate control port accepts test-harness / operator commands (drop or remove a pallet,
remove a dead robot's body). Ground truth (collisions, true poses, pallets) is streamed to the
observer ports for the dashboard and metrics recorder.

Motor watchdog: a robot whose process stops sending commands for 0.5 s brakes to a stop (as a
real motor controller would); its body stays on the floor until an operator removes it.
"""
from __future__ import annotations

import argparse
import json
import math
import time

from amr.core.config import DEFAULT
from amr.core.scenario import load_scenario
from amr.transport.udp import make_socket, safe_send
from amr.world.world import World

_enc = json.JSONEncoder(separators=(",", ":")).encode


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", default="demo")
    ap.add_argument("--world-port", type=int, default=9000)
    ap.add_argument("--control-port", type=int, default=9001)
    ap.add_argument("--observers", default="127.0.0.1:9050,127.0.0.1:9051")
    ap.add_argument("--seed", type=int, default=None)
    args = ap.parse_args(argv)

    sc = load_scenario(args.scenario)
    cfg = DEFAULT.with_(**sc.config) if sc.config else DEFAULT
    world = World(sc.map, cfg, seed=sc.seed if args.seed is None else args.seed)
    for r in sc.robots:
        world.add_robot(r.rid, r.cell[0] + 0.5, r.cell[1] + 0.5, r.theta, r.battery)
    sock = make_socket(args.world_port)
    ctl = make_socket(args.control_port)
    observers = []
    for o in args.observers.split(","):
        if o:
            h, p = o.split(":")
            observers.append((h, int(p)))
    addr: dict[int, tuple] = {}
    last_scans: dict = {}
    last_cmd: dict[int, float] = {}
    dt = 1.0 / cfg.control_hz
    lidar_every = max(1, round(cfg.control_hz / cfg.lidar_hz))
    step = 0
    t_next = time.perf_counter()
    truth_next = 0.0
    print(f"[world] scenario={sc.name} robots={len(sc.robots)} port={args.world_port}", flush=True)
    while True:
        # --- commands from robots (private channel)
        while True:
            try:
                data, a = sock.recvfrom(4096)
            except ConnectionResetError:
                continue      # Windows: ICMP 'port unreachable' from a dead peer; keep reading
            except (BlockingIOError, OSError):
                break
            try:
                m = json.loads(data)
            except ValueError:
                continue
            rid = m.get("id")
            if rid in world.bodies:
                addr[rid] = a
                last_cmd[rid] = time.time()
                b = world.bodies[rid]
                if b.present:
                    b.powered = True
                world.set_cmd(rid, float(m.get("v", 0.0)), float(m.get("w", 0.0)))
        # --- operator / harness control
        while True:
            try:
                data, a = ctl.recvfrom(4096)
            except ConnectionResetError:
                continue      # Windows: ICMP 'port unreachable' from a dead peer; keep reading
            except (BlockingIOError, OSError):
                break
            try:
                m = json.loads(data)
            except ValueError:
                continue
            op = m.get("op")
            reply = {"ok": True}
            if op == "block":
                reply["id"] = world.add_blockage(float(m["x"]), float(m["y"]), float(m.get("half", 0.4)))
            elif op == "unblock":
                bid = m.get("id")
                for b in list(world.blockages) if bid is None else [int(bid)]:
                    world.remove_blockage(b)
            elif op == "remove_body":
                world.remove_body(int(m["robot"]))
            elif op == "truth":
                reply["truth"] = world.truth()
            safe_send(ctl, _enc(reply).encode(), a)
        # --- motor watchdog
        now = time.time()
        for rid, b in world.bodies.items():
            t = last_cmd.get(rid)
            if t is not None and now - t > 0.5:
                b.v_cmd = b.w_cmd = 0.0
        # --- physics
        world.step(dt)
        step += 1
        scans = world.scans(list(addr)) if step % lidar_every == 0 else {}
        if scans:
            last_scans.update(scans)
        for rid, a in addr.items():
            b = world.bodies[rid]
            msg = {"t": "obs", "id": rid, "p": [b.x, b.y, b.th], "bat": round(b.battery, 3), "wt": now}
            if rid in scans:
                msg["scan"] = [round(float(x), 3) for x in scans[rid]]
            safe_send(sock, _enc(msg).encode(), a)
        if now >= truth_next:
            truth_next = now + 0.2
            tr = world.truth()
            tr["type"] = "truth"
            # observer-only: every 3rd lidar ray for the dashboard's "lidar rays" overlay
            tr["lidar"] = {str(r): [round(float(x), 2) for x in v[::3]] for r, v in last_scans.items()
                           if world.bodies[r].present}
            data = _enc({"t": "W", "b": tr}).encode()
            for o in observers:
                safe_send(sock, data, o)
        # --- pacing
        t_next += dt
        sl = t_next - time.perf_counter()
        if sl > 0:
            time.sleep(sl)
        else:
            t_next = time.perf_counter()


if __name__ == "__main__":
    main()
