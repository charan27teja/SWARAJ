"""Fault-injection CLI for the live runtime (test tooling; the Windows substitute for `tc netem`).

Network rules are written to runs/live/faults.json, which every robot's transport polls; robot
logic never reads it. Process kills use the pid files the processes write. Pallets and body
removal go to the world's control port (the world is physics, not a participant).

    python -m amr.faults partition 1,2,3 4,5        split the fleet into islands
    python -m amr.faults heal                       remove the partition
    python -m amr.faults loss 0.2 [--robot 3]       packet loss (whole fleet or one robot)
    python -m amr.faults delay 80 [--jitter 40]     one-way latency in ms
    python -m amr.faults deadzone 10 12 20 15       robots inside the rectangle lose their link
    python -m amr.faults clear                      remove all network rules
    python -m amr.faults kill 2 | bridge | recorder | world
    python -m amr.faults block 13 17                drop a pallet on cell (13,17)
    python -m amr.faults unblock [id]               remove pallet(s)
    python -m amr.faults remove 2                   operator removes robot 2's dead body
    python -m amr.faults status                     show active rules
"""
from __future__ import annotations

import argparse
import json
import os
import signal
import socket
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RUN_DIR = ROOT / "runs" / "live"
RULES = RUN_DIR / "faults.json"


def read_rules() -> dict:
    try:
        return json.loads(RULES.read_text(encoding="utf-8") or "{}")
    except (OSError, ValueError):
        return {}


def write_rules(r: dict) -> None:
    RUN_DIR.mkdir(parents=True, exist_ok=True)
    tmp = RULES.with_suffix(".tmp")
    tmp.write_text(json.dumps(r, indent=1), encoding="utf-8")
    os.replace(tmp, RULES)          # atomic: robots never read a half-written file


def world_ctl(msg: dict, port: int = 9001, timeout: float = 1.0) -> dict | None:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(timeout)
    try:
        s.sendto(json.dumps(msg).encode(), ("127.0.0.1", port))
        data, _ = s.recvfrom(65536)
        return json.loads(data)
    except OSError:
        return None
    finally:
        s.close()


def kill_proc(name: str) -> bool:
    f = RUN_DIR / (f"robot_{name}.pid" if name.isdigit() else f"{name}.pid")
    try:
        pid = int(f.read_text())
    except (OSError, ValueError):
        print(f"no pid file for {name}")
        return False
    try:
        os.kill(pid, signal.SIGTERM)
        print(f"killed {name} (pid {pid})")
        return True
    except OSError as e:
        print(f"could not kill {name}: {e}")
        return False
    finally:
        try:
            f.unlink()
        except OSError:
            pass


def apply(action: str, **kw) -> None:
    """Apply one fault. Shared by the CLI and the launcher's scenario schedule."""
    r = read_rules()
    if action == "partition":
        r["partition"] = kw["groups"]
    elif action == "heal":
        r.pop("partition", None)
    elif action == "loss":
        if kw.get("robot") is not None:
            r.setdefault("loss_by_robot", {})[str(kw["robot"])] = kw["p"]
        else:
            r["loss"] = kw["p"]
    elif action == "delay":
        r["delay_ms"] = kw["ms"]
        r["jitter_ms"] = kw.get("jitter_ms", 0)
    elif action == "dead_zone":
        r.setdefault("dead_zones", []).append(list(kw["rect"]))
    elif action == "clear":
        r = {}
    elif action == "kill":
        kill_proc(str(kw["robot"]))
        return
    elif action == "block":
        c = kw["cell"]
        print(world_ctl({"op": "block", "x": c[0] + 0.5, "y": c[1] + 0.5}))
        return
    elif action == "unblock":
        print(world_ctl({"op": "unblock", "id": kw.get("id")}))
        return
    elif action == "remove_body":
        print(world_ctl({"op": "remove_body", "robot": int(kw["robot"])}))
        return
    else:
        raise ValueError(action)
    write_rules(r)


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python -m amr.faults", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd")
    ap.add_argument("args", nargs="*")
    ap.add_argument("--robot", type=int)
    ap.add_argument("--jitter", type=float, default=0.0)
    a = ap.parse_args(argv)
    c, x = a.cmd, a.args
    if c == "partition":
        apply("partition", groups=[[int(i) for i in g.split(",")] for g in x])
    elif c == "heal":
        apply("heal")
    elif c == "loss":
        apply("loss", p=float(x[0]), robot=a.robot)
    elif c == "delay":
        apply("delay", ms=float(x[0]), jitter_ms=a.jitter)
    elif c == "deadzone":
        apply("dead_zone", rect=[float(v) for v in x[:4]])
    elif c == "clear":
        apply("clear")
    elif c == "kill":
        kill_proc(x[0])
    elif c == "block":
        apply("block", cell=(int(x[0]), int(x[1])))
    elif c == "unblock":
        apply("unblock", id=int(x[0]) if x else None)
    elif c == "remove":
        apply("remove_body", robot=int(x[0]))
    elif c == "status":
        print(json.dumps(read_rules(), indent=1) or "{}")
    else:
        ap.print_help()
        sys.exit(2)
    if c not in ("status", "kill", "block", "unblock", "remove"):
        print("rules:", json.dumps(read_rules()))


if __name__ == "__main__":
    main()
