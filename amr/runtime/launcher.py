"""Start the live system: world, one process per robot, dashboard bridge, optional recorder.

    python -m amr.runtime.launcher --scenario demo
    python -m amr.runtime.launcher --scenario circular_wait --apply-faults --recorder

The launcher is a process supervisor only: it starts and stops processes and (with
--apply-faults) replays the scenario's fault schedule through the same `amr.faults` functions
an operator would use. It takes no part in coordination.
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
from pathlib import Path

from amr.core.scenario import load_scenario, ROOT
from amr import faults

RUN_DIR = ROOT / "runs" / "live"


def spawn(args: list[str], name: str, log_dir: Path, quiet: bool) -> subprocess.Popen:
    log = open(log_dir / f"{name}.log", "w", encoding="utf-8")
    flags = 0
    if os.name == "nt":
        flags = subprocess.CREATE_NEW_PROCESS_GROUP
    p = subprocess.Popen([sys.executable, "-u", "-m"] + args, cwd=str(ROOT), stdout=log,
                         stderr=subprocess.STDOUT, creationflags=flags,
                         env={**os.environ, "PYTHONPATH": str(ROOT)})
    return p


class Fleet:
    """Programmatic start/stop of the live system (used by the launcher CLI and live tests)."""

    def __init__(self, scenario: str, bridge: bool = True, recorder: bool = False, quiet: bool = True,
                 extra_robot_args: list[str] | None = None):
        self.scenario = scenario
        self.sc = load_scenario(scenario)
        self.bridge = bridge
        self.recorder = recorder
        self.quiet = quiet
        self.extra = extra_robot_args or []
        self.procs: dict[str, subprocess.Popen] = {}
        self.t0 = None

    def start(self) -> "Fleet":
        RUN_DIR.mkdir(parents=True, exist_ok=True)
        for f in RUN_DIR.glob("*.pid"):
            f.unlink()
        faults.write_rules({})
        self.t0 = time.time() + 1.5
        t0 = ["--t0", f"{self.t0:.3f}"]
        self.procs["world"] = spawn(["amr.runtime.live_world", "--scenario", self.scenario], "world", RUN_DIR,
                                    self.quiet)
        (RUN_DIR / "world.pid").write_text(str(self.procs["world"].pid))
        if self.bridge:
            self.start_bridge()
        if self.recorder:
            self.procs["recorder"] = spawn(["amr.bench.recorder", "--scenario", self.scenario], "recorder",
                                           RUN_DIR, self.quiet)
            (RUN_DIR / "recorder.pid").write_text(str(self.procs["recorder"].pid))
        for r in self.sc.robots:
            self.procs[f"robot_{r.rid}"] = spawn(["amr.runtime.live_robot", "--id", str(r.rid), "--scenario",
                                                  self.scenario] + t0 + self.extra, f"robot_{r.rid}", RUN_DIR,
                                                 self.quiet)
        return self

    def start_bridge(self) -> None:
        self.procs["bridge"] = spawn(["amr.bridge.bridge", "--scenario", self.scenario], "bridge", RUN_DIR,
                                     self.quiet)
        (RUN_DIR / "bridge.pid").write_text(str(self.procs["bridge"].pid))

    def alive(self, name: str) -> bool:
        p = self.procs.get(name)
        return p is not None and p.poll() is None

    def stop(self) -> None:
        for p in self.procs.values():
            if p.poll() is None:
                p.terminate()
        for p in self.procs.values():
            try:
                p.wait(timeout=3)
            except subprocess.TimeoutExpired:
                p.kill()
        for f in RUN_DIR.glob("*.pid"):
            f.unlink()


def _in_aisle(sc, rid: int) -> bool:
    """Operator's view of the floor (world truth via the control port)."""
    tr = faults.world_ctl({"op": "truth"})
    if not tr:
        return False
    b = tr.get("truth", {}).get("robots", {}).get(str(rid))
    if not b:
        return False
    return sc.map.section_of(sc.map.cell_of(b["x"], b["y"])) is not None


def run_schedule(fleet: Fleet, apply: bool = True, duration: float | None = None, log=print, stop=None) -> None:
    """Replay the scenario's fault/blockage schedule (relative to the shared start t0) through the
    same functions an operator would use. `kill ... when: in_aisle` waits until the robot is inside
    a narrow aisle; `remove_body` is timed relative to the actual kill."""
    sc = fleet.sc
    t0 = fleet.t0
    schedule = sorted(sc.faults + [dict(b, action="block") for b in sc.blockages], key=lambda e: e["t"])         if apply else []
    ends = []
    killed_at: dict[int, float] = {}
    while True:
        now = time.time() - t0
        for e in [e for e in schedule if e["t"] <= now]:
            act = e["action"]
            if act == "kill" and e.get("when") == "in_aisle" and not _in_aisle(sc, e["robot"]):
                continue                                   # wait until it is inside an aisle
            if act == "remove_body":
                k = killed_at.get(e["robot"])
                kill_t = next((f["t"] for f in sc.faults if f["action"] == "kill" and f["robot"] == e["robot"]), 0)
                if k is None or now < k + (e["t"] - kill_t):
                    continue
            schedule.remove(e)
            log(f"[launcher] t={now:.1f}s fault: {e}")
            if act == "kill":
                faults.apply("kill", robot=e["robot"])
                killed_at[e["robot"]] = now
            elif act == "remove_body":
                faults.apply("remove_body", robot=e["robot"])
            elif act == "block":
                faults.apply("block", cell=e["cell"])
                if e.get("until"):
                    ends.append((e["until"], "unblock", e))
            elif act in ("partition", "heal", "dead_zone", "loss", "delay"):
                kw = {k: v for k, v in e.items() if k not in ("t", "action", "until")}
                faults.apply(act, **kw)
                if e.get("until"):
                    ends.append((e["until"], "clear_" + act, e))
        for item in [x for x in ends if x[0] <= now]:
            ends.remove(item)
            if item[1] == "unblock":
                faults.apply("unblock")
            else:
                r = faults.read_rules()
                key = {"clear_dead_zone": "dead_zones", "clear_loss": "loss", "clear_delay": "delay_ms",
                       "clear_partition": "partition"}.get(item[1])
                r.pop(key, None)
                faults.write_rules(r)
        if (duration and now > duration) or (stop is not None and stop.is_set()):
            return
        time.sleep(0.1)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", default="demo")
    ap.add_argument("--no-bridge", action="store_true")
    ap.add_argument("--recorder", action="store_true")
    ap.add_argument("--apply-faults", action="store_true", help="replay the scenario's fault schedule")
    ap.add_argument("--duration", type=float, default=None, help="stop after N seconds")
    ap.add_argument("--quiet", action="store_true")
    a = ap.parse_args(argv)

    fleet = Fleet(a.scenario, bridge=not a.no_bridge, recorder=a.recorder, quiet=a.quiet).start()
    sc = fleet.sc
    print(f"[launcher] scenario '{sc.name}': world + {len(sc.robots)} robots"
          + ("" if a.no_bridge else " + bridge (dashboard http://localhost:8080)")
          + (" + recorder" if a.recorder else ""), flush=True)
    print("[launcher] logs in runs/live/*.log  |  faults: python -m amr.faults ...  |  Ctrl+C to stop", flush=True)
    try:
        run_schedule(fleet, apply=a.apply_faults, duration=a.duration)
    except KeyboardInterrupt:
        pass
    finally:
        print("[launcher] stopping...", flush=True)
        fleet.stop()


if __name__ == "__main__":
    main()
