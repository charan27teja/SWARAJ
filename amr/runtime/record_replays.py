"""Record the dashboard replays used by the static demo site.

    python -m amr.runtime.record_replays            all 4 (demo, circular_wait, blockage, node_loss)
    python -m amr.runtime.record_replays demo       just one

Each run starts the real live system (5 robot processes + world) plus the passive bridge with
`--record`, replays the scenario's faults and saves the frames the bridge sends to browsers into
dashboard/public/replays/<id>.json.gz, then updates index.json.
"""
from __future__ import annotations

import json
import sys
import threading
import time

from amr.core.scenario import ROOT
from amr.runtime.launcher import Fleet, RUN_DIR, run_schedule, spawn

OUT = ROOT / "dashboard" / "public" / "replays"
RUNS = {
    "demo": (80, "5 robots on overlapping pick/drop traffic"),
    "circular_wait": (75, "4 robots forced into a cycle at the narrow cross; the lowest priority yields"),
    "blockage": (80, "a pallet is dropped in aisle A13; robots detect it with lidar and re-route"),
    "node_loss": (90, "R2 is killed inside an aisle: lease expiry, occupied-unknown, task re-auction"),
}


def record(sid: str) -> None:
    secs, _ = RUNS[sid]
    path = OUT / f"{sid}.json.gz"
    path.unlink(missing_ok=True)
    fleet = Fleet(sid, bridge=False).start()
    bridge = spawn(["amr.bridge.bridge", "--scenario", sid, "--no-http", "--record", str(path),
                    "--record-seconds", str(secs)], "bridge", RUN_DIR, True)
    stop = threading.Event()
    th = threading.Thread(target=run_schedule, args=(fleet,), kwargs={"duration": secs + 30, "stop": stop,
                                                                     "log": lambda m: print("  " + m, flush=True)},
                          daemon=True)
    th.start()
    end = time.time() + secs + 30
    while time.time() < end and not path.exists():
        time.sleep(0.5)
    time.sleep(1.0)
    stop.set()
    bridge.terminate()
    fleet.stop()
    print(f"{sid}: {path.stat().st_size // 1024 if path.exists() else 'MISSING'} kB", flush=True)


def write_index() -> None:
    items = [{"id": sid, "file": f"{sid}.json.gz", "title": sid.replace("_", " "), "desc": desc}
             for sid, (_, desc) in RUNS.items() if (OUT / f"{sid}.json.gz").exists()]
    (OUT / "index.json").write_text(json.dumps({"replays": items}, indent=1), encoding="utf-8")


def main(argv=None):
    ids = (argv if argv is not None else sys.argv[1:]) or list(RUNS)
    OUT.mkdir(parents=True, exist_ok=True)
    for sid in ids:
        record(sid)
    write_index()


if __name__ == "__main__":
    main()
