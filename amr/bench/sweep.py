"""Benchmark sweep in the fast runtime: our system vs baseline B0 on identical seeds.

    python -m amr.bench.sweep                       the project benchmark: 5 robots, congestion
                                                    medium/high, faults none/node_loss, 5 seeds
    python -m amr.bench.sweep --quick               smoke run (2 seeds)

Results are appended to runs/bench/results.csv as runs finish; re-running resumes (rows already
present are skipped), so the sweep can be interrupted safely.
"""
from __future__ import annotations

import argparse
import csv
import itertools
import os
import sys
import time
from multiprocessing import Pool
from pathlib import Path

from amr.core.scenario import random_overlap, ROOT

FIELDS = ["system", "robots", "congestion", "fault", "seed", "all_done", "completed", "n_tasks", "makespan",
          "sim_time", "throughput_per_h", "collisions", "wall_contacts", "obstacle_contacts", "min_robot_dist",
          "section_violations", "deadlocks", "deadlocks_resolved", "unresolved_deadlocks", "mean_ttr_s", "max_ttr_s",
          "mean_wait_s", "lock_requests", "duplicates", "bytes_per_robot_s", "msgs_per_robot_s", "beacon_bytes_mean",
          "yields", "blockage_events", "reauctions", "recovery_s", "agent_us_per_step", "wall_s"]
CAP_S = 900.0


def run_one(args):
    system, n, cong, fault, seed = args
    from amr.runtime.fast import FastSim
    t0 = time.time()
    try:
        sc = random_overlap(n, cong, seed=seed, fault=fault, duration=CAP_S)
        r = FastSim(sc, system=system).run()
    except Exception as e:                      # a crash is a result too: record it, never hide it
        import traceback
        traceback.print_exc()
        r = {"all_done": False, "completed": 0, "n_tasks": -1, "error": repr(e)}
    r.update({"system": system, "robots": n, "congestion": cong, "fault": fault, "seed": seed,
              "wall_s": round(time.time() - t0, 1)})
    return r


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--sizes", default="5")
    ap.add_argument("--congestion", default="medium,high")
    ap.add_argument("--faults", default="none,node_loss")
    ap.add_argument("--seeds", type=int, default=5)
    ap.add_argument("--seed-offset", type=int, default=1)
    ap.add_argument("--systems", default="ours,b0")
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    ap.add_argument("--out", default=str(ROOT / "runs" / "bench" / "results.csv"))
    ap.add_argument("--quick", action="store_true", help="2 seeds only")
    a = ap.parse_args(argv)
    if a.quick:
        a.seeds = 2
        a.out = str(ROOT / "runs" / "bench" / "quick.csv")
    out = Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    done = set()
    if out.exists():
        with out.open(newline="", encoding="utf-8") as f:
            for row in csv.DictReader(f):
                done.add((row["system"], int(row["robots"]), row["congestion"], row["fault"], int(row["seed"])))
    sizes = [int(x) for x in a.sizes.split(",")]
    seeds = range(a.seed_offset, a.seed_offset + a.seeds)
    jobs = []
    # smallest fleets first, seeds interleaved across cells so partial sweeps are balanced
    for n in sizes:
        for seed, cong, fault, system in itertools.product(seeds, a.congestion.split(","), a.faults.split(","),
                                                           a.systems.split(",")):
            key = (system, n, cong, fault, seed)
            if key not in done:
                jobs.append(key)
    print(f"[sweep] {len(jobs)} runs to do ({len(done)} already in {out.name}), {a.workers} workers", flush=True)
    new = not out.exists()
    t0 = time.time()
    with out.open("a", newline="", encoding="utf-8") as f, Pool(a.workers, maxtasksperchild=20) as pool:
        w = csv.DictWriter(f, fieldnames=FIELDS + ["error"], extrasaction="ignore")
        if new:
            w.writeheader()
        for i, r in enumerate(pool.imap_unordered(run_one, jobs), 1):
            w.writerow(r)
            f.flush()
            if i % 10 == 0 or i == len(jobs):
                el = time.time() - t0
                print(f"[sweep] {i}/{len(jobs)} runs, {el/60:.1f} min elapsed, ~{el/i*(len(jobs)-i)/60:.0f} min left",
                      flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
