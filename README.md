# SWARAJ — Decentralised AMR Fleet Coordination

**Smart India Hackathon 2026** · Problem Statement SIH26123 (Bharat Electronics Limited)

A prototype decentralised coordination and collision-avoidance framework for autonomous mobile robots (AMRs) in warehouse environments. SWARAJ enables **5 autonomous mobile robots** to coordinate peer-to-peer with zero central authority, achieving safe and efficient task execution through distributed consensus algorithms.

## Key Features

- **Peer-to-Peer Coordination**: Each robot runs identical agent code; no central server, coordinator, or global planner
- **Zero Collisions**: 0 robot–robot collisions in all 40 benchmark runs, using ORCA reciprocal avoidance plus an obstacle-scan safety stop
- **Deadlock Resolution**: Distributed cycle detection and priority-based yield strategy using wait-for graphs
- **Battery-Aware Task Allocation**: CBBA-based auction with feasibility constraints and orphan re-auctioning
- **Adaptive Path Planning**: Space-time reservations (SIPP) with dynamic blockage events and D* Lite replanning
- **Monitoring Dashboard**: Real-time monitoring with live positions, lock states, alerts, and performance metrics
- **Less Waiting at Choke Points**: 1.0–2.0 s mean wait at narrow aisles vs 3.7–5.0 s for centralised stop-and-wait
- **Fault Resilience**: Network partitions, node failures, and dead zones handled gracefully

## Architecture

| Layer | Job | Method (file) |
|---|---|---|
| L5 Observability | dashboard, zero authority | passive UDP→WebSocket bridge (`amr/bridge`) + React/TS/Vite/Tailwind (`dashboard/`) |
| L4 Task | decentralised, battery-feasible allocation | CBBA with time-discounted reward, congestion frozen per round, battery + capacity feasibility, orphan re-auction; sequential auction variant (`amr/agent/tasks.py`) |
| L3 Route | space-time reservations, re-planning | reservation-table A* → **SIPP** over peers' broadcast intents; **D\* Lite** incremental repair on BlockageEvents with TTL (`amr/agent/planner.py`) |
| L2 Contention | narrow aisles as mutex, deadlocks | **Ricart-Agrawala** + Lamport clocks, live-peer membership (epochs), leased grants, **sensor-confirmed entry**, degraded mode; wait-for graph (lock + physical edges) cycle detection, ageing priority, yield/back-off (`amr/agent/locks.py`, `deadlock.py`) |
| L1 Motion | reciprocal avoidance, 20 Hz | **ORCA** (RVO2 LP1/2/3) + wall / virtual-wall half-planes, speed capped along the actual heading (`amr/agent/orca.py`) |
| L1s Safety | network-independent last line | simulated 2D lidar, velocity-scaled protective stop; unexplained returns assumed to be approaching (`amr/agent/safety.py`) |
| L0 Transport | P2P, QoS split | UDP: 10 Hz best-effort `StateBeacon`, reliable `CoordEvent` (seq/ACK/retransmit/dedupe), fault shim (`amr/transport`) |

Processes in the live runtime (all native Windows, no WSL/Docker):

```
 robot 1 ─┐   UDP peer-to-peer (beacons, lock/task/blockage events)   ┌─ robot N
          ├─────────────────────────────────────────────────────────────┤
          │ private channel: pose+lidar ⇄ velocity                      │
          └──────────────► world (physics stand-in) ◄───────────────────┘
 copies of robot messages (fire-and-forget) ──► bridge (passive) ──► dashboard (browser)
                                            └─► metrics recorder (passive)
```

The same pure agent (`agent.step(now, dt, pose, scan, inbox) -> (cmd, outbox)`) also runs in a
**fast lockstep runtime** (`amr/runtime/fast.py`) with an in-memory bus that emulates latency, loss,
partitions and dead zones — used for tests and the benchmark.

The map (`maps/warehouse_a.yaml`, 30 m × 25 m) has 3 m main lanes, one-way single-width picking
aisles and a narrow bidirectional 4-way cross. Critical sections, passing bays and aisle mouths are
**extracted automatically** from the grid.

## Install (Windows `cmd`)

Requires Python 3.14 and Node.js 24.

```bat
cd amr-fleet
python -m venv .venv
.venv\Scripts\pip install -r requirements.txt
cd dashboard
npm install
npm run build
npx playwright install chromium   & rem only needed for the UI tests
cd ..
```

## Run

```bat
scripts\run_demo.cmd                 & rem full guided judges' demo (spec §11, steps 1-7), ~8 min
scripts\run_demo.cmd --fast          & rem same, shorter steps
scripts\run_demo.cmd --steps 2,4     & rem selected steps
scripts\run_demo.cmd demo            & rem one scenario, runs until Ctrl+C
```

The dashboard opens at **http://localhost:8080**. Scenarios (all 5 robots, `scenarios/*.yaml`):
`demo` (random_overlap), `head_on`, `circular_wait`, `blockage`, `node_loss`, `dead_zone`, `partition`.

Inject faults by hand while a scenario runs (the dashboard only *displays* active rules):

```bat
python -m amr.faults partition 1,2,3 4,5     & rem split the 5-robot fleet into two islands
python -m amr.faults heal
python -m amr.faults loss 0.2 --robot 3
python -m amr.faults delay 80 --jitter 40
python -m amr.faults deadzone 10 12 20 15
python -m amr.faults kill 2                  & rem also: kill bridge | recorder | world
python -m amr.faults block 13 17             & rem drop a pallet on cell (13,17)
python -m amr.faults remove 2                & rem operator removes a dead robot's body
python -m amr.faults clear
```

(Run them from the `amr-fleet` folder with `set PYTHONPATH=%CD%` and the venv's python, or via
`.venv\Scripts\python -m amr.faults ...`.)

## Demo steps (spec §11) — what `scripts\run_demo.cmd` does

Each step starts the live system for a scenario, replays its faults, tells you what to look at,
and then **checks from the passive recorder** that it really happened (PASS/FAIL per step).

1. **5 robots on random overlapping traffic** — live map, paths, lock tags, KPIs.
2. **circular_wait** — 5 robots; 4 of them enter the narrow cross from all arms; the cycle appears
   in red in the wait-for view; the lowest-priority robot yields to a passing bay.
3. **blockage** — a pallet is dropped in aisle A13; a robot's own lidar publishes a BlockageEvent;
   robots re-route / re-bid with no central input.
4. **node_loss** — R2 is killed inside an aisle: stale (grey, inflated), lease expiry → aisle
   *occupied-unknown*, task re-auctioned, body removed, lidar-confirmed clearance.
5. **partition** — two network islands ({1,2,3} and {4,5}) for 30 s, then heal; degraded mode keeps islands out of each
   other's aisles; membership and claims reconcile.
6. **dashboard killed and restarted** — banner "Bridge disconnected — fleet unaffected"; robot
   processes keep running; the console reconnects.
7. **benchmark report** — opens `docs/benchmark/report.html`.

## Tests

```bat
scripts\run_tests.cmd
```

runs `pytest` (unit tests for every layer, fast-runtime scenario tests asserting the phase
criteria, live multi-process tests over real UDP, architecture-purity checks) and the Playwright
UI smoke tests (loads & connects, robots appear, deadlock alert appears, bridge killed → banner →
recovers with robots unaffected).

## Benchmark vs baseline B0

B0 (`amr/baseline/b0.py`) is a **centralised stop-and-wait** planner implemented fairly: same map
and traffic rules, speeds, lidar safety, dwell times, battery model, admission control and seeds.
It differs only in coordination: FIFO task dispatch, FIFO cell/aisle zone reservations (robots halt
until their next zone is granted), a central deadlock supervisor, and robots that stop/creep when
they lose the server.

```bat
scripts\run_bench.cmd             & rem 5 robots x congestion medium,high x faults none,node_loss x 5 seeds, then the report
scripts\run_bench.cmd --quick     & rem 2 seeds only
```

Report: [`docs/benchmark/report.md`](docs/benchmark/report.md) (also `.html`, CSV, charts).

**Results (40 runs: 5 robots, medium/high congestion, no fault and node loss, 5 seeds, both
systems on identical seeds):**

| congestion | fault | ours makespan (s) | B0 makespan (s) | reduction vs B0 (95 % CI) |
|---|---|---:|---:|---:|
| medium | none | 195.8 ± 10.4 | 201.7 ± 22.3 | +2.6 ± 6.7 % |
| medium | node_loss | 246.0 ± 14.9 | 236.6 ± 25.5 | −4.3 ± 5.5 % |
| high | none | 242.0 ± 38.4 | 252.5 ± 43.2 | +3.6 ± 13.5 % |
| high | node_loss | 299.7 ± 55.7 | 301.7 ± 45.6 | +0.8 ± 10.0 % |

- **Safety gates met for both systems:** 0 robot–robot collisions, 0 unresolved deadlocks, never
  two robots in one aisle; 40/40 runs completed.
- **The ≥ 20 % makespan-reduction target is not met** in any cell: the two systems are within a
  few percent of each other and every confidence interval includes zero. Our fleet waits less at
  narrow aisles (1.0–2.0 s vs 3.7–5.0 s mean) but that does not shorten the makespan against a
  well-implemented central planner that knows everything.
- Duplicate task executions: 1 (ours) and 3 (B0) in the node-loss runs.

## Baseline B1: stop-and-wait without deadlock supervisor

B1 (`amr/baseline/b1.py`) is a simplified version of B0 that removes the active deadlock supervisor: robots still use FIFO zone reservations and stop-and-wait, but when a circular wait forms, no central process detects and breaks it. Result: **robots deadlock indefinitely**.

Quick benchmark (2 seeds): B1 hits unresolved deadlocks in all 4 scenarios (medium/high × none/node_loss), timing out at 900s without completing tasks. In contrast, B0 and our system complete all tasks in ~2–6 min. This demonstrates that **deadlock detection and breaking is essential** for central planners operating in confined spaces, even without the overhead of distributed coordination.

## Deploy the demo

Judges can watch **recorded runs of the real system** in the same dashboard from a public link.
The site opens in **Demo mode by default**, showing recorded 5-robot runs (`demo`, `circular_wait`,
`blockage`, `node_loss`, ~75–90 s each, ~100 kB each). These are recorded by the passive bridge
(`--record`) into `dashboard/public/replays/`. The badge shows **PEER-TO-PEER** (demo mode) or **LIVE**
(when connected to the real bridge). Modes are switchable anytime from the Simulation options.

```bat
cd dashboard
npm run build
npx vite preview                 & rem http://localhost:4173  (no Python needed)
```

Deploy to Vercel: import the repo with **Root Directory = `dashboard`** (settings come from
`dashboard/vercel.json`), or from `dashboard\` run `npx vercel --prod`. Any static host works:
upload `dashboard/dist`.

Re-record the runs (starts the live system; ~6 min): `python -m amr.runtime.record_replays`.
