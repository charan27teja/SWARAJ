# SIH26123 — Decentralised AMR Fleet Coordination: Build Spec

You are building the software prototype for **Smart India Hackathon 2026, Problem Statement SIH26123 (Bharat Electronics Limited, Software category, Robotics & Drones theme)**. This file is the single source of truth. Read it fully before writing code, and re-read the relevant section at the start of each phase.

**Working rules: autonomous build loop**

Build the whole project **without stopping for my go-ahead**. Run this loop until every phase P0–P7 in Section 9 is done and the final Definition of Done (Section 12) is met:

1. **Plan:** read `PROGRESS.md`, pick the next unfinished phase, and write its task list there.
2. **Build:** implement it.
3. **Verify:** run the tests and the phase's demo or scenario checks. For the dashboard, also run the UI checks in Section 6.
4. **Fix:** if anything fails or a "Done means" criterion is not met, fix it and go back to step 3. Don't move on with failing tests.
5. **Record:** update `PROGRESS.md` with what was done, the evidence (test counts, key metrics) and known issues. Then continue with the next phase.

More rules:
- **`PROGRESS.md` is your memory.** Keep it current so that if the session is interrupted or the context is compacted, you can resume exactly where you left off by reading it.
- If the same failure survives 3 genuinely different fix attempts, log it in `PROGRESS.md` as a known issue with your diagnosis. Take the simplest honest workaround consistent with this spec and continue; never block the whole build on one item.
- Stop and ask me **only** for things you can't do yourself: anything needing administrator rights, a restart, a password or an account, or a decision that contradicts this spec.
- Never fake a result. Don't weaken a test to make it pass, and don't hard-code metrics. If a target isn't met, say so plainly with the output, both in `PROGRESS.md` and in your final report.
- When everything is done, give a final report: what was built, how to run it, test results, benchmark numbers vs the targets, and any known issues.
- The development machine is **Windows 11, 8 GB RAM, 8 threads, Intel integrated GPU, Python 3.14, Node.js 24**. No WSL, no Docker, no ROS 2 yet. Everything in Phases 0–7 must run natively on Windows. Give run commands for Windows `cmd`.
- Keep dependencies few and pip/npm-installable on Windows. Pin versions in `requirements.txt` / `package.json`.
- Write a `README.md` that always states exactly how to install and run the current state.

---

## 1. What the problem statement requires

Build a **decentralised coordination and collision-avoidance framework for a fleet of 5 AMRs** (autonomous mobile robots) in a dynamic warehouse, able to run on **edge hardware (Raspberry Pi / Jetson Nano class)**, with:

1. **Peer-to-peer communication** of position and intent — no central server.
2. **Real-time deadlock and collision resolution at choke points** (narrow aisles, intersections).
3. **Autonomous task re-allocation and re-routing** when an aisle is blocked or a robot fails.
4. A **monitoring dashboard** showing live positions, planned paths, battery state and alerts.
5. Delivered as a **multi-robot simulation**.

**Success criteria (numeric):**
- **Zero inter-robot collisions.**
- **At least 20% reduction in total task completion time (makespan)** versus a traditional **stop-and-wait** approach, on overlapping paths.

**Motivating failures to survive:** network latency, Wi-Fi dead zones, single point of failure.

---

## 2. The one architectural rule (enforced by tests)

**No hidden centralisation.** Every robot runs the identical agent code, holding only its own state plus what it hears from peers. There is no coordinator, no shared database, no global planner, no auctioneer.

- The **dashboard is read-only**: it listens to robot broadcasts and never sends anything to robots. Killing it must have zero effect on the fleet.
- The **world process** (Section 4) is a stand-in for physics/Gazebo only. It gives each robot its own true pose and simulated lidar. It **never** makes a coordination decision and never relays robot-to-robot messages.
- The **fault harness** and **metrics recorder** are test instruments, not participants. Robots must not read from them.
- A standing test (Phase 6) kills any single process — any robot, the dashboard, the metrics recorder — and the remaining fleet must keep working.

---

## 3. Solution design (layers)

Each robot runs these layers. Established, citable algorithms only.

| Layer | Job | Method |
|---|---|---|
| L5 Observability | Dashboard, zero authority | Python WebSocket bridge + React/TypeScript (Vite) |
| L4 Task | Decentralised task allocation, battery-feasible | CBBA (Consensus-Based Bundle Algorithm); start with a simpler sequential auction with deterministic tie-break, upgrade to CBBA |
| L3 Route | Space-time reservations; replanning on blockage | A* with reservation table first, then SIPP (Safe Interval Path Planning); D* Lite for static-map repair |
| L2 Contention | Narrow aisles as mutual-exclusion resources; deadlock detect-and-break | Ricart-Agrawala with Lamport clocks, **live-peer membership**, **leased grants**, **sensor-confirmed entry**; wait-for-graph cycle detection |
| L1 Motion | Reciprocal collision avoidance at 20 Hz | ORCA (implement in numpy following the RVO2 formulation) |
| L1s Safety | Network-independent last line of defence | Simulated 2D lidar + velocity-scaled protective stop |
| L0 Transport | P2P messaging, QoS-split | UDP between robot processes (Section 5) |

### Key rules per layer
- **Beacons:** each robot broadcasts a `StateBeacon` at **10 Hz**, best-effort. A peer not heard for **500 ms** becomes an *unknown obstacle* with an inflated safety radius. **Loss of information must always make a robot more conservative, never less.**
- **Aisle locks (L2):**
  - Critical sections = corridor segments or intersections narrower than 2 robot diameters, extracted automatically from the map.
  - To enter, a robot needs a reply from every **live** peer (heard within 500 ms); membership changes bump an epoch number carried in beacons.
  - A granted lock is a **lease** (expected traversal time + margin) renewed via the holder's beacon. If the holder dies, the lease expires.
  - An expired lease does **not** mean an empty aisle: the segment stays `occupied-unknown` until an approaching robot's own lidar confirms it clear. Entry always requires peer replies **and** a sensor-clear check — so two network islands can never occupy the same aisle.
- **Deadlock (L2):** each robot puts its current wait-for edge in its beacon; every robot builds the wait-for graph locally and runs cycle detection. The **lowest-priority** member of a cycle releases its claim, backs off to the nearest passing bay, and replans. Priority = base priority + `alpha * t_wait` (ageing), tie-break by robot id. The same priority function is used for L2 victim choice and L3 planning order. Back-off uses randomised jitter. Views may briefly disagree; each robot decides only for itself, so the worst case is an extra (safe) yield.
- **Blockage (L3):** a robot that detects a blocked aisle publishes `BlockageEvent(segment, timestamp, confidence, ttl)`; peers add it to a local cost overlay; it expires by TTL — nobody "clears" it centrally.
- **Tasks (L4):** bid score = time-discounted task reward (keeps CBBA's diminishing-marginal-gains property). Congestion enters as travel-time cost frozen at the start of each auction round. Battery is a **hard feasibility filter**: never bid on a task you cannot finish and still reach a dock with reserve; below reserve, claim a charging dock. A peer timing out triggers re-auction of only its tasks.
- **Time:** ordering uses Lamport clocks only. In this single-machine prototype wall clocks agree; still pad reservation windows by a configurable clock-error margin so the logic is honest for real deployment.

---

## 4. Simulation architecture (Phases 0–7, no Gazebo)

Two runtimes sharing **the same agent code**:

1. **Live runtime (demo):** one OS process per robot + one world process + dashboard bridge + optional metrics recorder, all real-time. Robots talk to each other only over UDP.
2. **Fast runtime (benchmark):** all agents stepped in lockstep in a single process with an in-memory message bus that emulates latency/loss/partitions, running faster than real time. Used for the 30-seed benchmark sweeps.

To make this possible, **agent logic must be pure and runtime-agnostic**: e.g. `agent.step(now, dt, own_pose, lidar_scan, inbox) -> (velocity_cmd, outbox)`. No sockets, threads or `time.time()` inside agent logic.

**World process (physics stand-in):** holds true robot poses, integrates differential-drive kinematics from each robot's velocity command, simulates a 2D lidar per robot (ray-cast against walls, racks and other robots), detects and logs collisions (ground truth for metrics), and injects blockages (e.g. a pallet dropped in an aisle). Robot ↔ world is a private per-robot channel (pose + scan out, velocity command in). It never forwards data between robots.

**Robots:** differential drive, radius 0.3 m, max speed 1.0 m/s, max angular 1.5 rad/s, battery model (drain per metre + idle drain; charges at docks).

**Map:** a warehouse grid (about 30 m × 20 m) defined in a YAML/JSON file: walls, rack rows, **single-width aisles (choke points)**, wider main lanes, **passing bays**, pick/drop stations, charging docks. Include at least one 4-way intersection that can produce a circular wait.

**Scenarios** (seeded, reproducible):
- `head_on` — 5 robots; 2 of them enter the same narrow aisle from opposite ends.
- `circular_wait` — 5 robots; 4 of them forced into a cycle at an intersection.
- `blockage` — an aisle becomes blocked mid-run.
- `node_loss` — a robot process is killed mid-task, including while inside a narrow aisle.
- `dead_zone` — a robot's network link drops while it is in a defined map region.
- `partition` — the fleet splits into two islands, then heals.
- `random_overlap --robots 5 --congestion low|medium|high --seed S` — general benchmark scenario.

**Fleet size: every scenario, the demo and the benchmark use exactly 5 robots.**

---

## 5. Transport and messages

- Each robot process binds a UDP port; peers are discovered from a port range on `127.0.0.1` (configurable host list so it later works across real machines). No broker, no relay process.
- Two message classes:
  - `StateBeacon` — 10 Hz, best-effort, newest-wins: `robot_id, seq, lamport, epoch, pose(x,y,theta), velocity, intent (next waypoints), planned_path (short horizon), held_lock + lease_expiry, wait_for (robot_id or null), battery, current_task, mode`.
  - `CoordEvent` — reliable (sequence numbers, ACK, retransmit, dedupe): lock `REQUEST/REPLY/RELEASE`, task `BID/CLAIM`, `BlockageEvent`.
- Serialisation: JSON (readable) or msgpack; keep beacons small (target ~200 bytes) and log the measured bytes/robot/second.
- **Fault shim** inside the transport: drop / delay / jitter / partition rules read from a fault-rules file controlled by `python -m amr.faults ...`. This is the Windows substitute for Linux `tc netem`. The shim is test tooling; robot logic never reads it.
- The dashboard bridge is a **passive listener** that receives beacons/events (robots send a copy to the bridge's port if present; if the bridge is absent, nothing changes and nothing blocks).

---

## 6. Monitoring dashboard (L5)

Python bridge (`websockets`) → React + TypeScript (Vite) single page. **Read-only.** This is what the judges see, so **UI/UX quality is a first-class requirement, not polish at the end.**

### UI/UX standard
- **Look:** a modern control-room console, clean and calm, not a hobby project. Dark theme by default (it projects well), with a light theme toggle. One accent colour, plus semantic colours used consistently: green = OK/free, amber = waiting/warning, red = alert/collision/deadlock, grey = stale/unknown.
- **Design system:** use Tailwind CSS with a small set of design tokens (colours, spacing, radius, font sizes) defined once. Font: Inter or a similar clean sans-serif; a monospace font for ids and numbers. Use tabular numerals so KPIs don't jitter.
- **Layout:** the map is the hero and takes most of the screen. KPIs sit in a slim strip on top. Robot cards, locks and alerts go in a right-hand panel with tabs or collapsible sections. No page scroll on a 1920×1080 screen; it stays usable at 1366×768.
- **No decorative animation:** no transitions, fades, slide-ins or pulsing effects. Robot positions simply update with each beacon. State changes (lock colours, new alerts) appear immediately.
- **Clarity:**
  - Hovering a robot shows a tooltip (mode, task, battery, lock, last heard).
  - Clicking a robot selects it: its path is highlighted, the rest dims and its card opens.
  - Hovering an aisle shows its holder, lease and waiters.
  - The map has a legend.
  - Every colour has a text or icon backup, so the dashboard is colour-blind safe.
- **Map controls:** zoom and pan (mouse wheel and drag), fit-to-screen button, and toggles for paths, safety circles and lidar rays.
- **Alerts:** severity icons, timestamps, the robot ids involved, and filtering by severity. Clicking an alert pans the map to where it happened.
- **States:** a proper "Connecting…" state, a "Bridge disconnected, fleet unaffected" banner with auto-reconnect, and empty states (such as "No alerts: fleet healthy"). Never show a blank screen or a raw error.
- **Presenter mode:** a toggle (key `P`) that enlarges text and the KPI strip for projectors and hides developer detail.
- **Performance:** map stays responsive with the 5-robot fleet; no memory growth over a 30-minute run. The side panel shows all 5 robot cards without scrolling and without empty space. Cap the alert list and virtualise it if needed.
- **Accessibility:** WCAG AA contrast, keyboard focus states, and aria-labels on controls.

### UI verification (part of the build loop)
- Take screenshots of the running dashboard at 1920×1080 and 1366×768 using Playwright (headless Chromium). Look at them critically and fix anything misaligned, clipped, overlapping, low-contrast or confusing. Repeat until it looks professional.
- Add Playwright smoke tests:
  - the page loads and connects
  - robots appear
  - an alert appears when a deadlock scenario runs
  - the "disconnected" banner appears when the bridge is stopped, and it recovers when the bridge restarts

Must show:
- **Live warehouse map** (SVG/canvas): walls, racks, aisles; **critical sections coloured by lock state** (free / held by Rn / occupied-unknown); robots as oriented icons with id and colour; **planned paths**; blockages; charging docks; stale robots greyed out with an inflated safety circle.
- **Robot cards:** id, mode (idle / moving / waiting for lock / yielding / charging / lost), current task, **battery bar**, last-heard age.
- **Lock table:** segment → holder, lease remaining, waiters queue.
- **Wait-for graph** mini-view, highlighting detected cycles.
- **Alert feed:** deadlock detected & broken (who yielded), aisle blocked, robot lost, task re-auctioned, partition detected/healed, lease expired.
- **KPI strip:** tasks completed, collisions (must stay 0), deadlocks resolved, mean wait at choke points, messages/bytes per second.
- **Connection indicator** and graceful behaviour when the bridge restarts.

Fault injection is done from the CLI, **not** from the dashboard (the dashboard must stay authority-free). The dashboard may display the currently active fault rules read-only.

---

## 7. Baseline and benchmark (the 20% claim)

- **Baseline B0:** centralised planner with stop-and-wait: robots halt at conflicts and are released first-in-first-out. Implement it **fairly** (same maps, speeds, tasks, seeds); do not cripple it.
- **Experiment matrix:** fleet size **5** × congestion medium/high × fault none/node_loss; **5 seeds per cell**, identical seeds for B0 and our system.
- **Metrics per run:** collisions (gate: exactly 0), unresolved deadlocks (gate: 0) and time-to-resolve, **makespan**, throughput (tasks/hour), mean wait at choke points, duplicate task executions, CPU and memory per robot process, bytes/robot/second, recovery time after fault.
- **Output:** CSV of all runs + a report (Markdown/HTML) with mean and **95% confidence interval** per cell, % makespan reduction vs B0, and charts (makespan and throughput per cell). Report honestly where the margin is below 20%.
- **Edge-budget check:** in the live runtime, measure per-robot-process CPU% and RAM and report against a Pi-class budget (4 cores, 4 GB; target < 40% of one core-equivalent budget steady state). On Windows use `psutil`; document that on Linux the same processes will be run under cgroup caps later.

---

## 8. Repository layout (suggested)

```
amr-fleet/
  README.md
  requirements.txt
  amr/
    core/        # geometry, map loading, critical-section extraction, config
    agent/       # pure agent logic: beacons, orca, safety, locks, deadlock, planner, tasks
    transport/   # UDP P2P transport, reliable events, fault shim
    world/       # physics stand-in: kinematics, lidar, collisions, blockages
    runtime/     # live (multi-process) and fast (lockstep) runners
    baseline/    # B0 stop-and-wait centralised comparator
    bench/       # sweep runner, metrics, stats, report
    faults.py    # CLI for fault rules
    bridge/      # dashboard WebSocket bridge (passive listener)
  maps/          # warehouse maps (yaml/json)
  scenarios/     # scenario definitions
  dashboard/     # React + TypeScript (Vite)
  tests/         # pytest
  scripts/       # run_demo.cmd, run_bench.cmd, etc.
```

---

## 9. Build phases (loop through all of them)

Each phase must leave a working, demonstrable system before the loop moves on. Zero-collision work comes before throughput work.

| Phase | Build | Done means |
|---|---|---|
| **P0 Foundations** | Map format + loader, critical-section and passing-bay extraction, scenario/task generator, world process (kinematics, lidar, collision detection), metrics recorder, both runtimes skeleton | A scenario runs reproducibly from a seed; metrics are logged automatically; unit tests pass |
| **P1 Transport** | UDP P2P transport, StateBeacon 10 Hz, neighbour table, stale-peer handling, reliable CoordEvents, fault shim + `amr.faults` CLI | 5 robot processes see each other; killing one is detected within 500 ms; injected loss/latency behaves as configured |
| **P2 Dashboard** | Bridge + React dashboard (Section 6) | `run_demo` shows 5 robots live on the map with paths and battery; killing the dashboard does not affect robots |
| **P3 Motion + Safety** | ORCA at 20 Hz, stale-peer inflation, lidar protective stop | Zero collisions in open-floor scenarios with and without network faults |
| **P4 Contention** | Critical sections, Ricart-Agrawala with membership + leases + sensor-confirmed entry, wait-for cycle detection, yield/back-off/ageing | `head_on` and `circular_wait` never deadlock; `node_loss` inside an aisle does not block the fleet; `partition` never puts two robots in one aisle |
| **P5 Route** | Reservation-table A* → SIPP; D* Lite repair; BlockageEvent with TTL | `blockage` scenario re-routes with no central input |
| **P6 Task + Resilience** | Sequential auction → CBBA, battery feasibility, charging, orphan re-auction; kill-any-process test | Killing a robot re-auctions its tasks; killing any single process leaves the fleet working |
| **P7 Evidence** | Baseline B0, benchmark sweep in the fast runtime, stats report, edge-budget measurement | Report with CIs; zero collisions across all runs; % makespan reduction vs B0 per cell |
| P8 (later) | ROS 2 Jazzy + Gazebo Harmonic port under WSL2 (5 robots on this laptop); agent code reused, world process replaced by Gazebo | Not in scope until I say so |
| P9 (optional) | On-device congestion predictor (small model, ONNX Runtime), advisory only, fallback heuristic; ablation on/off | Only if time allows |

**Cut line if time runs short:** never cut P3 safety, P4 contention, the fault demos or the fair baseline. Cut in this order: P9, CBBA (keep the sequential auction).

---

## 10. Testing expectations

- `pytest` unit tests for: critical-section extraction, ORCA (pairwise no-collision), Ricart-Agrawala ordering and lease expiry, membership epochs, cycle detection and victim choice, planners, auction conflict resolution, battery feasibility.
- Scenario tests in the fast runtime asserting the Phase "done means" criteria (e.g. `circular_wait` resolves within a bounded time; zero collisions).
- A single `scripts\run_demo.cmd` that starts world, robots, bridge and dashboard for a chosen scenario, and `scripts\run_bench.cmd` for the sweep.

## 11. Demo script the system must support (for judges)
1. Start 5 robots on `random_overlap`; show live map and KPIs.
2. Trigger `circular_wait`; show the cycle appear in the wait-for view and the lowest-priority robot yield.
3. Drop a blockage in an aisle; show re-routing and task re-auction.
4. Kill a robot inside a narrow aisle; show lease expiry, `occupied-unknown`, lidar-confirmed clearance, tasks re-assigned.
5. Partition the network, then heal it; show no two robots in one aisle, claims reconciled.
6. Kill the dashboard, wait, restart it; show the fleet never stopped.
7. Show the benchmark report: zero collisions, % makespan reduction vs stop-and-wait with confidence intervals.

## 12. Definition of Done (the loop ends only when all are true)

- [ ] Phases P0–P7 complete; every "Done means" criterion in Section 9 verified and recorded in `PROGRESS.md`.
- [ ] All `pytest` and Playwright tests pass.
- [ ] `scripts\run_demo.cmd` starts the full system from a clean clone after installing per `README.md`, and every step of the Section 11 demo works.
- [ ] Zero collisions across every benchmark run; zero unresolved deadlocks.
- [ ] Benchmark report generated with 95% confidence intervals and % makespan reduction vs B0, reported honestly per cell.
- [ ] Dashboard passes the UI/UX standard in Section 6, checked from screenshots at both resolutions.
- [ ] The kill-any-single-process test passes.
- [ ] `README.md` explains install, run, demo steps and architecture, with a screenshot of the dashboard.

**Start now:** create `PROGRESS.md`, propose the file structure and the map/scenario formats in it, then begin the loop at Phase 0 and keep going until the Definition of Done is met.
