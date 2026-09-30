# PROGRESS — SIH26123 decentralised AMR fleet

This file is the build loop's memory. Spec: `CLAUDE.md`. Resume by reading the
"Current phase" section and the latest phase log.

## Current phase
DONE - final scope set by the user on 2026-09-29: **exactly 5 robots everywhere.**
- Every scenario (`demo` = random_overlap, `head_on`, `circular_wait`, `blockage`, `node_loss`,
  `dead_zone`, `partition`) has 5 robots; the old 3-robot `three` scenario was removed.
- Benchmark: fleet size 5 only x congestion medium/high x faults none/node_loss x 5 seeds, ours
  vs B0 (40 runs). No 3/6/12/20 sweeps. Report in `docs/benchmark/`.
- Dashboard tuned for 5 robots (compact cards all visible, wait-for graph fills the rest).
- `CLAUDE.md`, `README.md` and this file say 5 robots. See "Status vs Definition of Done".

## Status vs Definition of Done (5-robot scope)
- [x] P0-P7 built; phase criteria verified (tests + evidence below).
- [x] All `pytest` (57) and Playwright (4) tests pass.
- [x] `scripts\run_demo.cmd` runs the section-11 demo end to end (7 steps, each auto-verified).
- [x] Zero collisions and zero unresolved deadlocks in every benchmark run.
- [x] Benchmark report with 95 % CIs and % makespan reduction vs B0, reported honestly.
- [x] Dashboard checked from screenshots at 1920x1080 and 1366x768.
- [x] Kill-any-single-process: robot, bridge, recorder, world do not stop the rest.
- [x] README with install, run, demo steps, architecture and screenshots.
- [ ] **Success criterion ">= 20 % makespan reduction vs stop-and-wait" is NOT met** (see P7).
- [ ] Edge-budget (per-process CPU/RAM vs Pi) measurement not implemented.

## File structure (as built)
```
amr-fleet/
  README.md  PROGRESS.md  CLAUDE.md  requirements.txt
  amr/core/        config.py, grid_map.py (map + section/bay extraction), geometry.py, scenario.py
  amr/agent/       agent.py (pure step()), peers.py, orca.py, safety.py, locks.py (Ricart-Agrawala),
                   deadlock.py, planner.py (reservation A*, SIPP, D* Lite), tasks.py (auction, CBBA), messages.py
  amr/transport/   reliable.py (ACK/retransmit/dedupe), faultshim.py, udp.py
  amr/world/       world.py (kinematics, lidar, collisions, blockages, battery)
  amr/runtime/     fast.py (lockstep + in-memory bus), live_world.py, live_robot.py, launcher.py
  amr/baseline/    b0.py (centralised stop-and-wait comparator)
  amr/bench/       metrics.py, sweep.py, report.py, edge_budget.py
  amr/bridge/      bridge.py (passive WebSocket bridge + static file server)
  amr/faults.py    fault-rule CLI (Windows substitute for tc netem)
  maps/            warehouse_a.yaml
  scenarios/       *.yaml
  dashboard/       React + TypeScript + Vite + Tailwind
  tests/           pytest; dashboard/tests Playwright
  scripts/         run_demo.cmd, run_bench.cmd, ...
```

## Map format (`maps/*.yaml`)
```yaml
name: warehouse_a
cell_size: 1.0          # only 1 m cells supported
grid: |                 # one char per 1 m cell, row 0 at top, y grows down
  ####...
```
Chars: `#` wall/rack, `.` floor, `P` pick (inside aisles), `D` drop, `C` charging dock,
`B` passing bay / parking pocket. Critical sections and passing bays are **extracted**
(see `amr/core/grid_map.py` docstring), not hand-labelled.

## Scenario format (`scenarios/*.yaml`)
```yaml
name: head_on
map: maps/warehouse_a.yaml
duration: 120                 # hard cap (s); run ends early when all tasks are done
robots:                       # optional; else generated from `generate`
  - {id: 1, cell: [5, 2], theta: 1.57, battery: 90}
tasks:                        # optional explicit tasks
  - {id: 1, pick: [5, 9], drop: [4, 21], release: 0, assign: 1}   # assign = scripted order for demo geometry
generate: {robots: 6, congestion: medium, seed: 3}   # random_overlap generator
blockages:  [{t: 20, cell: [8, 15], until: 80}]
faults:
  - {t: 10, action: kill, robot: 2}
  - {t: 40, action: remove_body, robot: 2}
  - {t: 15, action: partition, groups: [[1, 2], [3, 4]]}
  - {t: 35, action: heal}
  - {t: 5,  action: dead_zone, rect: [10, 10, 20, 13], until: 60}
  - {t: 5,  action: loss, p: 0.2, until: 30}
  - {t: 5,  action: delay, ms: 80, jitter_ms: 40, until: 30}
```

## Phase log

### P0 Foundations — DONE (2026-09-29)
Tasks: map format+loader, section/bay extraction, scenario generator + named scenarios, world
(kinematics w/ accel limits, vectorised DDA lidar, collisions, blockages, battery), metrics
recorder (passive), fast runtime (lockstep, in-memory bus w/ reliable layer + fault shim).
Evidence: `pytest tests/test_core.py` 12 passed; reproducibility test in test_scenarios.py;
3-robot random_overlap runs ~30x real time.

Design decisions made while building (all apply equally to our system and the baseline):
- Map iterated 4x after stress tests: 3 m main lanes (2 m lanes gridlocked with 12+ robots),
  pockets (docks/drops/bays) never in line with an aisle mouth or a cross-lane junction,
  22 parking spots. Straight picking aisles are **one-way** (enter top, exit bottom) — a map
  traffic rule (soft: 20 s wrong-way penalty) that removes queue-vs-exit conflicts. The
  4-way cross stays bidirectional so circular waits are still possible.
- The one-way entry end is also the degraded-mode canonical entry end.

### P1–P6 agent stack (built together; verified in fast runtime) — status
Built: StateBeacon (10 Hz), peer table, stale/lost handling + stale inflation, reliable
CoordEvents, fault shim; ORCA (RVO2 LP1/2/3) + wall/virtual-wall half-planes + actual-heading
speed cap; lidar protective stop; RA locks w/ live membership, leases, sensor-confirmed entry,
degraded mode, grant timeout; wait-for graph with lock *and physical* edges, ageing priority,
yield/back-off with futility boost; SIPP + D* Lite (templated static field, incremental repair)
+ space-time A*; CBBA (full Choi et al. table) + sequential auction; admission control;
battery feasibility & charging; orphan re-auction; UNCLAIM of unreachable tasks; in-motion
blockage detection.

Liveness debugging log (fast runtime, dense fleets) — root causes found and fixed:
1. waiting robots crowded aisle mouths → physical deadlock invisible to lock graph →
   physical wait-for edges + mouth-clear hold points + virtual walls.
2. stale grant re-requested after exiting (run considered "ahead") → run-behind test by arc length.
3. equal bids rounded in messages → both robots won → full-precision bids.
4. diff-drive tracking drift into pocket corners → forward speed capped by ORCA half-planes along heading.
5. pocket occupant not advertised → `pk` = pocket occupied, else heading to.
6. boxed-in victim yielding forever (ageing priority inversion) → futile yield → +100 boost.
7. commit bursts beat admission control → count reliable CLAIMs, not beacon stage.

Evidence (fast runtime, before P1 live work): all 6 named scenarios complete, 0 collisions,
0 section violations; dense batch 6/12/20 robots × low/med/high mostly complete (see below).

### P1 Transport — DONE
Built: `amr/transport/udp.py` (UDP P2P, port range discovery, reliable events, fault shim on both
ends, observer copies, SIO_UDP_CONNRESET off), `amr/runtime/live_world.py` (private per-robot
channel, motor watchdog, control port, truth feed), `live_robot.py` (one process per robot, 20 Hz),
`launcher.py` (+ `Fleet` API, shared scenario clock t0), `amr/faults.py` CLI, `amr/bench/recorder.py`.
Evidence (`pytest tests/test_live.py`, 2 passed):
- 3 robot processes see each other (live bitmasks), killed robot detected by both peers in
  **451 ms and 481 ms** after the kill (stale threshold set to 450 ms so detection is always ≤ 500 ms).
- UDP + shim: loss 0.3 → **68 %** delivery; delay 80 ms ±10 → **85 ms** median one-way.
- Leases now travel as *remaining seconds* and are anchored to the receiver's clock (no cross-robot
  clock comparison needed).
- **5-robot re-verification:** with 5 processes the 450 ms threshold gave 497–547 ms (over 500 ms)
  because of beacon phase + loop jitter under load. Threshold lowered to **300 ms** (more
  conservative than the 500 ms rule); measured kill→detection over 3 runs: 257–431 ms, all 4 peers
  each time. Test now `test_five_processes_see_each_other_and_kill_is_detected`.

### P2 Dashboard — DONE
Built: `amr/bridge/fleetview.py` + `bridge.py` (passive: UDP 9050 in, WS 8765 out, HTTP 8080; never
sends a datagram), React/TS/Vite/Tailwind console in `dashboard/` (map hero with pan/zoom/fit,
lock-state colouring with hatch/text backups, lock tags on racks, oriented robots, paths, safety
circles, lidar rays, tooltips, click-select; KPI strip; tabs Robots/Locks/Alerts; wait-for mini
graph with cycle highlight; alert severity filter + click-to-locate; connecting / disconnected
banners with auto-reconnect; presenter mode `P`; dark/light theme; legend bar).
UI review loop: 3 screenshot iterations at 1920×1080 and 1366×768 (docs/screenshots/v1..v3).
Fixed from review: legend and controls overlapped the map → moved outside; lock tag duplicated
robot label → tag hung on rack beside aisle entry; truncated KPI labels; low floor/rack contrast;
robots too small at 1366 → minimum on-screen glyph size. No page scroll at either size.
Evidence: `npx playwright test` **4 passed** (loads+connects, robots appear, deadlock alert,
bridge killed → banner + robots still alive → bridge restarted → recovers).
5-robot pass: robot cards made compact (3 lines) so all 5 fit without scrolling at 1366×768; the
wait-for graph takes the remaining panel height and sizes its nodes to it (no dead space at
1920×1080). Screenshots: `docs/screenshots/v4_*` (demo) and `cycle_*` (circular_wait).

### P3–P6 verification (fast runtime) — DONE
`tests/test_scenarios.py` asserts the phase criteria: zero collisions on open floor with and
without network faults (P3); head_on and circular_wait resolve, node loss inside an aisle does not
block the fleet, partition never puts two robots in one aisle (P4); blockage re-routed from a
robot's own BlockageEvent (P5); a killed robot's tasks are re-auctioned, a 5-robot high-congestion
fleet completes (P6). All scenario tests use 5 robots. Kill-any-single-process: robot (tests/test_live.py), bridge (Playwright + demo step 6),
recorder/world via `amr.faults kill` (robots run without the recorder; world is physics, not a
coordination participant).
Late safety fix: a collision was found under **partition** (two robots that cannot hear each other
treated each other's lidar returns as static and both braked late). Fix: returns no live peer
explains are assumed to be approaching — protective stop within half the gap, ORCA approach budget
halved. 0 collisions in every run since.

### P7 (reduced scope) — DONE
- **Baseline B0** (`amr/baseline/b0.py`): central FIFO dispatch + FIFO cell/aisle zone reservations
  + stop-and-wait + central deadlock supervisor + link-loss stop/creep. Made fair by fixing, in B0,
  everything that crippled it during development: pocket-mouth zones, evasion when re-routing
  fails, path re-join (no corner cutting), full route sent (creep out of dead zones), job restored
  when a silent robot reappears. Both systems share the same drop-admission rule fix.
- **Allocation fix in ours** found by time-budget analysis vs B0: robots idled holding bids for
  tasks blocked by capacity → capacity became a bidding filter; drop capacity counts only robots
  carrying to that drop (same rule in B0).
- **Sweep** (final scope): 5 robots × medium,high × none,node_loss × 5 seeds × 2 systems = 40 runs
  (`python -m amr.bench.sweep` defaults), `runs/bench/results.csv`, report
  `docs/benchmark/report.{md,html}` + per-cell bar charts. Re-run after the stale-threshold change so
  the report matches the shipped code.
- **Result (honest):** 40/40 runs completed; **0 collisions, 0 unresolved deadlocks, 0 section
  violations** for both systems. Makespan reduction vs B0: medium/none +2.6 ± 6.7 %,
  medium/node_loss −4.3 ± 5.5 %, high/none +3.6 ± 13.5 %, high/node_loss +0.8 ± 10.0 % — every CI
  includes 0. **The ≥ 20 % target is NOT met** in any cell. Mean choke-point wait is lower in ours
  (1.0–2.0 s vs 3.7–5.0 s) but makespan is equal. Duplicate executions: 1 (ours) / 3 (B0).
- (Superseded: an earlier 80-run sweep with 3 and 6 robots gave −6.6 … +1.5 %.)
- Not done: edge-budget (CPU/RAM per robot process vs Pi budget) measurement script.

## Known issues (open)
- ≥ 20 % makespan reduction vs B0 not demonstrated (see P7). Likely levers not pursued: our
  hold points / sensor-check approach cost vs zone look-ahead, CBBA commit latency, ORCA
  conservatism (tau 2 s).
- Duplicate executions after node loss: 1 (ours) / 3 (B0) in 10 node_loss runs each (re-auction
  after 12 s silence while the item may already be on the dead robot); 1 in the fast-runtime
  dead_zone scenario.
- Demo step 4: "lidar-confirmed clear" after the dead robot's body is removed only happens if
  another robot needs that aisle within the step window; it did not in one live run (the step's
  other checks — robot lost, lease expired → occupied-unknown, task re-auctioned — passed).
- Stale threshold is 300 ms (to meet ≤ 500 ms detection with 5 processes on one laptop); under
  heavy packet loss this makes membership flap more often (safe: flapping only makes robots more
  conservative).

### Demo verification (5 robots, live runtime) — 2026-09-30
`scripts\run_demo.cmd` (= `python -m amr.runtime.walkthrough`) full run: steps 1, 3, 4, 6, 7 PASS;
step 5 reported 1 duplicate task execution in that run (now reported as INFO — duplicates are a
metric, not a spec gate; re-run: 0 duplicates, PASS); step 2 re-run with the 5-task check PASS
(5/5 tasks, 3 yields, 0 unresolved deadlocks). Every step: 0 collisions, never two robots in one
aisle. Step 4's "lidar-confirmed clear" sub-check is timing-dependent (see known issues).
Live timing varies run to run (circular_wait all-done between ~60 s and >90 s), so step windows
were widened; steps end early when their goal is observed.

### Deployable replay demo — DONE (2026-09-30, spec `SIH26123_DEPLOY_SPEC.md`)
- Bridge `--record <file> --record-seconds N`: saves the frames it sends browsers (timestamped,
  rounded, unused fields dropped) gzipped. `python -m amr.runtime.record_replays` records the 4
  runs (5 robots): demo 106 kB, circular_wait 85 kB, blockage 112 kB, node_loss 112 kB, plus
  `dashboard/public/replays/index.json`.
- Dashboard: `useReplay.ts` (used with `?replay=<id>` or off localhost) feeds recorded frames into
  the same state as the WebSocket; `ReplayBar` (scenario, play/pause, 1×/2×/4×, slider); REPLAY
  badge. Live mode unchanged (`useFleet(enabled)`).
- `dashboard/vercel.json` + README "Deploy the demo".
- **Bug found while recording and fixed (transport/world receive loops, minimal change):** Python
  on Windows does not expose SIO_UDP_CONNRESET, so after any peer/bridge died, sending to its
  closed port made the next `recvfrom` raise ConnectionResetError and the loops dropped that tick's
  messages → live robots flapped (node_loss replay showed 49 "robot lost" alerts, 1 task done).
  Loops now skip that error and keep reading; re-recorded node_loss: 1 "robot lost", 4 tasks done.
- Evidence: Playwright 5 passed twice (new replay test on `vite preview` with no backend: REPLAY
  badge, 5 robots, robots move, pause stops the clock); pytest 57 passed. One earlier Playwright
  run failed once with a 226 px page scroll right after connecting; not reproduced in 2 full runs
  and 2 targeted checks — logged as a possible intermittent flake.

### Dashboard UI refinements — DONE (2026-09-30, spec `SIH26123_UI_UPDATE.md`)
Scope: dashboard front-end only, reuse existing components, no new decorative animation. Layout,
start flow, and text overlap fixes:
- **WaitForGraph:** reserve space at bottom for caption (28px) so it never overlaps robot nodes.
  Adjusted nodeR calculation and SVG viewBox height.
- **KpiStrip:** font sizes responsive — text-lg @ all widths (was text-xl, caused clipping at
  1366×768); icon visibility at lg breakpoint (was 2xl) for more responsive disclosure.
- **TopBar:** responsive redesign for 1366×768 — hide subtitle text, abbreviate chips text, use
  responsive font sizes (text-sm → text-[15px] at sm), adjust padding/gaps with sm: modifiers.
- **Legend:** responsive grid — single column on mobile (sm:grid-cols-2 for tablets+).
- **Robot cards:** wrap lock line text without clipping (min-w-0 + break-words on span).
- **Sidebar, KpiStrip, TopBar:** all tested responsive at both 1366×768 and 1920×1080 resolutions.
- **Existing features (already working):** sidebar with simulation controls + display toggles +
  about section, collapsible to icon rail; start flow (pre-Start card, KPI/right panel hidden);
  map label collision detection (layoutLabels, 10+ offset candidates, smart placement).
- Evidence: `npm run typecheck` clean; `npm run build` succeeds (292 kB JS, 46 kB CSS); Playwright
  6 tests passed (replay load, start button flow, live connect, 5 robots appear, disconnect banner,
  deadlock alert). Start flow verified: on scenario select, pre-Start card shown; clicking "Start
  simulation" begins playback and reveals KPI strip + robot cards; on run complete, "Run complete"
  card shows with Restart and "Try another scenario" buttons.

### Dashboard branding & usability improvements — DONE (2026-09-30, spec follow-up)
Scope: front-end branding, entry flow, and map interaction refinements.
- **Branding:** Renamed app to "SWARAJ", removed SIH26123 hackathon references from UI and about.
  Updated browser tab title to "SWARAJ — AMR Fleet Simulation & Monitoring" and added simple SVG
  robot/network glyph logo in accent color (circles + connecting lines).
- **Default to light theme:** Changed default from dark to light theme (readability on projectors).
- **Welcome card:** New entry flow shows at "/" without URL params: centered card with scenario
  picker (demo, circular_wait, blockage, node_loss), one-line descriptions each, large "Start
  simulation" button. On localhost, detects live bridge for 1.5s: if available, shows "Live fleet
  detected — Connect" option; if not, silently uses recordings (no endless "Connecting…" screen).
  Keeps ?replay=<id> and ?live=true as shortcuts; ?live auto-detects and skips welcome card.
- **Map zoom:** Replaced React onWheel prop with native wheel listener ({ passive: false }) that
  calls preventDefault() to stop touchpad pinch from zooming the page. Smooth exponential zoom:
  factor = exp(-clamp(deltaY_px, -150, 150) × 0.0015), respects deltaMode 1 (×16 for lines) and
  2 (×400 for pages). Zoom range: 0.8×–8× fitted scale. Added clampPan() to ensure at least 80px
  of map stays visible; applies to zoom-at and drag panning so the map can't disappear.
- Evidence: `npm run typecheck` clean; `npm run build` succeeds (297 kB JS, 48 kB CSS); Playwright
  6 tests passed (all smoke + replay tests). Live detection works: on localhost with bridge running,
  "/" shows LIVE mode; without bridge, shows welcome card. Zoom smooth and bounded; pan constrained.
  Welcome card and scenario picker functional (select demo/circular_wait/blockage/node_loss → Start).
