// Messages pushed by the passive bridge (amr/bridge/bridge.py). The dashboard never sends anything.

export type Cell = [number, number];

export interface SectionInfo {
  sid: number;
  kind: "corridor" | "intersection";
  label: string;
  cells: Cell[];
  canonical_outside: Cell | null;
  one_way: boolean;
  exit_outside: Cell | null;
}

export interface MapInfo {
  name: string;
  width: number;
  height: number;
  rows: string[];
  picks: Cell[];
  drops: Cell[];
  docks: Cell[];
  bays: Cell[];
  sections: SectionInfo[];
  mouths: Cell[];
}

export interface Hello {
  type: "hello";
  map: MapInfo;
  scenario: { name: string; robots: number; tasks: number };
}

export type Mode = "idle" | "moving" | "waiting" | "yielding" | "charging" | "working" | "lost";

export interface Robot {
  id: number;
  x: number;
  y: number;
  th: number;
  v: [number, number];
  mode: Mode;
  stale: boolean;
  age: number;
  battery: number | null;
  task: number | null;
  stage: string;
  held: number[];
  wf: number | null;
  wq: number | null;
  prio: number | null;
  path: Cell[];
  degraded: boolean;
  epoch: number;
  live: number[];
}

export type SectionState = "free" | "held" | "unknown";

export interface SectionView {
  sid: number;
  label: string;
  state: SectionState;
  blocked: boolean;
  holder: number | null;
  lease: number | null;
  waiters: number[];
}

export type Severity = "info" | "warn" | "crit";

export interface Alert {
  id: number;
  type: string;
  sev: Severity;
  msg: string;
  robots: number[];
  x: number | null;
  y: number | null;
  t: number;
}

export interface Kpi {
  tasks_done: number;
  tasks_total: number;
  collisions: number;
  deadlocks_resolved: number;
  mean_wait_s: number;
  msgs_per_s: number;
  bytes_per_robot_s: number;
  robots_live: number;
  robots_total: number;
  min_robot_dist: number | null;
}

export interface FleetState {
  type: "state";
  t: number;
  robots: Robot[];
  sections: SectionView[];
  cycles: number[][];
  blocks: { x: number; y: number; ttl: number; by: number }[];
  pallets: { id: number; x: number; y: number; half: number }[];
  lidar?: Record<string, number[]>;
  world_ok: boolean;
  kpi: Kpi;
  faults: FaultRules;
  alerts: Alert[];
}

export interface FaultRules {
  loss?: number;
  loss_by_robot?: Record<string, number>;
  delay_ms?: number;
  jitter_ms?: number;
  partition?: number[][];
  dead_zones?: number[][];
  down?: number[];
}

export type ConnStatus = "connecting" | "live" | "disconnected" | "replay";
