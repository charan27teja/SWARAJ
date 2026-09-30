import type { Mode, Severity } from "../types";

// 20 identity colours for robots, chosen to stay distinguishable on both themes. Identity is
// never conveyed by colour alone: every robot is always labelled with its id.
const PALETTE = [
  "#4aa3ff", "#f4a261", "#2ec4b6", "#e76f98", "#a78bfa", "#8ac926", "#ffca3a", "#ff7b54",
  "#56cfe1", "#c77dff", "#90be6d", "#f9844a", "#4d96ff", "#f15bb5", "#00bbf9", "#b5e48c",
  "#fee440", "#9b5de5", "#00f5d4", "#ff9f1c",
];

export function robotColor(id: number): string {
  return PALETTE[(id - 1) % PALETTE.length];
}

export const MODE_LABEL: Record<Mode, string> = {
  idle: "Idle",
  moving: "Moving",
  waiting: "Waiting for lock",
  yielding: "Yielding",
  charging: "Charging",
  working: "Picking / dropping",
  lost: "Lost",
};

export const STAGE_LABEL: Record<string, string> = {
  to_pick: "to pick",
  pick: "picking",
  to_drop: "to drop",
  drop: "dropping",
  to_dock: "to dock",
  charging: "charging",
  to_spot: "parking",
  parked: "parked",
};

/** semantic tone of a mode: ok (green) / warn (amber) / crit (red) / unknown (grey) / neutral */
export function modeTone(m: Mode, stale: boolean): "ok" | "warn" | "crit" | "unknown" | "accent" {
  if (stale || m === "lost") return "unknown";
  if (m === "waiting") return "warn";
  if (m === "yielding") return "crit";
  if (m === "charging") return "accent";
  return "ok";
}

export const TONE_TEXT: Record<string, string> = {
  ok: "text-ok",
  warn: "text-warn",
  crit: "text-crit",
  unknown: "text-unknown",
  accent: "text-accent",
};

export const TONE_BG: Record<string, string> = {
  ok: "bg-ok-soft text-ok",
  warn: "bg-warn-soft text-warn",
  crit: "bg-crit-soft text-crit",
  unknown: "bg-panel-2 text-unknown",
  accent: "bg-accent-soft text-accent",
};

export const SEV_LABEL: Record<Severity, string> = { info: "Info", warn: "Warning", crit: "Critical" };

export function fmtClock(t: number): string {
  if (!isFinite(t) || t < 0) t = 0;
  const m = Math.floor(t / 60);
  const s = Math.floor(t % 60);
  return `${String(m).padStart(2, "0")}:${String(s).padStart(2, "0")}`;
}

export function fmtAge(a: number): string {
  if (a < 1) return `${Math.round(a * 1000)} ms`;
  if (a < 60) return `${a.toFixed(1)} s`;
  return `${Math.floor(a / 60)} min`;
}

export function fmtBytes(b: number): string {
  if (b >= 1024 * 1024) return `${(b / 1024 / 1024).toFixed(1)} MB`;
  if (b >= 1024) return `${(b / 1024).toFixed(1)} kB`;
  return `${Math.round(b)} B`;
}
