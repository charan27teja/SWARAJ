import { WifiOff, Radio, Loader, Network, Hourglass, Scissors, ZapOff } from "lucide-react";
import type { ConnStatus, FaultRules } from "../types";
import { fmtClock } from "../lib/ui";

interface Props {
  status: ConnStatus;
  scenario: string | null;
  simTime: number | null;
  faults: FaultRules | null;
  url: string;
  attempt: number;
}

function faultChips(f: FaultRules | null): { icon: React.ReactNode; text: string }[] {
  if (!f) return [];
  const out: { icon: React.ReactNode; text: string }[] = [];
  if (f.partition?.length) out.push({ icon: <Scissors size={12} />, text: `Partition ${f.partition.map((g) => `{${g.join(",")}}`).join(" | ")}` });
  if (f.dead_zones?.length) out.push({ icon: <ZapOff size={12} />, text: `${f.dead_zones.length} dead zone${f.dead_zones.length > 1 ? "s" : ""}` });
  if (f.loss) out.push({ icon: <Network size={12} />, text: `Loss ${(f.loss * 100).toFixed(0)}%` });
  if (f.loss_by_robot && Object.keys(f.loss_by_robot).length)
    out.push({ icon: <Network size={12} />, text: Object.entries(f.loss_by_robot).map(([r, p]) => `R${r} loss ${(p * 100).toFixed(0)}%`).join(", ") });
  if (f.delay_ms) out.push({ icon: <Hourglass size={12} />, text: `Delay ${f.delay_ms} ms${f.jitter_ms ? ` ±${f.jitter_ms}` : ""}` });
  return out;
}

export function TopBar(p: Props) {
  const chips = faultChips(p.faults);
  const conn =
    p.status === "replay"
      ? { cls: "bg-panel-2 text-muted", icon: <Radio size={13} />, text: "PEER-TO-PEER" }
      : p.status === "live"
      ? { cls: "bg-ok-soft text-ok", icon: <Radio size={13} />, text: "LIVE" }
      : p.status === "connecting"
        ? { cls: "bg-panel-2 text-muted", icon: <Loader size={13} />, text: "Connecting…" }
        : { cls: "bg-warn-soft text-warn", icon: <WifiOff size={13} />, text: `Reconnecting (try ${p.attempt})` };
  return (
    <header className="flex h-14 min-w-0 items-center gap-3 overflow-hidden border-b border-line bg-panel px-4 sm:gap-4">
      <div className="flex min-w-0 shrink-0 items-center gap-2 sm:gap-3">
        <svg width="24" height="24" viewBox="0 0 24 24" aria-hidden="true" className="shrink-0 text-accent">
          <circle cx="8" cy="8" r="1.5" fill="currentColor" />
          <circle cx="16" cy="8" r="1.5" fill="currentColor" />
          <circle cx="12" cy="16" r="1.5" fill="currentColor" />
          <path d="M8 8 L12 16 M16 8 L12 16" stroke="currentColor" strokeWidth="1.5" fill="none" strokeLinecap="round" />
        </svg>
        <div className="min-w-0 leading-tight">
          <div className="truncate text-sm font-semibold tracking-tight text-fg sm:text-[15px]">SWARAJ</div>
        </div>
      </div>

      <div className="ml-1 flex min-w-0 flex-1 items-center gap-1.5 overflow-hidden sm:ml-2 sm:gap-2">
        {p.scenario && (
          <span className="shrink-0 rounded-[var(--radius-chip)] border border-line bg-panel-2 px-2 py-1 text-xs text-muted">
            <span className="hidden sm:inline">Scenario </span><span className="font-medium text-fg">{p.scenario}</span>
          </span>
        )}
        {chips.map((c, i) => (
          <span key={i} className="flex shrink-0 items-center gap-1 rounded-[var(--radius-chip)] bg-warn-soft px-2 py-1 text-2xs font-medium text-warn sm:gap-1.5 sm:text-xs"
                title="Active fault rule (read-only; injected with python -m amr.faults)">
            {c.icon}<span className="hidden sm:inline">{c.text}</span>
          </span>
        ))}
      </div>

      <div className="flex shrink-0 items-center gap-1.5 sm:gap-2">
        {p.simTime != null && (
          <span className="num hidden rounded-[var(--radius-chip)] px-2 py-1 text-sm text-muted md:inline" title="Time since the bridge started observing">
            {fmtClock(p.simTime)}
          </span>
        )}
        <span className={`flex items-center gap-1 rounded-full px-2 py-1 text-2xs font-semibold sm:gap-1.5 sm:px-2.5 sm:text-xs ${conn.cls}`}
              role="status" aria-live="polite" data-testid="conn-status">
          {conn.icon}<span className="hidden sm:inline">{conn.text}</span>
        </span>
        <span className="dev-only hidden text-2xs text-muted xl:inline">{p.url}</span>
      </div>
    </header>
  );
}
