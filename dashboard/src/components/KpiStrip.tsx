import { CheckCircle2, AlertOctagon, PackageCheck, Unlink, Timer, Bot, MessagesSquare, Gauge } from "lucide-react";
import type { Kpi } from "../types";
import { fmtBytes } from "../lib/ui";

function Tile({ label, icon, children, tone, dev, testid }: {
  label: string; icon: React.ReactNode; children: React.ReactNode;
  tone?: "ok" | "crit" | "warn"; dev?: boolean; testid?: string;
}) {
  const ring = tone === "crit" ? "border-crit/60 bg-crit-soft" : "border-line bg-panel";
  return (
    <div className={`flex min-w-0 flex-1 items-center gap-3 rounded-[var(--radius-card)] border px-3.5 py-2 ${ring} ${dev ? "dev-only" : ""}`}
         data-testid={testid}>
      <span className={`hidden shrink-0 lg:inline ${tone === "ok" ? "text-ok" : tone === "crit" ? "text-crit" : tone === "warn" ? "text-warn" : "text-muted"}`}>
        {icon}
      </span>
      <div className="min-w-0">
        <div className="kpi-label truncate text-xs font-medium text-muted">{label}</div>
        <div className="kpi-value num truncate text-lg font-semibold leading-6 text-fg">{children}</div>
      </div>
    </div>
  );
}

export function KpiStrip({ kpi }: { kpi: Kpi | null }) {
  const k = kpi;
  const pct = k && k.tasks_total ? Math.min(100, (100 * k.tasks_done) / k.tasks_total) : 0;
  return (
    <section aria-label="Key performance indicators" className="kpi-strip flex gap-2.5 border-b border-line bg-bg px-3 py-2.5">
      <Tile label="Tasks done" icon={<PackageCheck size={20} />} testid="kpi-tasks">
        {k ? (
          <span className="flex items-baseline gap-2">
            {k.tasks_done}
            <span className="text-sm font-normal text-muted">/ {k.tasks_total}</span>
            <span className="relative ml-1 hidden h-1.5 w-16 overflow-hidden rounded-full bg-panel-2 lg:inline-block" aria-hidden="true">
              <span className="absolute inset-y-0 left-0 bg-accent" style={{ width: `${pct}%` }} />
            </span>
          </span>
        ) : "—"}
      </Tile>
      <Tile label="Collisions" icon={k && k.collisions > 0 ? <AlertOctagon size={20} /> : <CheckCircle2 size={20} />}
            tone={k ? (k.collisions > 0 ? "crit" : "ok") : undefined} testid="kpi-collisions">
        {k ? <span>{k.collisions}<span className="ml-2 font-sans text-xs font-medium text-ok">{k.collisions ? "" : "✓ none"}</span></span> : "—"}
      </Tile>
      <Tile label="Deadlocks resolved" icon={<Unlink size={20} />} testid="kpi-deadlocks">{k ? k.deadlocks_resolved : "—"}</Tile>
      <Tile label="Choke-point wait" icon={<Timer size={20} />} tone={k && k.mean_wait_s > 10 ? "warn" : undefined}>
        {k ? <>{k.mean_wait_s.toFixed(1)}<span className="ml-1 text-sm font-normal text-muted">s</span></> : "—"}
      </Tile>
      <Tile label="Robots live" icon={<Bot size={20} />} tone={k && k.robots_live < k.robots_total ? "warn" : undefined}>
        {k ? <>{k.robots_live}<span className="ml-1 text-sm font-normal text-muted">/ {k.robots_total}</span></> : "—"}
      </Tile>
      <Tile label="Messages/s" icon={<MessagesSquare size={20} />}>{k ? k.msgs_per_s.toFixed(0) : "—"}</Tile>
      <Tile label="Bytes/s per robot" icon={<Gauge size={20} />} dev>{k ? fmtBytes(k.bytes_per_robot_s) : "—"}</Tile>
    </section>
  );
}
