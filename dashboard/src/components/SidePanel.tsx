import { useEffect, useMemo, useRef, useState } from "react";
import type { KeyboardEvent } from "react";
import { AlertOctagon, AlertTriangle, BatteryCharging, CheckCircle2, Info, Lock, LocateFixed, Unlock, HelpCircle } from "lucide-react";
import type { Alert, Robot, SectionView, Severity } from "../types";
import { MODE_LABEL, STAGE_LABEL, TONE_BG, fmtAge, fmtClock, modeTone, robotColor } from "../lib/ui";
import { WaitForGraph } from "./WaitForGraph";

type Tab = "robots" | "locks" | "alerts";

interface Props {
  robots: Robot[];
  sections: SectionView[];
  alerts: Alert[];
  cycles: number[][];
  selected: number | null;
  onSelect: (id: number | null) => void;
  onLocate: (x: number, y: number) => void;
}

export function SidePanel(p: Props) {
  const [tab, setTab] = useState<Tab>("robots");
  const held = p.sections.filter((s) => s.state !== "free" || s.blocked || s.waiters.length).length;
  const crit = p.alerts.filter((a) => a.sev === "crit").length;
  useEffect(() => {
    if (p.selected != null) setTab("robots");
  }, [p.selected]);
  const tabs: { id: Tab; label: string; badge?: number; tone?: string }[] = [
    { id: "robots", label: "Robots", badge: p.robots.length },
    { id: "locks", label: "Locks", badge: held },
    { id: "alerts", label: "Alerts", badge: p.alerts.length, tone: crit ? "bg-crit text-white" : undefined },
  ];
  const onKey = (e: KeyboardEvent) => {
    const i = tabs.findIndex((t) => t.id === tab);
    if (e.key === "ArrowRight") setTab(tabs[(i + 1) % tabs.length].id);
    if (e.key === "ArrowLeft") setTab(tabs[(i + tabs.length - 1) % tabs.length].id);
  };
  return (
    <aside className="flex min-h-0 flex-col border-l border-line bg-panel" aria-label="Fleet details">
      <div role="tablist" aria-label="Panels" className="flex border-b border-line px-2" onKeyDown={onKey}>
        {tabs.map((t) => (
          <button key={t.id} role="tab" type="button" aria-selected={tab === t.id} aria-controls={`panel-${t.id}`}
                  id={`tab-${t.id}`} tabIndex={tab === t.id ? 0 : -1} onClick={() => setTab(t.id)}
                  className={`flex items-center gap-2 border-b-2 px-3 py-2.5 text-sm font-medium ${tab === t.id ? "border-accent text-fg" : "border-transparent text-muted hover:text-fg"}`}>
            {t.label}
            {t.badge != null && (
              <span className={`num rounded-full px-1.5 text-2xs ${t.tone ?? "bg-panel-2 text-muted"}`}>{t.badge}</span>
            )}
          </button>
        ))}
      </div>
      <div className="scroll-thin min-h-0 shrink overflow-y-auto" role="tabpanel" id={`panel-${tab}`} aria-labelledby={`tab-${tab}`}>
        {tab === "robots" && <RobotList robots={p.robots} sections={p.sections} selected={p.selected} onSelect={p.onSelect} />}
        {tab === "locks" && <LockTable sections={p.sections} />}
        {tab === "alerts" && <AlertFeed alerts={p.alerts} onLocate={p.onLocate} />}
      </div>
      {/* the wait-for view takes whatever height the tab content leaves (no dead space) */}
      <div className="flex min-h-[120px] flex-1 flex-col border-t border-line">
        <WaitForGraph robots={p.robots} cycles={p.cycles} />
      </div>
    </aside>
  );
}

/* ------------------------------------------------------------------ robots */
function Battery({ v }: { v: number | null }) {
  const pct = Math.max(0, Math.min(100, v ?? 0));
  const tone = pct < 20 ? "bg-crit" : pct < 45 ? "bg-warn" : "bg-ok";
  return (
    <div className="flex items-center gap-2" aria-label={`Battery ${pct.toFixed(0)} percent`}>
      <div className="relative h-2 w-20 overflow-hidden rounded-full bg-panel-2">
        <div className={`absolute inset-y-0 left-0 ${tone}`} style={{ width: `${pct}%` }} />
      </div>
      <span className="num w-9 text-right text-xs text-fg">{pct.toFixed(0)}%</span>
    </div>
  );
}

function RobotList({ robots, sections, selected, onSelect }: {
  robots: Robot[]; sections: SectionView[]; selected: number | null; onSelect: (id: number | null) => void;
}) {
  const refs = useRef<Record<number, HTMLButtonElement | null>>({});
  useEffect(() => {
    if (selected != null) refs.current[selected]?.scrollIntoView({ block: "nearest" });
  }, [selected]);
  const lab = (sid: number | null) => (sid == null ? null : sections.find((s) => s.sid === sid)?.label ?? `#${sid}`);
  if (!robots.length) return <Empty icon={<HelpCircle size={18} />} text="No robots heard yet" sub="Waiting for the first beacons…" />;
  return (
    <ul className="space-y-1.5 p-2">
      {robots.map((r) => {
        const tone = modeTone(r.mode, r.stale);
        const sel = selected === r.id;
        return (
          <li key={r.id}>
            <button type="button" ref={(el) => { refs.current[r.id] = el; }}
                    onClick={() => onSelect(sel ? null : r.id)} aria-pressed={sel}
                    aria-label={`Robot ${r.id} card`} data-testid={`robot-card-${r.id}`}
                    className={`w-full rounded-[var(--radius-card)] border px-3 py-1.5 text-left ${sel ? "border-accent bg-accent-soft" : "border-line bg-panel hover:bg-panel-2"} ${r.stale ? "opacity-70" : ""}`}>
              <div className="flex items-center gap-2">
                <span className="inline-block h-3 w-3 shrink-0 rounded-full" style={{ background: r.stale ? "var(--c-unknown)" : robotColor(r.id) }} />
                <span className="num text-sm font-semibold text-fg">R{r.id}</span>
                <span className={`rounded-[var(--radius-chip)] px-1.5 py-0.5 text-2xs font-semibold ${TONE_BG[tone]}`}>
                  {r.stale ? (r.mode === "lost" ? "Lost" : "Stale") : MODE_LABEL[r.mode]}
                </span>
                {r.degraded && !r.stale && (
                  <span className="rounded-[var(--radius-chip)] bg-warn-soft px-1.5 py-0.5 text-2xs font-semibold text-warn" title="Some peers are unreachable: conservative lock rules">
                    degraded
                  </span>
                )}
                <span className="ml-auto"><Battery v={r.battery} /></span>
              </div>
              <div className="mt-1 flex items-baseline justify-between gap-2 text-xs">
                <span className="min-w-0 truncate text-fg">
                  <span className="text-muted">Task </span>
                  {r.task != null ? <><span className="num">#{r.task}</span> · {STAGE_LABEL[r.stage] ?? r.stage}</> : r.mode === "charging" ? <><BatteryCharging size={12} className="inline" /> charging</> : "—"}
                </span>
                <span className={`num shrink-0 text-2xs ${r.stale ? "text-unknown" : "text-muted"}`}
                      title={`priority ${r.prio?.toFixed(1) ?? "—"} · membership epoch ${r.epoch}`}>
                  heard {fmtAge(r.age)} ago
                </span>
              </div>
              {/* the lock line wraps rather than being cut off */}
              <div className="mt-0.5 min-w-0 text-xs leading-snug text-fg">
                <span className="text-muted">Lock </span>
                <span className="break-words">
                  {r.held.length ? <><Lock size={11} className="mr-0.5 inline text-accent" />{r.held.map((s) => lab(s)).join(", ")}</> : "—"}
                  {r.wq != null && <span className="text-warn"> · waits for {lab(r.wq)}{r.wf ? ` (R${r.wf})` : ""}</span>}
                  {r.wq == null && r.wf != null && <span className="text-warn"> · blocked by R{r.wf}</span>}
                </span>
              </div>
            </button>
          </li>
        );
      })}
    </ul>
  );
}

/* ------------------------------------------------------------------ locks */
function LockTable({ sections }: { sections: SectionView[] }) {
  const rows = useMemo(() => {
    const rank = (s: SectionView) => (s.blocked ? 0 : s.state === "unknown" ? 1 : s.state === "held" ? 2 : s.waiters.length ? 3 : 4);
    return [...sections].sort((a, b) => rank(a) - rank(b) || a.sid - b.sid);
  }, [sections]);
  const busy = rows.filter((s) => s.state !== "free" || s.blocked || s.waiters.length).length;
  return (
    <div className="p-2.5">
      {busy === 0 && <Empty icon={<Unlock size={18} />} text="All aisles free" sub="No robot holds or waits for a critical section." />}
      <table className="w-full text-xs">
        <caption className="sr-only">Critical sections and their Ricart-Agrawala lock state</caption>
        <thead>
          <tr className="text-left text-2xs uppercase tracking-wide text-muted">
            <th className="px-2 py-1.5 font-medium">Aisle</th>
            <th className="px-2 py-1.5 font-medium">State</th>
            <th className="px-2 py-1.5 font-medium">Holder</th>
            <th className="px-2 py-1.5 text-right font-medium">Lease</th>
            <th className="px-2 py-1.5 font-medium">Waiters</th>
          </tr>
        </thead>
        <tbody>
          {rows.map((s) => {
            const free = s.state === "free" && !s.blocked && !s.waiters.length;
            const badge = s.blocked
              ? { c: "bg-crit-soft text-crit", t: "✕ Blocked" }
              : s.state === "unknown" ? { c: "bg-panel-2 text-unknown", t: "? Unknown" }
                : s.state === "held" ? { c: "bg-accent-soft text-accent", t: "● Held" }
                  : { c: "bg-ok-soft text-ok", t: "○ Free" };
            return (
              <tr key={s.sid} className={`border-t border-line ${free ? "opacity-60" : ""}`}>
                <td className="num px-2 py-1.5 text-fg">{s.label}</td>
                <td className="px-2 py-1.5"><span className={`rounded-[var(--radius-chip)] px-1.5 py-0.5 text-2xs font-semibold ${badge.c}`}>{badge.t}</span></td>
                <td className="num px-2 py-1.5">
                  {s.holder != null ? (
                    <span className="flex items-center gap-1.5"><span className="inline-block h-2 w-2 rounded-full" style={{ background: robotColor(s.holder) }} />R{s.holder}</span>
                  ) : <span className="text-muted">—</span>}
                </td>
                <td className="num px-2 py-1.5 text-right">{s.lease != null ? `${s.lease.toFixed(1)} s` : <span className="text-muted">—</span>}</td>
                <td className="num px-2 py-1.5">
                  {s.waiters.length ? <span className="text-warn">⏳ {s.waiters.map((w) => `R${w}`).join(" → ")}</span> : <span className="text-muted">—</span>}
                </td>
              </tr>
            );
          })}
        </tbody>
      </table>
    </div>
  );
}

/* ------------------------------------------------------------------ alerts */
const SEV_ICON: Record<Severity, React.ReactNode> = {
  crit: <AlertOctagon size={15} className="text-crit" aria-label="Critical" />,
  warn: <AlertTriangle size={15} className="text-warn" aria-label="Warning" />,
  info: <Info size={15} className="text-accent" aria-label="Info" />,
};

function AlertFeed({ alerts, onLocate }: { alerts: Alert[]; onLocate: (x: number, y: number) => void }) {
  const [filter, setFilter] = useState<"all" | Severity>("all");
  const counts = { all: alerts.length, crit: 0, warn: 0, info: 0 } as Record<string, number>;
  for (const a of alerts) counts[a.sev] += 1;
  const shown = useMemo(() => [...alerts].reverse().filter((a) => filter === "all" || a.sev === filter).slice(0, 150),
    [alerts, filter]);
  const opts: { id: "all" | Severity; label: string }[] = [
    { id: "all", label: "All" }, { id: "crit", label: "Critical" }, { id: "warn", label: "Warning" }, { id: "info", label: "Info" },
  ];
  return (
    <div className="flex h-full flex-col">
      <div className="flex gap-1 border-b border-line p-2" role="group" aria-label="Filter alerts by severity">
        {opts.map((o) => (
          <button key={o.id} type="button" aria-pressed={filter === o.id} onClick={() => setFilter(o.id)}
                  className={`rounded-[var(--radius-chip)] px-2.5 py-1 text-xs font-medium ${filter === o.id ? "bg-accent-soft text-accent" : "text-muted hover:bg-panel-2 hover:text-fg"}`}>
            {o.label} <span className="num opacity-80">{counts[o.id]}</span>
          </button>
        ))}
      </div>
      {shown.length === 0 ? (
        <Empty icon={<CheckCircle2 size={18} className="text-ok" />} text="No alerts: fleet healthy"
               sub={filter === "all" ? "Deadlocks, blockages, lost robots and partitions will appear here." : "Nothing at this severity."} />
      ) : (
        <ul className="divide-y divide-line" data-testid="alert-list">
          {shown.map((a) => {
            const loc = a.x != null && a.y != null;
            return (
              <li key={a.id} data-alert-type={a.type}>
                <button type="button" disabled={!loc} onClick={() => loc && onLocate(a.x!, a.y!)}
                        aria-label={`${a.sev} alert: ${a.msg}${loc ? ". Show on map" : ""}`}
                        className={`flex w-full gap-2.5 px-3 py-2 text-left text-xs ${loc ? "hover:bg-panel-2" : "cursor-default"}`}>
                  <span className="mt-0.5 shrink-0">{SEV_ICON[a.sev]}</span>
                  <span className="min-w-0 flex-1">
                    <span className="block text-fg">{a.msg}</span>
                    <span className="mt-1 flex flex-wrap items-center gap-1">
                      {a.robots.slice(0, 8).map((r) => (
                        <span key={r} className="num flex items-center gap-1 rounded bg-panel-2 px-1 text-2xs text-muted">
                          <span className="inline-block h-1.5 w-1.5 rounded-full" style={{ background: robotColor(r) }} />R{r}
                        </span>
                      ))}
                      {loc && <span className="ml-1 flex items-center gap-0.5 text-2xs text-accent"><LocateFixed size={11} />locate</span>}
                    </span>
                  </span>
                  <span className="num shrink-0 text-2xs text-muted">{fmtClock(a.t)}</span>
                </button>
              </li>
            );
          })}
        </ul>
      )}
    </div>
  );
}

function Empty({ icon, text, sub }: { icon: React.ReactNode; text: string; sub?: string }) {
  return (
    <div className="flex flex-col items-center gap-1.5 px-6 py-8 text-center">
      <span className="text-muted">{icon}</span>
      <div className="text-sm font-medium text-fg">{text}</div>
      {sub && <div className="text-xs text-muted">{sub}</div>}
    </div>
  );
}
