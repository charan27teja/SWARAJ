import { useState } from "react";
import { ChevronsLeft, ChevronsRight, Info, Moon, Pause, Play, Presentation, RotateCcw, SlidersHorizontal, Sun, Map as MapIcon } from "lucide-react";
import type { ReplayCtl } from "../useReplay";
import type { Toggles } from "./MapView";
import { LegendGrid } from "./Legend";
import { fmtClock } from "../lib/ui";

interface Props {
  replay: ReplayCtl | null;             // null in live mode
  scenario: string | null;
  toggles: Toggles;
  setToggles: (t: Toggles) => void;
  theme: "dark" | "light";
  onTheme: () => void;
  presenter: boolean;
  onPresenter: () => void;
}

function Section({ title, children }: { title: string; children: React.ReactNode }) {
  return (
    <section className="border-b border-line px-4 py-3">
      <h2 className="mb-2 text-2xs font-semibold uppercase tracking-wide text-muted">{title}</h2>
      {children}
    </section>
  );
}

function Switch({ label, on, onClick }: { label: string; on: boolean; onClick: () => void }) {
  return (
    <button type="button" role="switch" aria-checked={on} onClick={onClick}
            className="flex w-full items-center justify-between rounded-[var(--radius-chip)] px-1 py-1 text-sm text-fg hover:bg-panel-2">
      <span>{label}</span>
      <span className={`relative h-4 w-7 rounded-full ${on ? "bg-accent" : "bg-line"}`} aria-hidden="true">
        <span className={`absolute top-0.5 h-3 w-3 rounded-full bg-white ${on ? "left-3.5" : "left-0.5"}`} />
      </span>
    </button>
  );
}

/** Left control sidebar. It controls the replay player and the view only — never the robots. */
export function Sidebar(p: Props) {
  const [open, setOpen] = useState(true);
  const [legendOpen, setLegendOpen] = useState(() => window.innerHeight >= 900);
  const r = p.replay;
  if (!open) {
    return (
      <nav className="flex w-12 flex-col items-center gap-3 border-r border-line bg-panel py-3" aria-label="Controls (collapsed)">
        <button type="button" onClick={() => setOpen(true)} aria-label="Expand sidebar" title="Expand sidebar"
                className="grid h-8 w-8 place-items-center rounded-[var(--radius-chip)] text-muted hover:bg-panel-2 hover:text-fg">
          <ChevronsRight size={16} />
        </button>
        {r && (
          <button type="button" onClick={() => (r.started ? r.setPlaying(!r.playing) : r.start())}
                  aria-label={r.playing ? "Pause" : "Start / resume"} title={r.playing ? "Pause" : "Start / resume"}
                  className="grid h-8 w-8 place-items-center rounded-[var(--radius-chip)] text-fg hover:bg-panel-2">
            {r.playing ? <Pause size={15} /> : <Play size={15} />}
          </button>
        )}
        <span className="text-muted" title="Display"><SlidersHorizontal size={15} /></span>
        <span className="text-muted" title="Legend"><MapIcon size={15} /></span>
      </nav>
    );
  }
  const item = r?.list.find((i) => i.id === r.id);
  return (
    <nav className="scroll-thin flex w-[280px] shrink-0 flex-col overflow-y-auto border-r border-line bg-panel" aria-label="Controls">
      <div className="flex items-center justify-between px-4 pt-3">
        <span className="text-sm font-semibold text-fg">Controls</span>
        <button type="button" onClick={() => setOpen(false)} aria-label="Collapse sidebar" title="Collapse sidebar"
                className="grid h-7 w-7 place-items-center rounded-[var(--radius-chip)] text-muted hover:bg-panel-2 hover:text-fg">
          <ChevronsLeft size={16} />
        </button>
      </div>

      <Section title="Simulation">
        {r ? (
          <div className="space-y-3" data-testid="sim-controls">
            <div>
              <select aria-label="Scenario" value={r.id ?? ""} onChange={(e) => r.select(e.target.value)}
                      className="h-8 w-full rounded-[var(--radius-chip)] border border-line bg-panel-2 px-2 text-sm text-fg">
                {r.list.map((i) => <option key={i.id} value={i.id}>{i.title}</option>)}
              </select>
              {item && <p className="mt-1.5 text-xs leading-snug text-muted">{item.desc}</p>}
            </div>
            <div className="flex overflow-hidden rounded-[var(--radius-chip)] border border-line text-xs" role="group" aria-label="Playback speed">
              {[1, 2, 4].map((s) => (
                <button key={s} type="button" aria-pressed={r.speed === s} onClick={() => r.setSpeed(s)}
                        className={`h-8 flex-1 ${r.speed === s ? "bg-accent-soft font-semibold text-accent" : "text-muted hover:bg-panel-2 hover:text-fg"}`}>
                  {s}×
                </button>
              ))}
            </div>
            {!r.started ? (
              <button type="button" onClick={r.start} data-testid="sidebar-start"
                      className="flex h-9 w-full items-center justify-center gap-2 rounded-[var(--radius-chip)] bg-accent text-sm font-semibold text-white hover:opacity-90">
                <Play size={15} /> Start
              </button>
            ) : (
              <div className="flex gap-2">
                <button type="button" onClick={() => r.setPlaying(!r.playing)} disabled={r.ended}
                        data-testid="replay-play" aria-label={r.playing ? "Pause" : "Resume"}
                        className="flex h-8 flex-1 items-center justify-center gap-1.5 rounded-[var(--radius-chip)] border border-line text-sm text-fg hover:bg-panel-2 disabled:opacity-40">
                  {r.playing ? <><Pause size={14} /> Pause</> : <><Play size={14} /> Resume</>}
                </button>
                <button type="button" onClick={r.restart} aria-label="Restart"
                        className="flex h-8 flex-1 items-center justify-center gap-1.5 rounded-[var(--radius-chip)] border border-line text-sm text-fg hover:bg-panel-2">
                  <RotateCcw size={14} /> Restart
                </button>
              </div>
            )}
            <div>
              <input type="range" min={0} max={r.duration || 0} step={0.1} value={r.t} aria-label="Playback position"
                     disabled={!r.started} onChange={(e) => r.seek(Number(e.target.value))}
                     className="w-full accent-[var(--c-accent)] disabled:opacity-40" />
              <div className="num flex justify-between text-2xs text-muted">
                <span>{fmtClock(r.t)}</span><span>{fmtClock(r.duration)}</span>
              </div>
            </div>
          </div>
        ) : (
          <div className="space-y-1 text-sm" data-testid="sim-live">
            <div className="text-fg">Scenario <span className="font-semibold">{p.scenario ?? "—"}</span></div>
            <p className="text-xs leading-snug text-muted">
              Live fleet, started by <span className="num">scripts\run_demo.cmd</span>. This console is read-only.
            </p>
          </div>
        )}
      </Section>

      <Section title="Display">
        <div className="space-y-0.5">
          <Switch label="Planned paths" on={p.toggles.paths} onClick={() => p.setToggles({ ...p.toggles, paths: !p.toggles.paths })} />
          <Switch label="Safety circles" on={p.toggles.safety} onClick={() => p.setToggles({ ...p.toggles, safety: !p.toggles.safety })} />
          <Switch label="Lidar rays" on={p.toggles.lidar} onClick={() => p.setToggles({ ...p.toggles, lidar: !p.toggles.lidar })} />
          <Switch label="Lock tags" on={p.toggles.tags} onClick={() => p.setToggles({ ...p.toggles, tags: !p.toggles.tags })} />
          <Switch label="Robot labels" on={p.toggles.labels} onClick={() => p.setToggles({ ...p.toggles, labels: !p.toggles.labels })} />
        </div>
        <div className="mt-2 flex gap-2">
          <button type="button" onClick={p.onTheme} aria-label={`Switch to ${p.theme === "dark" ? "light" : "dark"} theme`}
                  className="flex h-8 flex-1 items-center justify-center gap-1.5 rounded-[var(--radius-chip)] border border-line text-xs text-muted hover:bg-panel-2 hover:text-fg">
            {p.theme === "dark" ? <Sun size={14} /> : <Moon size={14} />} {p.theme === "dark" ? "Light" : "Dark"}
          </button>
          <button type="button" onClick={p.onPresenter} aria-pressed={p.presenter} aria-label="Presenter mode (P)"
                  className={`flex h-8 flex-1 items-center justify-center gap-1.5 rounded-[var(--radius-chip)] border border-line text-xs ${p.presenter ? "bg-accent-soft text-accent" : "text-muted hover:bg-panel-2 hover:text-fg"}`}>
            <Presentation size={14} /> Presenter
          </button>
        </div>
      </Section>

      <section className="border-b border-line px-4 py-3">
        <button type="button" onClick={() => setLegendOpen(!legendOpen)} aria-expanded={legendOpen}
                className="flex w-full items-center justify-between text-2xs font-semibold uppercase tracking-wide text-muted">
          Legend <span aria-hidden="true">{legendOpen ? "–" : "+"}</span>
        </button>
        {legendOpen && <div className="mt-2"><LegendGrid /></div>}
      </section>

      <section className="px-4 py-3 text-xs leading-snug text-muted">
        <div className="mb-1 flex items-center gap-1.5 font-semibold text-fg"><Info size={13} /> About</div>
        <p>Decentralised AMR fleet · 5 robots · no central server</p>
        <p>{r ? "Recorded runs of the real system." : "Live run of the real system."}</p>
      </section>
    </nav>
  );
}
