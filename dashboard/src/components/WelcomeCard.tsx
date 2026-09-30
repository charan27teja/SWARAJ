import { Play, Wifi } from "lucide-react";
import type { ReplayCtl } from "../useReplay";

interface Props {
  replay: ReplayCtl | null;
  liveAvailable: boolean;
  onSelectReplay: (id: string) => void;
  onStartLive: () => void;
}

export function WelcomeCard({ replay, liveAvailable, onSelectReplay, onStartLive }: Props) {
  const scenarios = [
    { id: "demo", title: "Demo", desc: "Watch robots coordinate across overlapping paths" },
    { id: "circular_wait", title: "Circular Wait", desc: "See deadlock detection and resolution at a 4-way intersection" },
    { id: "blockage", title: "Blockage", desc: "Observe re-routing when an aisle becomes blocked" },
    { id: "node_loss", title: "Node Loss", desc: "Fleet adapts when a robot process is killed mid-run" },
  ];

  const selectedScenario = replay && scenarios.find((s) => s.id === replay.id);

  return (
    <div className="absolute inset-0 z-30 grid place-items-center bg-black/20 p-6" data-testid="welcome-card">
      <div className="w-full max-w-2xl rounded-[var(--radius-card)] border border-line bg-panel p-8 shadow-xl">
        <div className="mb-2 flex items-center gap-2">
          <svg width="32" height="32" viewBox="0 0 24 24" aria-hidden="true" className="text-accent">
            <circle cx="8" cy="8" r="1.5" fill="currentColor" />
            <circle cx="16" cy="8" r="1.5" fill="currentColor" />
            <circle cx="12" cy="16" r="1.5" fill="currentColor" />
            <path d="M8 8 L12 16 M16 8 L12 16" stroke="currentColor" strokeWidth="1.5" fill="none" strokeLinecap="round" />
          </svg>
          <h1 className="text-2xl font-bold text-fg">SWARAJ</h1>
        </div>
        <h2 className="mb-3 text-lg font-semibold text-fg">AMR Fleet Simulation & Monitoring</h2>
        <p className="mb-6 text-sm leading-relaxed text-muted">
          Watch a fleet of 5 autonomous mobile robots coordinate peer-to-peer: live positions, planned paths, aisle locks, deadlock resolution, battery and alerts.
        </p>

        {liveAvailable && (
          <button type="button" onClick={onStartLive}
                  className="mb-6 flex w-full items-center justify-center gap-2 rounded-[var(--radius-chip)] bg-ok px-4 py-2 text-sm font-semibold text-white hover:opacity-90">
            <Wifi size={16} /> Live fleet detected — Connect
          </button>
        )}

        <div className="mb-6">
          <label className="mb-3 block text-xs font-semibold uppercase tracking-wide text-muted">Choose a scenario</label>
          <div className="grid gap-2 sm:grid-cols-2">
            {scenarios.map((s) => (
              <button key={s.id} type="button" onClick={() => onSelectReplay(s.id)}
                      className={`flex flex-col items-start rounded-[var(--radius-chip)] border px-3 py-2 text-left text-xs transition-colors ${
                        selectedScenario?.id === s.id
                          ? "border-accent bg-accent-soft text-fg"
                          : "border-line bg-panel hover:bg-panel-2 text-fg"
                      }`}>
                <div className="font-semibold">{s.title}</div>
                <div className="text-2xs text-muted">{s.desc}</div>
              </button>
            ))}
          </div>
        </div>

        <button type="button" onClick={() => selectedScenario && replay?.start()} disabled={!selectedScenario}
                className="flex w-full items-center justify-center gap-2 rounded-[var(--radius-chip)] bg-accent px-4 py-3 text-sm font-semibold text-white hover:opacity-90 disabled:opacity-40">
          <Play size={18} /> Start simulation
        </button>
      </div>
    </div>
  );
}
