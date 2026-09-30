import { useCallback, useEffect, useState } from "react";
import { CheckCircle2, Play, PlugZap, RotateCcw, ServerOff, Shuffle } from "lucide-react";
import { useFleet } from "./useFleet";
import { isReplayMode, useReplay } from "./useReplay";
import { useLiveAvailable } from "./useLiveAvailable";
import { TopBar } from "./components/TopBar";
import { KpiStrip } from "./components/KpiStrip";
import { MapView, type Toggles } from "./components/MapView";
import { SidePanel } from "./components/SidePanel";
import { Sidebar } from "./components/Sidebar";
import { WelcomeCard } from "./components/WelcomeCard";

const REPLAY = isReplayMode();

function readPref<T extends string>(k: string, d: T): T {
  try {
    return (localStorage.getItem(k) as T) || d;
  } catch {
    return d;
  }
}
function writePref(k: string, v: string) {
  try {
    localStorage.setItem(k, v);
  } catch {
    /* private mode etc. */
  }
}

export default function App() {
  const liveAvailable = useLiveAvailable();
  const q = new URLSearchParams(window.location.search);
  const forceLive = q.has("live");
  const forceReplay = q.has("replay");
  const [useLive, setUseLive] = useState(false);

  useEffect(() => {
    if (forceLive) setUseLive(true);
    else if (!forceReplay && liveAvailable) setUseLive(true);
  }, [liveAvailable, forceLive, forceReplay]);

  const live = useFleet(!REPLAY && useLive);
  const rec = useReplay(REPLAY || !useLive);
  const fleet = useLive ? live : rec;
  const [theme, setTheme] = useState<"dark" | "light">(() => readPref("amr-theme", "light"));
  const [presenter, setPresenter] = useState(false);
  const [selected, setSelected] = useState<number | null>(null);
  const [toggles, setToggles] = useState<Toggles>({ paths: true, safety: false, lidar: false, tags: true, labels: true });
  const [focus, setFocus] = useState<{ x: number; y: number; n: number } | null>(null);
  const [fitSignal, setFitSignal] = useState(0);

  useEffect(() => {
    document.documentElement.dataset.theme = theme;
    writePref("amr-theme", theme);
  }, [theme]);
  useEffect(() => {
    document.documentElement.classList.toggle("presenter", presenter);
  }, [presenter]);

  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      const t = e.target as HTMLElement;
      if (t && (t.tagName === "INPUT" || t.tagName === "TEXTAREA" || t.tagName === "SELECT")) return;
      if (e.key === "p" || e.key === "P") setPresenter((v) => !v);
      else if (e.key === "f" || e.key === "F") setFitSignal((n) => n + 1);
      else if (e.key === "Escape") setSelected(null);
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, []);

  const onLocate = useCallback((x: number, y: number) => setFocus((f) => ({ x, y, n: (f?.n ?? 0) + 1 })), []);

  const { status, hello, state, alerts } = fleet;
  const r = (REPLAY || !useLive) ? rec.replay : null;
  const showWelcome = !!r && r.list.length > 0 && !r.started && !r.id;

  const sidebar = (
    <Sidebar replay={r} scenario={hello?.scenario.name ?? null} toggles={toggles} setToggles={setToggles}
             theme={theme} onTheme={() => setTheme(theme === "dark" ? "light" : "dark")}
             presenter={presenter} onPresenter={() => setPresenter(!presenter)} />
  );
  const topBar = (
    <TopBar status={status} scenario={hello?.scenario.name ?? null} simTime={hello ? state?.t ?? null : null}
            faults={state?.faults ?? null} url={fleet.url} attempt={fleet.attempt} />
  );

  if (!hello && !showWelcome) {
    return (
      <div className="flex h-full flex-col">
        {topBar}
        <main className="grid flex-1 place-items-center p-8" aria-busy="true">
          <div className="max-w-md rounded-[var(--radius-card)] border border-line bg-panel p-6 text-center" data-testid="connecting">
            <PlugZap size={28} className="mx-auto mb-3 text-accent" />
            <h1 className="text-lg font-semibold text-fg">{!useLive ? "Loading recorded runs…" : "Connecting to the fleet bridge…"}</h1>
            <p className="mt-2 text-sm text-muted">
              {!useLive ? "Fetching the scenario library."
                : <>Listening for <span className="num">{fleet.url}</span> (attempt {fleet.attempt}). The console is read-only:
                  the robots coordinate peer-to-peer and never depend on it.</>}
            </p>
            {useLive && <p className="mt-3 text-xs text-muted">Start the system with <span className="num text-fg">scripts\run_demo.cmd</span></p>}
          </div>
        </main>
      </div>
    );
  }

  if (showWelcome && r) {
    const mapData = hello?.map || rec.hello?.map;
    if (!mapData) {
      return (
        <div className="flex h-full flex-col bg-bg">
          <TopBar status="connecting" scenario={null} simTime={null} faults={null} url="" attempt={0} />
          <main className="grid flex-1 place-items-center p-8">
            <div className="text-center">
              <PlugZap size={28} className="mx-auto mb-3 text-accent" />
              <h1 className="text-lg font-semibold text-fg">Loading scenario library…</h1>
            </div>
          </main>
        </div>
      );
    }
    return (
      <div className="flex h-full flex-col bg-bg">
        <TopBar status="connecting" scenario={null} simTime={null} faults={null} url="" attempt={0} />
        <main className="relative flex-1">
          <MapView map={mapData} state={null} selected={null} onSelect={() => {}}
                   toggles={toggles} focus={null} fitSignal={fitSignal} frozen={true} />
          <WelcomeCard replay={r} liveAvailable={liveAvailable}
                       onSelectReplay={(id) => r.select(id)}
                       onStartLive={() => setUseLive(true)} />
        </main>
      </div>
    );
  }

  const disconnected = status !== "live" && status !== "replay";
  const preStart = !!r && !r.started;        // replay home page: first frame, dimmed, Start card
  const nextScenario = () => {
    if (!r || !r.list.length) return;
    const i = r.list.findIndex((x) => x.id === r.id);
    r.select(r.list[(i + 1) % r.list.length].id);
  };
  if (!hello) return null;

  return (
    <div className="grid h-full grid-rows-[auto_minmax(0,1fr)]">
      {topBar}
      <div className="flex min-h-0">
        {sidebar}
        <div className="grid min-h-0 min-w-0 flex-1 grid-rows-[auto_minmax(0,1fr)]">
          <KpiStrip kpi={preStart ? null : state?.kpi ?? null} />
          <main className="grid min-h-0 grid-cols-[minmax(0,1fr)_340px] 2xl:grid-cols-[minmax(0,1fr)_400px]">
            <div className="relative min-h-0 min-w-0">
              <MapView map={hello.map} state={state} selected={selected} onSelect={setSelected}
                       toggles={preStart ? { ...toggles, tags: false, labels: false } : toggles}
                       focus={focus} fitSignal={fitSignal} frozen={disconnected || preStart} />
              {disconnected && (
                <div role="alert" data-testid="disconnected-banner"
                     className="absolute left-1/2 top-14 z-30 flex max-w-[92%] -translate-x-1/2 items-center gap-3 rounded-[var(--radius-card)] border border-warn/60 bg-panel px-4 py-2.5 text-sm shadow-lg">
                  <ServerOff size={18} className="shrink-0 text-warn" />
                  <span>
                    <span className="font-semibold text-warn">Bridge disconnected — fleet unaffected.</span>{" "}
                    <span className="text-muted">Robots keep coordinating peer-to-peer. Reconnecting automatically (try {fleet.attempt})…</span>
                  </span>
                </div>
              )}
              {preStart && r && (
                <div className="absolute inset-0 top-11 z-20 grid place-items-center p-6">
                  <div className="w-[min(480px,92%)] rounded-[var(--radius-card)] border border-line bg-panel p-8 text-center shadow-lg" data-testid="start-card">
                    <h1 className="text-xl font-semibold text-fg">Simulation (Demo)</h1>
                    <p className="mt-3 text-sm leading-relaxed text-muted">
                      A fleet of 5 autonomous mobile robots shares one warehouse. Each robot plans its own route, negotiates narrow aisles with its peers and resolves deadlocks, with no central server. Watch positions, planned paths, aisle locks, battery and alerts in real time.
                    </p>
                    <p className="mt-3 text-xs text-muted">
                      Demo mode runs by default. Switch modes anytime from the Simulation options.
                    </p>
                    <button type="button" onClick={r.start} autoFocus
                            className="mx-auto mt-6 flex h-11 items-center gap-2 rounded-[var(--radius-card)] bg-accent px-6 text-base font-semibold text-white hover:opacity-90">
                      <Play size={18} /> Start simulation
                    </button>
                    <p className="mt-4 text-2xs text-muted">Runs are captured from the real multi-robot simulation.</p>
                  </div>
                </div>
              )}
              {r && r.ended && (
                <div className="absolute bottom-4 left-1/2 z-20 -translate-x-1/2" data-testid="complete-card">
                  <div className="flex items-center gap-3 rounded-[var(--radius-card)] border border-line bg-panel px-4 py-3 shadow-lg">
                    <CheckCircle2 size={18} className="shrink-0 text-ok" />
                    <span className="text-sm font-semibold text-fg">Run complete</span>
                    <button type="button" onClick={r.restart}
                            className="flex h-8 items-center gap-1.5 rounded-[var(--radius-chip)] border border-line px-3 text-sm text-fg hover:bg-panel-2">
                      <RotateCcw size={14} /> Restart
                    </button>
                    <button type="button" onClick={nextScenario}
                            className="flex h-8 items-center gap-1.5 rounded-[var(--radius-chip)] bg-accent px-3 text-sm font-semibold text-white hover:opacity-90">
                      <Shuffle size={14} /> Try another scenario
                    </button>
                  </div>
                </div>
              )}
            </div>
            {preStart ? (
              <aside className="flex min-h-0 flex-col items-center justify-center gap-2 border-l border-line bg-panel p-6 text-center" aria-label="Fleet details">
                <div className="text-sm font-medium text-fg">Robots, locks and alerts</div>
                <p className="text-xs text-muted">appear here with live values once the run starts.</p>
              </aside>
            ) : (
              <SidePanel robots={state?.robots ?? []} sections={state?.sections ?? []} alerts={alerts}
                         cycles={state?.cycles ?? []} selected={selected} onSelect={setSelected} onLocate={onLocate} />
            )}
          </main>
        </div>
      </div>
    </div>
  );
}
