import { useEffect, useMemo, useRef, useState } from "react";
import { GitBranch } from "lucide-react";
import type { Robot } from "../types";
import { robotColor } from "../lib/ui";

/** Wait-for graph mini-view built from the `wf` field every robot puts in its beacon.
 *  Cycles (deadlocks) are drawn in red and named in text. */
export function WaitForGraph({ robots, cycles }: { robots: Robot[]; cycles: number[][] }) {
  const box = useRef<HTMLDivElement>(null);
  const [dim, setDim] = useState({ w: 360, h: 150 });
  useEffect(() => {
    const el = box.current;
    if (!el) return;
    const ro = new ResizeObserver(() => setDim({ w: Math.max(200, el.clientWidth), h: Math.max(110, el.clientHeight) }));
    ro.observe(el);
    return () => ro.disconnect();
  }, []);
  const W = dim.w, H = dim.h;
  const live = robots.filter((r) => !r.stale);
  const edges = live.filter((r) => r.wf != null && robots.some((q) => q.id === r.wf)).map((r) => [r.id, r.wf as number]);
  const cycleEdges = useMemo(() => {
    const s = new Set<string>();
    for (const c of cycles) for (let i = 0; i < c.length; i++) s.add(`${c[i]}>${c[(i + 1) % c.length]}`);
    return s;
  }, [cycles]);
  const inCycle = new Set(cycles.flat());
  const nodes = robots;
  const pos = new Map<number, [number, number]>();
  // Reserve space at the bottom for caption (16px) and margin
  const graphHeight = Math.max(110, H - 28);
  const nodeR = Math.max(12, Math.min(24, Math.min(W, graphHeight) / (nodes.length > 8 ? 12 : 8)));
  const cx = W / 2, cy = graphHeight / 2;
  // nodes live strictly inside the drawing area (arrow heads + labels included)
  const rx = Math.max(10, W / 2 - nodeR - 16), ry = Math.max(8, graphHeight / 2 - nodeR - 6);
  nodes.forEach((r, i) => {
    const a = -Math.PI / 2 + (2 * Math.PI * i) / Math.max(1, nodes.length);
    pos.set(r.id, [cx + Math.min(rx, ry * 1.6) * Math.cos(a), cy + ry * Math.sin(a)]);
  });
  return (
    <div className="flex min-h-0 flex-1 flex-col px-3 pb-2 pt-2" data-testid="waitfor">
      <div className="mb-1 flex items-center justify-between">
        <span className="flex items-center gap-1.5 text-xs font-semibold text-fg"><GitBranch size={13} className="text-muted" />Wait-for graph</span>
        {cycles.length > 0 ? (
          <span className="rounded-[var(--radius-chip)] bg-crit-soft px-1.5 py-0.5 text-2xs font-semibold text-crit" data-testid="cycle-label">
            ⟳ Cycle: {cycles.map((c) => [...c, c[0]].map((r) => `R${r}`).join("→")).join("; ")}
          </span>
        ) : edges.length ? (
          <span className="text-2xs text-muted">{edges.length} waiting, no cycle</span>
        ) : (
          <span className="text-2xs text-muted">No robot waits on another</span>
        )}
      </div>
      <div ref={box} className="relative min-h-0 flex-1 overflow-hidden">
      <svg viewBox={`0 0 ${W} ${graphHeight}`} width={W} height={graphHeight} role="img" className="absolute inset-0"
           aria-label={cycles.length ? `Deadlock cycle ${cycles[0].join(" to ")}` : `${edges.length} wait-for edges`}>
        <defs>
          <marker id="wf-arrow" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="6" markerHeight="6" orient="auto">
            <path d="M0 0L10 5L0 10z" fill="var(--c-muted)" />
          </marker>
          <marker id="wf-arrow-red" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="6" markerHeight="6" orient="auto">
            <path d="M0 0L10 5L0 10z" fill="var(--c-crit)" />
          </marker>
        </defs>
        {edges.map(([a, b]) => {
          const p = pos.get(a), q = pos.get(b);
          if (!p || !q) return null;
          const dx = q[0] - p[0], dy = q[1] - p[1];
          const d = Math.hypot(dx, dy) || 1;
          const ux = dx / d, uy = dy / d;
          const red = cycleEdges.has(`${a}>${b}`);
          return (
            <line key={`${a}-${b}`} x1={p[0] + ux * (nodeR + 1)} y1={p[1] + uy * (nodeR + 1)}
                  x2={q[0] - ux * (nodeR + 3)} y2={q[1] - uy * (nodeR + 3)}
                  stroke={red ? "var(--c-crit)" : "var(--c-muted)"} strokeWidth={red ? 2.4 : 1.4}
                  markerEnd={red ? "url(#wf-arrow-red)" : "url(#wf-arrow)"} />
          );
        })}
        {nodes.map((r) => {
          const p = pos.get(r.id)!;
          const active = edges.some(([a, b]) => a === r.id || b === r.id);
          return (
            <g key={r.id} transform={`translate(${p[0]},${p[1]})`} opacity={active || inCycle.has(r.id) ? 1 : 0.6}>
              <circle r={nodeR} fill={r.stale ? "var(--c-unknown)" : robotColor(r.id)}
                      stroke={inCycle.has(r.id) ? "var(--c-crit)" : "var(--c-panel)"} strokeWidth={inCycle.has(r.id) ? 2.5 : 1.5} />
              <text y={nodeR * 0.36} fontSize={nodeR * 0.95} textAnchor="middle" fill="#0b0f16" fontWeight={700}
                    fontFamily="var(--font-mono)">{r.id}</text>
            </g>
          );
        })}
      </svg>
      </div>
      <div className="shrink-0 pt-1 text-center text-2xs leading-tight text-muted">A → B: A waits for B · red = deadlock cycle</div>
    </div>
  );
}
