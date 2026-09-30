import { memo, useCallback, useEffect, useMemo, useRef, useState } from "react";
import type { PointerEvent as RPointerEvent } from "react";
import { Maximize2, Minus, Plus, MapPin } from "lucide-react";
import type { FleetState, MapInfo, Robot, SectionView } from "../types";
import { MODE_LABEL, STAGE_LABEL, fmtAge, robotColor } from "../lib/ui";
import { layoutLabels, TAG_W, WAIT_W, TAG_H, type Pt } from "../lib/labels";

export interface Toggles {
  paths: boolean;
  safety: boolean;
  lidar: boolean;
  tags: boolean;
  labels: boolean;
}

interface Props {
  map: MapInfo;
  state: FleetState | null;
  selected: number | null;
  onSelect: (id: number | null) => void;
  toggles: Toggles;
  focus: { x: number; y: number; n: number } | null;
  fitSignal: number;
  frozen: boolean;
}

type Hover =
  | { kind: "robot"; id: number; sx: number; sy: number }
  | { kind: "section"; sid: number; sx: number; sy: number }
  | null;

interface View {
  k: number;
  tx: number;
  ty: number;
}

/* --------------------------------------------------------------- static layer (memoised) */
const StaticLayer = memo(function StaticLayer({ map }: { map: MapInfo }) {
  const racks = useMemo(() => {
    const out: { x: number; y: number; w: number }[] = [];
    map.rows.forEach((row, y) => {
      let x = 0;
      while (x < row.length) {
        if (row[x] === "#") {
          let e = x;
          while (e < row.length && row[e] === "#") e++;
          out.push({ x, y, w: e - x });
          x = e;
        } else x++;
      }
    });
    return out;
  }, [map]);
  const grid = useMemo(() => {
    const ls: string[] = [];
    for (let x = 1; x < map.width; x++) ls.push(`M${x} 0V${map.height}`);
    for (let y = 1; y < map.height; y++) ls.push(`M0 ${y}H${map.width}`);
    return ls.join("");
  }, [map]);
  return (
    <g>
      <rect x={0} y={0} width={map.width} height={map.height} fill="var(--m-floor)" />
      <path d={grid} stroke="var(--m-grid)" strokeWidth={0.02} fill="none" />
      {racks.map((r, i) => (
        <rect key={i} x={r.x} y={r.y} width={r.w} height={1} fill="var(--m-rack)" />
      ))}
      {map.docks.map(([x, y]) => (
        <g key={`c${x},${y}`} transform={`translate(${x + 0.5},${y + 0.5})`}>
          <rect x={-0.42} y={-0.42} width={0.84} height={0.84} rx={0.12} fill="none"
                stroke="var(--c-accent)" strokeWidth={0.05} />
          <path d="M0.05 -0.3 L-0.16 0.04 L0 0.04 L-0.05 0.3 L0.16 -0.06 L0 -0.06 Z" fill="var(--c-accent)" />
        </g>
      ))}
      {map.drops.map(([x, y]) => (
        <g key={`d${x},${y}`} transform={`translate(${x + 0.5},${y + 0.5})`}>
          <rect x={-0.42} y={-0.42} width={0.84} height={0.84} rx={0.12} fill="none"
                stroke="var(--c-muted)" strokeWidth={0.05} />
          <path d="M0 -0.26 V0.14 M-0.16 0 L0 0.18 L0.16 0 M-0.2 0.28 H0.2" stroke="var(--c-muted)"
                strokeWidth={0.06} fill="none" strokeLinecap="round" />
        </g>
      ))}
      {map.bays.map(([x, y]) => (
        <text key={`b${x},${y}`} x={x + 0.5} y={y + 0.64} fontSize={0.42} textAnchor="middle"
              fill="var(--c-muted)" opacity={0.6} fontWeight={600}>
          P
        </text>
      ))}
    </g>
  );
});

/* --------------------------------------------------------------- helpers */
function sectionBox(map: MapInfo, sid: number) {
  const cells = map.sections[sid].cells;
  let x0 = 1e9, y0 = 1e9, x1 = -1e9, y1 = -1e9;
  for (const [x, y] of cells) {
    x0 = Math.min(x0, x); y0 = Math.min(y0, y);
    x1 = Math.max(x1, x + 1); y1 = Math.max(y1, y + 1);
  }
  return { x0, y0, x1, y1 };
}

const RAY_ANG = Array.from({ length: 40 }, (_, i) => -Math.PI + (3 * i * 2 * Math.PI) / 120);

export function MapView(props: Props) {
  const { map, state, selected, onSelect, toggles, focus, fitSignal, frozen } = props;
  const ref = useRef<HTMLDivElement>(null);
  const [size, setSize] = useState({ w: 800, h: 600 });
  const [view, setView] = useState<View>({ k: 20, tx: 0, ty: 0 });
  const [hover, setHover] = useState<Hover>(null);
  const [mark, setMark] = useState<{ x: number; y: number } | null>(null);
  const drag = useRef<{ x: number; y: number; tx: number; ty: number; moved: boolean } | null>(null);
  const userMoved = useRef(false);

  const fit = useCallback(() => {
    const pad = 14;
    const k = Math.max(4, Math.min((size.w - 2 * pad) / map.width, (size.h - 2 * pad) / map.height));
    setView({ k, tx: (size.w - k * map.width) / 2, ty: (size.h - k * map.height) / 2 });
    userMoved.current = false;
  }, [size, map]);

  useEffect(() => {
    const el = ref.current;
    if (!el) return;
    const ro = new ResizeObserver(() => setSize({ w: el.clientWidth, h: el.clientHeight }));
    ro.observe(el);
    setSize({ w: el.clientWidth, h: el.clientHeight });
    return () => ro.disconnect();
  }, []);

  useEffect(() => {
    if (!userMoved.current) fit();
  }, [size, fit]);

  useEffect(() => {
    if (fitSignal) fit();
  }, [fitSignal]); // eslint-disable-line react-hooks/exhaustive-deps

  useEffect(() => {
    if (!focus) return;
    setView((v) => {
      const k = Math.max(v.k, (Math.min(size.w / map.width, size.h / map.height)) * 1.6);
      return { k, tx: size.w / 2 - focus.x * k, ty: size.h / 2 - focus.y * k };
    });
    userMoved.current = true;
    setMark({ x: focus.x, y: focus.y });
    const t = window.setTimeout(() => setMark(null), 4000);
    return () => window.clearTimeout(t);
  }, [focus]); // eslint-disable-line react-hooks/exhaustive-deps

  const clamp = (n: number, min: number, max: number) => Math.min(max, Math.max(min, n));

  const clampPan = (v: View) => {
    const maxPan = 80;
    const minTx = -v.k * map.width + maxPan;
    const maxTx = size.w - maxPan;
    const minTy = -v.k * map.height + maxPan;
    const maxTy = size.h - maxPan;
    return {
      ...v,
      tx: clamp(v.tx, minTx, maxTx),
      ty: clamp(v.ty, minTy, maxTy),
    };
  };

  const zoomAt = (factor: number, cx: number, cy: number) => {
    setView((v) => {
      const fitScale = Math.min(size.w / map.width, size.h / map.height);
      const minK = fitScale * 0.8;
      const maxK = fitScale * 8;
      const k = clamp(v.k * factor, minK, maxK);
      const f = k / v.k;
      const newView = { k, tx: cx - (cx - v.tx) * f, ty: cy - (cy - v.ty) * f };
      return clampPan(newView);
    });
    userMoved.current = true;
  };

  useEffect(() => {
    const el = ref.current;
    if (!el) return;
    const handleWheel = (e: WheelEvent) => {
      e.preventDefault();
      const deltaY_px = clamp(e.deltaY * (e.deltaMode === 1 ? 16 : e.deltaMode === 2 ? 400 : 1), -150, 150);
      const factor = Math.exp(-deltaY_px * 0.0015);
      const r = el.getBoundingClientRect();
      zoomAt(factor, e.clientX - r.left, e.clientY - r.top);
    };
    el.addEventListener("wheel", handleWheel, { passive: false });
    return () => el.removeEventListener("wheel", handleWheel);
  }, [size, map]);
  const onDown = (e: RPointerEvent) => {
    if (e.button !== 0) return;
    drag.current = { x: e.clientX, y: e.clientY, tx: view.tx, ty: view.ty, moved: false };
    (e.target as Element).setPointerCapture?.(e.pointerId);
  };
  const onMove = (e: RPointerEvent) => {
    const d = drag.current;
    if (!d) return;
    const dx = e.clientX - d.x, dy = e.clientY - d.y;
    if (Math.abs(dx) + Math.abs(dy) > 3) d.moved = true;
    if (d.moved) {
      setView((v) => clampPan({ ...v, tx: d.tx + dx, ty: d.ty + dy }));
      userMoved.current = true;
    }
  };
  const onUp = () => {
    const d = drag.current;
    drag.current = null;
    if (d && !d.moved) onSelect(null);
  };

  const robots = state?.robots ?? [];
  const sections = state?.sections ?? [];
  const byId = useMemo(() => new Map(robots.map((r) => [r.id, r])), [robots]);
  const inCycle = useMemo(() => new Set((state?.cycles ?? []).flat()), [state]);
  const localPos = (e: RPointerEvent) => {
    const r = ref.current!.getBoundingClientRect();
    return { sx: e.clientX - r.left, sy: e.clientY - r.top };
  };

  // robots and their labels keep a minimum on-screen size (physical circles stay to scale)
  const presenterOn = typeof document !== "undefined" && document.documentElement.classList.contains("presenter");
  const glyph = Math.min(2.0, Math.max(1, (presenterOn ? 34 : 26) / view.k));
  const place = useMemo(() => layoutLabels(map, sections, robots, glyph, toggles),
    [map, sections, robots, glyph, toggles]);
  const hoveredRobot = hover?.kind === "robot" ? byId.get(hover.id) : undefined;
  const hoveredSection = hover?.kind === "section" ? sections.find((s) => s.sid === hover.sid) : undefined;

  return (
    <div className="flex h-full min-h-0 flex-col">
      <div className="flex h-11 shrink-0 items-center gap-3 border-b border-line bg-panel px-3">
        <span className="text-sm font-semibold text-fg">Warehouse map</span>
        <span className="num hidden text-xs text-muted md:inline">{map.name} · {map.width} × {map.height} m</span>
        <div className="ml-auto flex items-center gap-2">
          <div className="flex overflow-hidden rounded-[var(--radius-chip)] border border-line">
            <IconBtn label="Zoom in" onClick={() => zoomAt(1.25, size.w / 2, size.h / 2)}><Plus size={16} /></IconBtn>
            <IconBtn label="Zoom out" onClick={() => zoomAt(0.8, size.w / 2, size.h / 2)}><Minus size={16} /></IconBtn>
            <IconBtn label="Fit map to screen (F)" onClick={fit}><Maximize2 size={15} /></IconBtn>
          </div>
        </div>
      </div>
    <div ref={ref} className="relative min-h-0 w-full flex-1 overflow-hidden select-none bg-bg"
         aria-label="Live warehouse map" role="region">
      <svg width={size.w} height={size.h} onPointerDown={onDown} onPointerMove={onMove}
           onPointerUp={onUp} onPointerLeave={() => { drag.current = null; }}
           className={drag.current?.moved ? "cursor-grabbing" : "cursor-grab"} role="img"
           aria-label={`Warehouse ${map.name}, ${robots.length} robots`}>
        <defs>
          <pattern id="hatch-grey" width="0.35" height="0.35" patternUnits="userSpaceOnUse"
                   patternTransform="rotate(45)">
            <rect width="0.35" height="0.35" fill="var(--m-unknown)" />
            <line x1="0" y1="0" x2="0" y2="0.35" stroke="var(--c-unknown)" strokeWidth="0.1" />
          </pattern>
          <pattern id="hatch-red" width="0.3" height="0.3" patternUnits="userSpaceOnUse"
                   patternTransform="rotate(-45)">
            <line x1="0" y1="0" x2="0" y2="0.3" stroke="var(--c-crit)" strokeWidth="0.09" />
          </pattern>
          <marker id="arrow" viewBox="0 0 10 10" refX="5" refY="5" markerWidth="4" markerHeight="4"
                  orient="auto-start-reverse">
            <path d="M0 0L10 5L0 10z" fill="var(--c-muted)" />
          </marker>
        </defs>
        <g transform={`translate(${view.tx},${view.ty}) scale(${view.k})`}
           opacity={frozen ? 0.55 : 1}>
          <StaticLayer map={map} />
          {/* critical sections coloured by lock state (text/pattern backups) */}
          {sections.map((s) => (
            <SectionShape key={s.sid} s={s} map={map} tagAt={place.tags.get(s.sid) ?? null}
                          waitAt={place.waits.get(s.sid) ?? null}
                          onEnter={(e) => setHover({ kind: "section", sid: s.sid, ...localPos(e) })}
                          onLeave={() => setHover(null)} />
          ))}
          {/* pick stations on top of aisle fill */}
          {map.picks.map(([x, y]) => (
            <rect key={`p${x},${y}`} x={x + 0.3} y={y + 0.3} width={0.4} height={0.4} rx={0.06}
                  fill="none" stroke="var(--m-label)" strokeWidth={0.05} opacity={0.55} />
          ))}
          {/* blockages known to the fleet (BlockageEvent, TTL) */}
          {state?.blocks.map((b) => (
            <g key={`blk${b.x},${b.y}`}>
              <rect x={b.x} y={b.y} width={1} height={1} fill="url(#hatch-red)" stroke="var(--c-crit)"
                    strokeWidth={0.06} />
              <text x={b.x + 0.5} y={b.y - 0.15} fontSize={0.34} textAnchor="middle" fill="var(--c-crit)"
                    fontWeight={700}>
                ✕ blocked {Math.max(0, b.ttl).toFixed(0)}s
              </text>
            </g>
          ))}
          {/* physical pallets (world ground truth) */}
          {state?.pallets.map((p) => (
            <rect key={`pal${p.id}`} x={p.x - p.half} y={p.y - p.half} width={2 * p.half} height={2 * p.half}
                  rx={0.06} fill="var(--m-pallet)" stroke="#5a3a18" strokeWidth={0.04} />
          ))}
          {/* planned paths */}
          {toggles.paths && robots.map((r) => (r.path.length > 0 && !r.stale) && (
            <polyline key={`path${r.id}`}
                      points={[[r.x, r.y], ...r.path.map(([x, y]) => [x + 0.5, y + 0.5])].map((p) => p.join(",")).join(" ")}
                      fill="none" stroke={robotColor(r.id)} strokeWidth={selected === r.id ? 0.14 : 0.08}
                      strokeLinejoin="round" strokeLinecap="round"
                      strokeDasharray={r.mode === "waiting" ? "0.25 0.18" : undefined}
                      opacity={selected == null ? 0.65 : selected === r.id ? 1 : 0.12} />
          ))}
          {/* lidar rays (world feed) */}
          {toggles.lidar && state?.lidar && robots.map((r) => {
            const rays = state.lidar?.[String(r.id)];
            if (!rays || r.stale) return null;
            const d = rays.map((rg, i) => {
              if (rg >= 7.9) return "";
              const a = r.th + RAY_ANG[i];
              return `M${r.x} ${r.y}L${r.x + rg * Math.cos(a)} ${r.y + rg * Math.sin(a)}`;
            }).join("");
            return <path key={`ray${r.id}`} d={d} stroke={robotColor(r.id)} strokeWidth={0.025} opacity={0.35} />;
          })}
          {/* robots */}
          {robots.map((r) => (
            <RobotShape key={r.id} r={r} gs={glyph} labelAt={place.labels.get(r.id) ?? null} selected={selected === r.id} dim={selected != null && selected !== r.id}
                        safety={toggles.safety} inCycle={inCycle.has(r.id)}
                        onEnter={(e) => setHover({ kind: "robot", id: r.id, ...localPos(e) })}
                        onLeave={() => setHover(null)}
                        onClick={() => onSelect(r.id)} />
          ))}
          {mark && (
            <g transform={`translate(${mark.x},${mark.y})`} pointerEvents="none">
              <circle r={1.1} fill="none" stroke="var(--c-accent)" strokeWidth={0.1} />
              <circle r={0.12} fill="var(--c-accent)" />
            </g>
          )}
        </g>
      </svg>

      {/* tooltips */}
      {hoveredRobot && hover && (
        <Tooltip x={hover.sx} y={hover.sy} w={size.w} h={size.h}>
          <RobotTip r={hoveredRobot} sections={sections} />
        </Tooltip>
      )}
      {hoveredSection && hover && (
        <Tooltip x={hover.sx} y={hover.sy} w={size.w} h={size.h}>
          <SectionTip s={hoveredSection} oneWay={map.sections[hoveredSection.sid].one_way} />
        </Tooltip>
      )}
      {mark && (
        <div className="pointer-events-none absolute left-1/2 top-3 -translate-x-1/2 rounded-full border border-line bg-panel px-3 py-1 text-xs text-muted shadow-sm">
          <MapPin size={12} className="mr-1 inline text-accent" />Alert location
        </div>
      )}
    </div>
    </div>
  );
}

/* --------------------------------------------------------------- sections */
function SectionShape({ s, map, tagAt, waitAt, onEnter, onLeave }: {
  s: SectionView; map: MapInfo; tagAt: Pt | null; waitAt: Pt | null;
  onEnter: (e: RPointerEvent) => void; onLeave: () => void;
}) {
  const b = sectionBox(map, s.sid);
  const w = b.x1 - b.x0, h = b.y1 - b.y0;
  let fill = "var(--m-free)", stroke = "var(--m-free-edge)";
  let tag: string | null = null;
  let tagColor = "var(--c-ok)";
  if (s.state === "held" && s.holder != null) {
    const c = robotColor(s.holder);
    fill = c; stroke = c;
    tag = `R${s.holder}`;
    tagColor = c;
  } else if (s.state === "unknown") {
    fill = "url(#hatch-grey)"; stroke = "var(--c-unknown)";
    tag = "? unknown";
    tagColor = "var(--c-unknown)";
  }
  const info = map.sections[s.sid];
  const cx = (b.x0 + b.x1) / 2;
  return (
    <g onPointerEnter={onEnter} onPointerLeave={onLeave} style={{ cursor: "help" }}>
      <rect x={b.x0} y={b.y0} width={w} height={h} fill={fill}
            fillOpacity={s.state === "held" ? 0.26 : 1} stroke={stroke} strokeWidth={0.06} />
      {s.blocked && <rect x={b.x0} y={b.y0} width={w} height={h} fill="url(#hatch-red)" opacity={0.7} />}
      {s.waiters.length > 0 && (
        <rect x={b.x0 - 0.08} y={b.y0 - 0.08} width={w + 0.16} height={h + 0.16} fill="none"
              stroke="var(--c-warn)" strokeWidth={0.08} strokeDasharray="0.3 0.2" />
      )}
      {info.one_way && info.canonical_outside && (
        <path d={`M${cx - 0.14} ${b.y0 + 0.22} L${cx} ${b.y0 + 0.4} L${cx + 0.14} ${b.y0 + 0.22}`}
              stroke="var(--c-muted)" strokeWidth={0.05} fill="none" opacity={0.55} />
      )}
      {/* lock tag hangs on the rack beside the aisle entry, so it never hides the robot */}
      {tag && tagAt && (
        <g transform={`translate(${tagAt.x},${tagAt.y})`} pointerEvents="none">
          <rect x={0} y={0} width={TAG_W} height={TAG_H} rx={0.12} fill="var(--c-panel)" stroke={tagColor} strokeWidth={0.05} />
          {s.state === "held" ? (
            <path d="M0.2 0.23 h0.26 v0.18 h-0.26 z M0.24 0.23 v-0.07 a0.09 0.09 0 0 1 0.18 0 v0.07" fill="none"
                  stroke="var(--m-label)" strokeWidth={0.04} />
          ) : null}
          <text x={s.state === "held" ? 0.86 : 0.65} y={0.37} fontSize={0.32} textAnchor="middle" fill="var(--m-label)"
                fontWeight={700} fontFamily="var(--font-mono)">
            {s.state === "unknown" ? "? unk" : tag}
          </text>
        </g>
      )}
      {s.waiters.length > 0 && waitAt && (
        <g transform={`translate(${waitAt.x},${waitAt.y})`} pointerEvents="none">
          <rect x={0} y={0} width={WAIT_W} height={TAG_H} rx={0.12} fill="var(--c-warn)" />
          <text x={0.5} y={0.37} fontSize={0.3} textAnchor="middle" fill="#1a1204" fontWeight={700}>
            ⏳{s.waiters.length}
          </text>
        </g>
      )}
    </g>
  );
}

/* --------------------------------------------------------------- robots */
const BADGE: Partial<Record<string, { g: string; c: string }>> = {
  waiting: { g: "‖", c: "var(--c-warn)" },
  yielding: { g: "↺", c: "var(--c-crit)" },
  charging: { g: "⚡", c: "var(--c-accent)" },
  working: { g: "■", c: "var(--c-ok)" },
  lost: { g: "?", c: "var(--c-unknown)" },
};

function RobotShape({ r, gs, labelAt, selected, dim, safety, inCycle, onEnter, onLeave, onClick }: {
  r: Robot; gs: number; labelAt: Pt | null; selected: boolean; dim: boolean; safety: boolean; inCycle: boolean;
  onEnter: (e: RPointerEvent) => void; onLeave: () => void; onClick: () => void;
}) {
  const color = r.stale ? "var(--c-unknown)" : robotColor(r.id);
  const inflate = r.stale ? Math.min(1.0, Math.max(0, r.age - 0.45)) : 0;
  const badge = BADGE[r.stale ? "lost" : r.mode];
  const deg = (r.th * 180) / Math.PI;
  return (
    <g transform={`translate(${r.x},${r.y})`} opacity={dim ? 0.35 : 1} style={{ cursor: "pointer" }}
       onPointerEnter={onEnter} onPointerLeave={onLeave}
       onPointerDown={(e) => { e.stopPropagation(); }}
       onPointerUp={(e) => { e.stopPropagation(); onClick(); }}
       role="button" aria-label={`Robot ${r.id}, ${MODE_LABEL[r.mode]}`}>
      {(safety || r.stale) && (
        <circle r={0.4 + inflate} fill={r.stale ? "var(--c-unknown)" : "none"} fillOpacity={0.08}
                stroke={r.stale ? "var(--c-unknown)" : color} strokeWidth={0.04} strokeDasharray="0.14 0.1" />
      )}
      {inCycle && <circle r={0.5} fill="none" stroke="var(--c-crit)" strokeWidth={0.09} />}
      {selected && <circle r={0.52} fill="none" stroke="var(--c-accent)" strokeWidth={0.08} />}
      <g transform={`scale(${gs}) rotate(${deg})`}>
        <circle r={0.3} fill={color} stroke="var(--m-robot-stroke)" strokeWidth={0.05} />
        <path d="M0.27 0 L-0.08 0.16 L-0.08 -0.16 Z" fill="var(--m-robot-stroke)" opacity={0.75} />
      </g>
      {labelAt && (
        <g transform={`translate(${labelAt.x},${labelAt.y}) scale(${gs})`} pointerEvents="none">
          <rect x={-0.36} y={-0.21} width={0.72} height={0.42} rx={0.1} fill="var(--c-panel)" opacity={0.92} />
          <text y={0.12} fontSize={0.32} textAnchor="middle" fill="var(--m-label)" fontWeight={700}
                fontFamily="var(--font-mono)">
            {`R${r.id}`}
          </text>
        </g>
      )}
      {badge && (
        <g transform={`scale(${gs}) translate(0.3,0.26)`} pointerEvents="none">
          <circle r={0.17} fill={badge.c} stroke="var(--m-robot-stroke)" strokeWidth={0.03} />
          <text y={0.08} fontSize={0.22} textAnchor="middle" fill="#0b0f16" fontWeight={800}>{badge.g}</text>
        </g>
      )}
    </g>
  );
}

/* --------------------------------------------------------------- tooltips & controls */
function Tooltip({ x, y, w, h, children }: { x: number; y: number; w: number; h: number; children: React.ReactNode }) {
  const left = x + 260 > w ? x - 256 : x + 16;
  const top = y + 170 > h ? Math.max(8, y - 170) : y + 16;
  return (
    <div className="pointer-events-none absolute z-20 w-60 rounded-[var(--radius-card)] border border-line bg-panel p-3 text-xs shadow-lg"
         style={{ left, top }} role="tooltip">
      {children}
    </div>
  );
}

function Row({ k, v }: { k: string; v: React.ReactNode }) {
  return (
    <div className="flex justify-between gap-3 py-0.5">
      <span className="text-muted">{k}</span>
      <span className="text-right text-fg">{v}</span>
    </div>
  );
}

function RobotTip({ r, sections }: { r: Robot; sections: SectionView[] }) {
  const lab = (sid: number | null) => (sid == null ? "—" : sections.find((s) => s.sid === sid)?.label ?? `#${sid}`);
  return (
    <div>
      <div className="mb-1.5 flex items-center gap-2 font-semibold">
        <span className="inline-block h-2.5 w-2.5 rounded-full" style={{ background: robotColor(r.id) }} />
        <span className="num">R{r.id}</span>
        <span className="text-muted">·</span>
        <span>{r.stale ? "Stale (not heard)" : MODE_LABEL[r.mode]}</span>
      </div>
      <Row k="Task" v={r.task != null ? <span className="num">#{r.task} {STAGE_LABEL[r.stage] ?? ""}</span> : "—"} />
      <Row k="Battery" v={<span className="num">{r.battery?.toFixed(0) ?? "—"}%</span>} />
      <Row k="Lock held" v={r.held.length ? r.held.map((s) => lab(s)).join(", ") : "—"} />
      <Row k="Waiting for" v={r.wq != null ? `${lab(r.wq)}${r.wf ? ` (R${r.wf})` : ""}` : r.wf ? `R${r.wf}` : "—"} />
      <Row k="Last heard" v={<span className="num">{fmtAge(r.age)} ago</span>} />
    </div>
  );
}

function SectionTip({ s, oneWay }: { s: SectionView; oneWay: boolean }) {
  const st = s.state === "free" ? "Free" : s.state === "held" ? `Held by R${s.holder}` : "Occupied — unknown";
  return (
    <div>
      <div className="mb-1.5 font-semibold">
        Aisle <span className="num">{s.label}</span>
        {oneWay && <span className="ml-1 text-muted">(one-way ↓)</span>}
      </div>
      <Row k="State" v={st} />
      <Row k="Lease left" v={s.lease != null ? <span className="num">{s.lease.toFixed(1)} s</span> : "—"} />
      <Row k="Waiters" v={s.waiters.length ? s.waiters.map((w) => `R${w}`).join(", ") : "none"} />
      {s.blocked && <Row k="Blockage" v={<span className="text-crit">✕ reported blocked</span>} />}
    </div>
  );
}

function IconBtn({ label, onClick, children }: { label: string; onClick: () => void; children: React.ReactNode }) {
  return (
    <button type="button" aria-label={label} title={label} onClick={onClick}
            className="grid h-8 w-8 place-items-center text-muted hover:bg-panel-2 hover:text-fg">
      {children}
    </button>
  );
}
