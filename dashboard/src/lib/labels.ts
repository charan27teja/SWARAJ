import type { MapInfo, Robot, SectionView } from "../types";

// Collision-checked placement of on-map text: aisle lock tags, queue badges and robot labels.
// Each label tries a few candidate positions and takes the first that overlaps no text already
// placed (with a small gap); if every candidate collides it is hidden — text is never drawn
// over text. Tags only consider static text and each other so they stay put as robots move.

interface Box { x0: number; y0: number; x1: number; y1: number }
export interface Pt { x: number; y: number }
export interface Placement {
  tags: Map<number, Pt | null>;        // top-left of the 1.3 x 0.5 m lock tag
  waits: Map<number, Pt | null>;       // top-left of the 1.0 x 0.5 m queue badge
  labels: Map<number, Pt | null>;      // centre offset of the robot label from the robot
}

export const TAG_W = 1.3, WAIT_W = 1.0, TAG_H = 0.5;
const GAP = 0.08;

function hit(a: Box, b: Box): boolean {
  return a.x0 < b.x1 + GAP && b.x0 < a.x1 + GAP && a.y0 < b.y1 + GAP && b.y0 < a.y1 + GAP;
}

export function sectionBox(map: MapInfo, sid: number): Box {
  let x0 = 1e9, y0 = 1e9, x1 = -1e9, y1 = -1e9;
  for (const [x, y] of map.sections[sid].cells) {
    x0 = Math.min(x0, x); y0 = Math.min(y0, y);
    x1 = Math.max(x1, x + 1); y1 = Math.max(y1, y + 1);
  }
  return { x0, y0, x1, y1 };
}

function firstFree(cands: Pt[], w: number, h: number, placed: Box[]): Pt | null {
  for (const c of cands) {
    const b = { x0: c.x, y0: c.y, x1: c.x + w, y1: c.y + h };
    if (!placed.some((p) => hit(p, b))) {
      placed.push(b);
      return c;
    }
  }
  return null;
}

export function layoutLabels(map: MapInfo, sections: SectionView[], robots: Robot[], gs: number,
                             show: { tags: boolean; labels: boolean }): Placement {
  const placed: Box[] = map.bays.map(([x, y]) => ({ x0: x + 0.3, y0: y + 0.25, x1: x + 0.7, y1: y + 0.75 }));
  const tags = new Map<number, Pt | null>();
  const waits = new Map<number, Pt | null>();
  const labels = new Map<number, Pt | null>();

  if (show.tags) {
    for (const s of sections) {
      const needTag = s.state !== "free";
      if (!needTag && !s.waiters.length) continue;
      const b = sectionBox(map, s.sid);
      const cx = (b.x0 + b.x1) / 2, cy = (b.y0 + b.y1) / 2;
      const vertical = b.y1 - b.y0 > b.x1 - b.x0;
      const single = b.y1 - b.y0 <= 1 && b.x1 - b.x0 <= 1;
      const cands: Pt[] = single
        ? [{ x: cx - 0.65, y: b.y0 - 0.62 }, { x: cx - 0.65, y: b.y1 + 0.12 }, { x: b.x1 + 0.1, y: cy - 0.25 }, { x: b.x0 - 1.4, y: cy - 0.25 }]
        : vertical
          ? [{ x: b.x1 + 0.08, y: b.y0 + 0.1 }, { x: b.x0 - 1.38, y: b.y0 + 0.1 }, { x: b.x1 + 0.08, y: b.y1 - 0.6 },
             { x: b.x0 - 1.38, y: b.y1 - 0.6 }, { x: cx - 0.65, y: b.y0 - 0.62 }, { x: cx - 0.65, y: b.y1 + 0.12 }]
          : [{ x: b.x0 + 0.05, y: b.y0 - 0.62 }, { x: b.x0 + 0.05, y: b.y1 + 0.12 }, { x: b.x1 - 1.35, y: b.y0 - 0.62 },
             { x: b.x1 - 1.35, y: b.y1 + 0.12 }];
      const tag = needTag ? firstFree(cands, TAG_W, TAG_H, placed) : null;
      tags.set(s.sid, tag);
      if (s.waiters.length) {
        const around = tag
          ? [{ x: tag.x, y: tag.y + 0.58 }, { x: tag.x + TAG_W + 0.1, y: tag.y }, { x: tag.x - WAIT_W - 0.1, y: tag.y },
             { x: tag.x, y: tag.y - 0.58 }]
          : cands;
        waits.set(s.sid, firstFree(around, WAIT_W, TAG_H, placed));
      }
    }
  }

  if (show.labels) {
    const hw = 0.36 * gs, hh = 0.21 * gs;
    // robot bodies and mode badges are obstacles for labels (not for tags, which must stay put)
    const others: Box[] = [];
    for (const r of robots) {
      const rr = 0.3 * gs;
      others.push({ x0: r.x - rr, y0: r.y - rr, x1: r.x + rr, y1: r.y + rr });
      const bx = r.x + 0.3 * gs, by = r.y + 0.26 * gs, br = 0.17 * gs;
      others.push({ x0: bx - br, y0: by - br, x1: bx + br, y1: by + br });
    }
    const all = placed.concat(others);
    const offs = [[0, -0.62], [0, 0.62], [0.8, 0], [-0.8, 0], [0.8, -0.6], [-0.8, -0.6], [0.8, 0.6], [-0.8, 0.6],
                  [0, -1.05], [0, 1.05]];
    for (const r of [...robots].sort((a, b) => a.id - b.id)) {
      let got: Pt | null = null;
      for (const [dx, dy] of offs) {
        const c = { x: r.x + dx * gs, y: r.y + dy * gs };
        const box = { x0: c.x - hw, y0: c.y - hh, x1: c.x + hw, y1: c.y + hh };
        if (!all.some((p) => hit(p, box))) {
          all.push(box);
          got = { x: dx * gs, y: dy * gs };
          break;
        }
      }
      labels.set(r.id, got);
    }
  }
  return { tags, waits, labels };
}
