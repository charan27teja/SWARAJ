"""Warehouse map: loading, geometry helpers, critical-section and passing-bay extraction.

Cells are (x, y) integer tuples; the cell (x, y) covers [x, x+1) x [y, y+1) metres, so its
centre is (x + 0.5, y + 0.5). y grows "down" the screen. Everything outside the grid is wall.

Critical sections (spec: corridor segments or intersections narrower than 2 robot diameters)
are extracted automatically:
  * a free cell is *wide* if it belongs to at least one all-free 2x2 block (>= 2 m of clear
    width, i.e. more than 2 robot diameters), otherwise *narrow* (1 m < 1.2 m);
  * narrow cells with exactly one free neighbour are *pockets* (docks, drops, passing bays)
    and are terminals, not sections;
  * remaining narrow cells with >= 3 narrow neighbours are *junctions*; connected junction
    cells form one intersection section;
  * the rest of the narrow cells, split at junctions, form corridor sections.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml

Cell = tuple[int, int]

DIRS4 = ((1, 0), (-1, 0), (0, 1), (0, -1))
FREE_CHARS = set(".PDCB")


@dataclass
class Section:
    sid: int
    kind: str                      # "corridor" | "intersection"
    cells: list[Cell]
    ends: list[tuple[Cell, Cell]] = field(default_factory=list)  # (inside cell, outside cell)
    canonical_outside: Cell | None = None   # only entry allowed in degraded mode
    label: str = ""
    one_way: bool = False                   # traffic rule: enter at canonical end only
    exit_outside: Cell | None = None

    @property
    def length(self) -> float:
        return float(len(self.cells))


class GridMap:
    def __init__(self, rows: list[str], name: str = "map", cell_size: float = 1.0,
                 one_way_aisles: bool = True):
        if cell_size != 1.0:
            raise ValueError("only 1 m cells are supported")
        self.name = name
        self.one_way_aisles = one_way_aisles
        self.rows = rows
        self.height = len(rows)
        self.width = max(len(r) for r in rows)
        self.rows = [r.ljust(self.width, "#") for r in rows]
        self.occ = np.array([[c not in FREE_CHARS for c in r] for r in self.rows], dtype=bool)
        self.free: set[Cell] = {(x, y) for y in range(self.height) for x in range(self.width)
                                if not self.occ[y, x]}
        self.picks = self._cells_with("P")
        self.drops = self._cells_with("D")
        self.docks = self._cells_with("C")
        self._extract()

    # ------------------------------------------------------------------ loading
    @classmethod
    def load(cls, path: str | Path) -> "GridMap":
        data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        rows = [r for r in data["grid"].splitlines() if r.strip()]
        return cls(rows, name=data.get("name", Path(path).stem), cell_size=data.get("cell_size", 1.0),
                   one_way_aisles=bool(data.get("one_way_aisles", True)))

    def _cells_with(self, ch: str) -> list[Cell]:
        return sorted(((x, y) for y, r in enumerate(self.rows) for x, c in enumerate(r) if c == ch),
                      key=lambda c: (c[1], c[0]))

    # ------------------------------------------------------------------ basics
    def is_free(self, c: Cell) -> bool:
        return c in self.free

    def is_wall_xy(self, x: int, y: int) -> bool:
        return not (0 <= x < self.width and 0 <= y < self.height) or bool(self.occ[y, x])

    def neighbors(self, c: Cell) -> list[Cell]:
        x, y = c
        return [(x + dx, y + dy) for dx, dy in DIRS4 if (x + dx, y + dy) in self.free]

    @staticmethod
    def center(c: Cell) -> tuple[float, float]:
        return (c[0] + 0.5, c[1] + 0.5)

    @staticmethod
    def cell_of(x: float, y: float) -> Cell:
        return (int(math.floor(x)), int(math.floor(y)))

    def nearest_free(self, x: float, y: float) -> Cell:
        c = self.cell_of(x, y)
        if c in self.free:
            return c
        return min(self.free, key=lambda f: (f[0] + 0.5 - x) ** 2 + (f[1] + 0.5 - y) ** 2)

    def wall_cells_near(self, x: float, y: float, rng: float) -> list[Cell]:
        x0, x1 = int(math.floor(x - rng)), int(math.floor(x + rng))
        y0, y1 = int(math.floor(y - rng)), int(math.floor(y + rng))
        out = []
        for cy in range(y0, y1 + 1):
            for cx in range(x0, x1 + 1):
                if self.is_wall_xy(cx, cy):
                    out.append((cx, cy))
        return out

    # ------------------------------------------------------------------ extraction
    def _extract(self) -> None:
        free = self.free
        wide: set[Cell] = set()
        for (x, y) in free:
            for ox in (0, -1):
                for oy in (0, -1):
                    blk = [(x + ox + i, y + oy + j) for i in (0, 1) for j in (0, 1)]
                    if all(b in free for b in blk):
                        wide.add((x, y))
        self.wide = wide
        narrow = free - wide
        pockets = {c for c in narrow if len(self.neighbors(c)) == 1}
        self.pockets = pockets
        self.bays = sorted((c for c in pockets if self.rows[c[1]][c[0]] in "B."),
                           key=lambda c: (c[1], c[0]))
        corridor_cells = narrow - pockets
        junction = {c for c in corridor_cells
                    if sum(1 for n in self.neighbors(c) if n in corridor_cells) >= 3}
        sections: list[Section] = []

        def flood(seed: Cell, pool: set[Cell]) -> list[Cell]:
            comp, stack = [], [seed]
            pool.discard(seed)
            while stack:
                c = stack.pop()
                comp.append(c)
                for n in self.neighbors(c):
                    if n in pool:
                        pool.discard(n)
                        stack.append(n)
            return sorted(comp, key=lambda c: (c[1], c[0]))

        pool = set(junction)
        while pool:
            comp = flood(min(pool, key=lambda c: (c[1], c[0])), pool)
            sections.append(Section(len(sections), "intersection", comp))
        pool = corridor_cells - junction
        while pool:
            comp = flood(min(pool, key=lambda c: (c[1], c[0])), pool)
            sections.append(Section(len(sections), "corridor", comp))
        # renumber: corridors first by position (stable, readable ids), then intersections
        sections.sort(key=lambda s: (s.kind != "corridor", s.cells[0][1], s.cells[0][0]))
        self.cell_section: dict[Cell, int] = {}
        for i, s in enumerate(sections):
            s.sid = i
            for c in s.cells:
                self.cell_section[c] = i
        for s in sections:
            cs = set(s.cells)
            for c in s.cells:
                for n in self.neighbors(c):
                    if n not in cs:
                        s.ends.append((c, n))
            # canonical end: prefer an end that opens onto a wide cell; lowest (y, x) first
            outs = sorted((o for _, o in s.ends if o in wide), key=lambda c: (c[1], c[0]))
            s.canonical_outside = outs[0] if outs else None
            xs = sorted({c[0] for c in s.cells})
            ys = sorted({c[1] for c in s.cells})
            # one-way picking aisles: straight corridors whose both ends open onto wide lanes
            wide_outs = [o for _, o in s.ends if o in wide]
            if (self.one_way_aisles and s.kind == "corridor" and len(s.ends) == 2 and len(wide_outs) == 2
                    and len(xs) == 1 and len(s.cells) >= 4):
                s.one_way = True
                s.canonical_outside = min(wide_outs, key=lambda c: c[1])
                s.exit_outside = max(wide_outs, key=lambda c: c[1])
            if s.kind == "intersection":
                s.label = f"X({xs[0]},{ys[0]})"
            elif len(xs) == 1:
                s.label = f"A{xs[0]}:{ys[0]}-{ys[-1]}"
            else:
                s.label = f"A{xs[0]}-{xs[-1]}:{ys[0]}"
        self.sections = sections
        # mouth cells: the outside cell in front of every section end and every pocket.
        # Robots must not park/wait on these (they would block someone else's exit).
        mouths: set[Cell] = set()
        for s in sections:
            for _, o in s.ends:
                if o not in self.cell_section:
                    mouths.add(o)
        for pk in pockets:
            mouths.update(self.neighbors(pk))
        self.mouths = mouths

    def section_of(self, c: Cell) -> int | None:
        return self.cell_section.get(c)

    # ------------------------------------------------------------------ export
    def to_json(self) -> dict:
        return {
            "name": self.name, "width": self.width, "height": self.height,
            "rows": self.rows,
            "picks": self.picks, "drops": self.drops, "docks": self.docks, "bays": self.bays,
            "sections": [{"sid": s.sid, "kind": s.kind, "label": s.label, "cells": s.cells,
                          "canonical_outside": s.canonical_outside, "one_way": s.one_way,
                          "exit_outside": s.exit_outside} for s in self.sections],
            "mouths": sorted(self.mouths),
        }


def load_map(path: str | Path) -> GridMap:
    return GridMap.load(path)
