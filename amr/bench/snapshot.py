"""Render a fast-runtime state to PNG (debugging / report figures)."""
from __future__ import annotations

import math

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Circle, Rectangle  # noqa: E402


def render(sim, path: str, title: str = "") -> None:
    gm = sim.map
    fig, ax = plt.subplots(figsize=(12, 9.5))
    for y in range(gm.height):
        for x in range(gm.width):
            if gm.occ[y, x]:
                ax.add_patch(Rectangle((x, y), 1, 1, color="#444"))
    for s in gm.sections:
        for (x, y) in s.cells:
            ax.add_patch(Rectangle((x, y), 1, 1, color="#dde8ff" if s.kind == "corridor" else "#ffe0b0"))
    for (x, y) in gm.mouths:
        ax.add_patch(Rectangle((x + .4, y + .4), .2, .2, color="#bbb"))
    for c, col in ((gm.docks, "#4caf50"), (gm.drops, "#e91e63"), (gm.bays, "#9e9e9e"), (gm.picks, "#2196f3")):
        for (x, y) in c:
            ax.add_patch(Rectangle((x + .3, y + .3), .4, .4, color=col, alpha=.6))
    for bl in sim.world.blockages.values():
        ax.add_patch(Rectangle((bl.x - bl.half, bl.y - bl.half), 2 * bl.half, 2 * bl.half, color="#795548"))
    cmap = plt.get_cmap("tab20")
    for rid, b in sim.world.bodies.items():
        if not b.present:
            continue
        a = sim.agents.get(rid)
        col = cmap((rid - 1) % 20)
        if a is not None and getattr(a, "path", None):
            p = a.path[a.idx:]
            ax.plot([c[0] + .5 for c in p], [c[1] + .5 for c in p], "-", color=col, lw=1.5, alpha=.7)
        ax.add_patch(Circle((b.x, b.y), 0.3, color=col if b.powered else "#888", ec="k", lw=.8))
        ax.plot([b.x, b.x + .35 * math.cos(b.th)], [b.y, b.y + .35 * math.sin(b.th)], "k-", lw=1)
        lab = f"{rid}"
        if a is not None and hasattr(a, "mode"):
            wf = a._wait_for_eff() if hasattr(a, "_wait_for_eff") else None
            lab += f"\n{a.mode[:4]}" + (f">{wf}" if wf else "")
        ax.text(b.x + .3, b.y - .3, lab, fontsize=7)
    ax.set_xlim(0, gm.width)
    ax.set_ylim(gm.height, 0)
    ax.set_aspect("equal")
    ax.set_title(title or f"t={sim.t:.1f}s")
    fig.tight_layout()
    fig.savefig(path, dpi=80)
    plt.close(fig)
