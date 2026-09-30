"""Small geometry helpers (pure Python, hot-path friendly)."""
from __future__ import annotations

import math


def wrap(a: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return (a + math.pi) % (2 * math.pi) - math.pi


def dist(ax: float, ay: float, bx: float, by: float) -> float:
    return math.hypot(ax - bx, ay - by)


def point_box_dist(px: float, py: float, x0: float, y0: float, x1: float, y1: float) -> tuple[float, float, float]:
    """Distance from point to axis-aligned box and the closest point on the box."""
    cx = min(max(px, x0), x1)
    cy = min(max(py, y0), y1)
    return math.hypot(px - cx, py - cy), cx, cy


def project_on_segment(px, py, ax, ay, bx, by) -> tuple[float, float, float, float]:
    """Project p onto segment ab. Returns (t in [0,1], qx, qy, distance)."""
    vx, vy = bx - ax, by - ay
    L2 = vx * vx + vy * vy
    t = 0.0 if L2 <= 1e-12 else max(0.0, min(1.0, ((px - ax) * vx + (py - ay) * vy) / L2))
    qx, qy = ax + t * vx, ay + t * vy
    return t, qx, qy, math.hypot(px - qx, py - qy)
