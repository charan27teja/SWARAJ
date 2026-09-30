"""L1s Safety: network-independent last line of defence.

Uses only the robot's own simulated 2D lidar. The protective field is the rectangle the robot
body would sweep while braking from its current speed (velocity-scaled). The allowed forward
speed is the largest v such that reaction distance + braking distance + margin fits before the
first lidar return inside the swept corridor. Turning in place is always allowed, so a robot
can rotate away from an obstacle and then move.
"""
from __future__ import annotations

import math

import numpy as np


class Lidar:
    """Robot-frame ray geometry for a scan layout (precomputed once)."""

    def __init__(self, n_rays: int, max_range: float):
        self.n = n_rays
        self.max_range = max_range
        self.angles = -math.pi + np.arange(n_rays) * (2 * math.pi / n_rays)
        self.cos = np.cos(self.angles)
        self.sin = np.sin(self.angles)


def forward_clearance(ranges: np.ndarray, lidar: Lidar, half_width: float) -> float:
    """Distance the robot centre can travel straight ahead before its swept corridor
    (|lateral| < half_width) touches a lidar return."""
    valid = ranges < lidar.max_range - 1e-3
    px = ranges * lidar.cos
    py = ranges * lidar.sin
    m = valid & (px > 0.0) & (np.abs(py) < half_width)
    if not m.any():
        return float("inf")
    return float(np.min(px[m] - np.sqrt(half_width ** 2 - py[m] ** 2)))


def forward_clearance_points(pts: np.ndarray, x: float, y: float, th: float, half_width: float) -> float:
    """Same as forward_clearance for world-frame points (e.g. lidar returns no peer explains)."""
    if pts is None or not len(pts):
        return float("inf")
    dx, dy = pts[:, 0] - x, pts[:, 1] - y
    c, s = math.cos(th), math.sin(th)
    px = dx * c + dy * s
    py = -dx * s + dy * c
    m = (px > 0.0) & (np.abs(py) < half_width)
    if not m.any():
        return float("inf")
    return float(np.min(px[m] - np.sqrt(half_width ** 2 - py[m] ** 2)))


def max_safe_speed(clearance: float, a_max: float, t_react: float, margin: float) -> float:
    """Largest v with v*t_react + v^2/(2a) + margin <= clearance."""
    free = clearance - margin
    if free <= 0.0:
        return 0.0
    if math.isinf(free):
        return float("inf")
    return a_max * (-t_react + math.sqrt(t_react * t_react + 2.0 * free / a_max))
