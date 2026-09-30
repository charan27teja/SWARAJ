"""L1 Motion: Optimal Reciprocal Collision Avoidance (ORCA).

Follows the RVO2 library formulation (van den Berg, Guy, Lin, Manocha, "Reciprocal n-body
collision avoidance", ISRR 2011): one half-plane per neighbour from its velocity obstacle,
solved with the incremental 2D linear programs linearProgram1/2/3. Static obstacles (walls,
lidar-seen unknown objects) are hard half-planes that are never relaxed.

Pure Python on tuples: with ~5-15 constraints this is faster than numpy for one robot.
A line is (px, py, dx, dy); velocities v with det(d, p - v) <= 0 are permitted.
"""
from __future__ import annotations

import math

EPS = 1e-6


def _det(ax, ay, bx, by):
    return ax * by - ay * bx


def agent_line(px, py, vx, vy, opx, opy, ovx, ovy, radius, tau, dt, responsibility=0.5):
    """ORCA half-plane induced on the agent at p (velocity v) by a neighbour at op (velocity ov).
    `responsibility` = 0.5 for a reciprocating peer, 1.0 for a non-reciprocating obstacle."""
    rpx, rpy = opx - px, opy - py
    rvx, rvy = vx - ovx, vy - ovy
    dist_sq = rpx * rpx + rpy * rpy
    r_sq = radius * radius
    inv_tau = 1.0 / tau
    if dist_sq > r_sq:
        wx, wy = rvx - inv_tau * rpx, rvy - inv_tau * rpy
        w_len_sq = wx * wx + wy * wy
        dot1 = wx * rpx + wy * rpy
        if dot1 < 0.0 and dot1 * dot1 > r_sq * w_len_sq:
            w_len = math.sqrt(w_len_sq)
            ux_, uy_ = wx / w_len, wy / w_len
            dx, dy = uy_, -ux_
            k = radius * inv_tau - w_len
            ux, uy = k * ux_, k * uy_
        else:
            leg = math.sqrt(dist_sq - r_sq)
            if _det(rpx, rpy, wx, wy) > 0.0:
                dx = (rpx * leg - rpy * radius) / dist_sq
                dy = (rpx * radius + rpy * leg) / dist_sq
            else:
                dx = -(rpx * leg + rpy * radius) / dist_sq
                dy = -(-rpx * radius + rpy * leg) / dist_sq
            dot2 = rvx * dx + rvy * dy
            ux, uy = dot2 * dx - rvx, dot2 * dy - rvy
    else:
        inv_dt = 1.0 / dt
        wx, wy = rvx - inv_dt * rpx, rvy - inv_dt * rpy
        w_len = math.hypot(wx, wy)
        if w_len < EPS:
            # exactly overlapping and same velocity: push apart along the relative position
            n = math.hypot(rpx, rpy) or 1.0
            wx, wy, w_len = -rpx / n * EPS, -rpy / n * EPS, EPS
        ux_, uy_ = wx / w_len, wy / w_len
        dx, dy = uy_, -ux_
        k = radius * inv_dt - w_len
        ux, uy = k * ux_, k * uy_
    return (vx + responsibility * ux, vy + responsibility * uy, dx, dy)


def halfplane_line(nx, ny, min_normal_speed):
    """Constraint v . n >= min_normal_speed (n unit). Used for walls / static points."""
    return (min_normal_speed * nx, min_normal_speed * ny, ny, -nx)


def _lp1(lines, no, radius, ox, oy, dir_opt):
    px, py, dx, dy = lines[no]
    dot = px * dx + py * dy
    disc = dot * dot + radius * radius - (px * px + py * py)
    if disc < 0.0:
        return None
    s = math.sqrt(disc)
    t_left, t_right = -dot - s, -dot + s
    for i in range(no):
        qx, qy, ex, ey = lines[i]
        denom = _det(dx, dy, ex, ey)
        numer = _det(ex, ey, px - qx, py - qy)
        if abs(denom) <= EPS:
            if numer < 0.0:
                return None
            continue
        t = numer / denom
        if denom >= 0.0:
            t_right = min(t_right, t)
        else:
            t_left = max(t_left, t)
        if t_left > t_right:
            return None
    if dir_opt:
        t = t_right if ox * dx + oy * dy > 0.0 else t_left
    else:
        t = dx * (ox - px) + dy * (oy - py)
        t = t_left if t < t_left else (t_right if t > t_right else t)
    return (px + t * dx, py + t * dy)


def _lp2(lines, radius, ox, oy, dir_opt, res=None):
    if dir_opt:
        rx, ry = ox * radius, oy * radius
    elif ox * ox + oy * oy > radius * radius:
        n = math.hypot(ox, oy)
        rx, ry = ox / n * radius, oy / n * radius
    else:
        rx, ry = ox, oy
    for i, (px, py, dx, dy) in enumerate(lines):
        if _det(dx, dy, px - rx, py - ry) > 0.0:
            r = _lp1(lines, i, radius, ox, oy, dir_opt)
            if r is None:
                return i, (rx, ry)
            rx, ry = r
    return len(lines), (rx, ry)


def _lp3(lines, n_obst, begin, radius, rx, ry):
    distance = 0.0
    for i in range(begin, len(lines)):
        px, py, dx, dy = lines[i]
        if _det(dx, dy, px - rx, py - ry) > distance:
            proj = list(lines[:n_obst])
            for j in range(n_obst, i):
                qx, qy, ex, ey = lines[j]
                determinant = _det(dx, dy, ex, ey)
                if abs(determinant) <= EPS:
                    if dx * ex + dy * ey > 0.0:
                        continue
                    lpx, lpy = 0.5 * (px + qx), 0.5 * (py + qy)
                else:
                    k = _det(ex, ey, px - qx, py - qy) / determinant
                    lpx, lpy = px + k * dx, py + k * dy
                ndx, ndy = ex - dx, ey - dy
                n = math.hypot(ndx, ndy)
                if n < EPS:
                    continue
                proj.append((lpx, lpy, ndx / n, ndy / n))
            fail, r = _lp2(proj, radius, -dy, dx, True)
            if fail >= len(proj):
                rx, ry = r
            distance = _det(dx, dy, px - rx, py - ry)
    return rx, ry


def solve(obstacle_lines, agent_lines, v_max, pref_x, pref_y):
    """Velocity closest to the preferred one satisfying all hard obstacle lines and as many
    agent lines as possible (least-violation fallback, as in RVO2)."""
    lines = list(obstacle_lines) + list(agent_lines)
    n_obst = len(obstacle_lines)
    fail, (rx, ry) = _lp2(lines, v_max, pref_x, pref_y, False)
    if fail < len(lines):
        if fail < n_obst:
            # hard constraints themselves infeasible (e.g. squeezed between wall and object):
            # stop; the safety layer and re-planning take over.
            return 0.0, 0.0
        rx, ry = _lp3(lines, n_obst, fail, v_max, rx, ry)
    return rx, ry


def max_speed_along(lines, hx, hy, v_hi):
    """Largest v in [0, v_hi] such that v*(hx, hy) satisfies every line, or None if even the
    feasible part of the ray is empty (then the caller stops / rotates)."""
    lo, hi = 0.0, v_hi
    for px, py, dx, dy in lines:
        # constraint: det(d, p - v h) <= 0  <=>  det(d, p) - v det(d, h) <= 0
        a = dx * hy - dy * hx
        c = dx * py - dy * px
        if abs(a) < EPS:
            if c > 0.0:
                return None
            continue
        bound = c / a
        if a > 0.0:
            lo = max(lo, bound)
        else:
            hi = min(hi, bound)
        if lo > hi:
            return None
    return hi
