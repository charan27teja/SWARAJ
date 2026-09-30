"""P3: ORCA (pairwise / n-body no-collision) and the lidar protective stop."""
import math

import numpy as np
import pytest

from amr.agent import orca
from amr.agent.safety import Lidar, forward_clearance, max_safe_speed


def simulate(starts, goals, radius=0.3, margin=0.1, vmax=1.0, dt=0.05, steps=600, tau=2.0):
    """Holonomic agents running ORCA against each other. Returns min pairwise distance and
    whether all reached their goals."""
    n = len(starts)
    p = [list(s) for s in starts]
    v = [[0.0, 0.0] for _ in range(n)]
    dmin = float("inf")
    for _ in range(steps):
        newv = []
        for i in range(n):
            gx, gy = goals[i][0] - p[i][0], goals[i][1] - p[i][1]
            d = math.hypot(gx, gy)
            sp = min(vmax, d / dt) if d > 1e-9 else 0.0
            pref = (gx / d * sp, gy / d * sp) if d > 1e-9 else (0.0, 0.0)
            # tiny deterministic perturbation breaks perfect symmetry (as in RVO2 examples)
            ang = 1e-3 * (i + 1)
            pref = (pref[0] * math.cos(ang) - pref[1] * math.sin(ang), pref[0] * math.sin(ang) + pref[1] * math.cos(ang))
            lines = []
            for j in range(n):
                if j != i and math.hypot(p[j][0] - p[i][0], p[j][1] - p[i][1]) < 5.0:
                    lines.append(orca.agent_line(p[i][0], p[i][1], v[i][0], v[i][1], p[j][0], p[j][1],
                                                 v[j][0], v[j][1], 2 * (radius + margin), tau, dt))
            newv.append(orca.solve([], lines, vmax, pref[0], pref[1]))
        for i in range(n):
            v[i] = list(newv[i])
            p[i][0] += v[i][0] * dt
            p[i][1] += v[i][1] * dt
        for i in range(n):
            for j in range(i + 1, n):
                dmin = min(dmin, math.hypot(p[i][0] - p[j][0], p[i][1] - p[j][1]))
    reached = all(math.hypot(p[i][0] - goals[i][0], p[i][1] - goals[i][1]) < 0.1 for i in range(n))
    return dmin, reached


def test_orca_head_on_pair_never_collides():
    dmin, reached = simulate([(0, 0), (6, 0)], [(6, 0), (0, 0)])
    assert dmin >= 0.6 and reached


def test_orca_crossing_pair_never_collides():
    dmin, reached = simulate([(0, 3), (3, 0)], [(6, 3), (3, 6)])
    assert dmin >= 0.6 and reached


def test_orca_circle_swap_5_agents():
    n = 5
    starts = [(4 * math.cos(2 * math.pi * k / n), 4 * math.sin(2 * math.pi * k / n)) for k in range(n)]
    goals = [(-x, -y) for x, y in starts]
    dmin, reached = simulate(starts, goals, steps=1200)
    assert dmin >= 0.6
    assert reached


def test_orca_respects_hard_wall_halfplane():
    # wall at x = 1.0 to the right; agent wants to go right at full speed
    line = orca.halfplane_line(-1.0, 0.0, -(1.0 - 0.35 - 0.0) / 0.6)
    vx, vy = orca.solve([line], [], 1.0, 1.0, 0.0)
    assert vx <= (1.0 - 0.35) / 0.6 + 1e-9


def test_max_speed_along_heading():
    line = orca.halfplane_line(-1.0, 0.0, -0.5)          # v.x <= 0.5
    assert orca.max_speed_along([line], 1.0, 0.0, 1.0) == pytest.approx(0.5)
    assert orca.max_speed_along([line], 0.0, 1.0, 1.0) == pytest.approx(1.0)
    push = orca.halfplane_line(-1.0, 0.0, 0.2)           # must move left at >= 0.2
    assert orca.max_speed_along([push], 1.0, 0.0, 1.0) is None


def test_protective_stop_scales_with_distance():
    lid = Lidar(120, 8.0)
    ranges = np.full(120, 8.0, dtype=np.float32)
    k0 = int(np.argmin(np.abs(lid.angles)))
    ranges[k0] = 1.0                                     # obstacle 1 m straight ahead
    clr = forward_clearance(ranges, lid, 0.35)
    assert clr == pytest.approx(1.0 - 0.35, abs=0.01)
    v_far = max_safe_speed(3.0, 1.5, 0.1, 0.08)
    v_near = max_safe_speed(clr, 1.5, 0.1, 0.08)
    assert v_far > v_near > 0.0
    assert max_safe_speed(0.05, 1.5, 0.1, 0.08) == 0.0
    # obstacle beside the robot (outside the swept corridor) does not stop it
    ranges[:] = 8.0
    kside = int(np.argmin(np.abs(lid.angles - math.pi / 2)))
    ranges[kside] = 0.45
    assert math.isinf(forward_clearance(ranges, lid, 0.35))
