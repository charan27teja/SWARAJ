"""Physics stand-in (replaced by Gazebo in P8).

Holds the *true* robot poses, integrates differential-drive kinematics from each robot's
velocity command, simulates a 2D lidar per robot, detects and logs collisions (ground truth
for metrics), models batteries and injects blockages. It never makes a coordination decision
and never relays anything between robots: its only outputs per robot are that robot's own
pose, battery and scan.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from amr.core.config import Config, DEFAULT
from amr.core.geometry import wrap
from amr.core.grid_map import GridMap


@dataclass
class Body:
    rid: int
    x: float
    y: float
    th: float
    battery: float = 100.0
    v: float = 0.0
    w: float = 0.0
    v_cmd: float = 0.0
    w_cmd: float = 0.0
    powered: bool = True     # False after the robot's process is killed: it brakes and stays put
    present: bool = True     # False once an operator has removed the body from the floor
    odometer: float = 0.0


@dataclass
class Blockage:
    bid: int
    x: float
    y: float
    half: float = 0.4        # a pallet ~0.8 m square


@dataclass
class CollisionEvent:
    t: float
    kind: str                # "robot" | "wall" | "obstacle"
    a: int
    b: int | None
    x: float
    y: float


class World:
    def __init__(self, gmap: GridMap, cfg: Config = DEFAULT, seed: int = 0):
        self.map = gmap
        self.cfg = cfg
        self.rng = np.random.default_rng(seed)
        self.t = 0.0
        self.bodies: dict[int, Body] = {}
        self.blockages: dict[int, Blockage] = {}
        self._next_bid = 1
        self.events: list[CollisionEvent] = []
        self._contacts: set = set()
        k = cfg.lidar_rays
        self.ray_offsets = -math.pi + np.arange(k) * (2 * math.pi / k)
        self.min_robot_dist = float("inf")      # closest approach between any two bodies (m)

    # ------------------------------------------------------------------ bodies
    def add_robot(self, rid: int, x: float, y: float, th: float = 0.0, battery: float = 100.0) -> None:
        self.bodies[rid] = Body(rid, x, y, th, battery)

    def set_cmd(self, rid: int, v: float, w: float) -> None:
        b = self.bodies.get(rid)
        if b is not None and b.powered:
            b.v_cmd = float(v)
            b.w_cmd = float(w)

    def kill(self, rid: int) -> None:
        b = self.bodies.get(rid)
        if b:
            b.powered = False
            b.v_cmd = b.w_cmd = 0.0

    def remove_body(self, rid: int) -> None:
        b = self.bodies.get(rid)
        if b:
            b.present = False
            b.powered = False

    def pose(self, rid: int) -> tuple[float, float, float]:
        b = self.bodies[rid]
        return (b.x, b.y, b.th)

    # ------------------------------------------------------------------ blockages
    def add_blockage(self, x: float, y: float, half: float = 0.4) -> int:
        bid = self._next_bid
        self._next_bid += 1
        self.blockages[bid] = Blockage(bid, x, y, half)
        return bid

    def remove_blockage(self, bid: int) -> None:
        self.blockages.pop(bid, None)

    # ------------------------------------------------------------------ physics
    def _hits_wall(self, x: float, y: float, r: float) -> bool:
        m = self.map
        for cy in range(int(math.floor(y - r)), int(math.floor(y + r)) + 1):
            for cx in range(int(math.floor(x - r)), int(math.floor(x + r)) + 1):
                if m.is_wall_xy(cx, cy):
                    px = min(max(x, cx), cx + 1)
                    py = min(max(y, cy), cy + 1)
                    if (x - px) ** 2 + (y - py) ** 2 < (r - 1e-3) ** 2:
                        return True
        return False

    def step(self, dt: float) -> None:
        c = self.cfg
        for b in self.bodies.values():
            if not b.present:
                continue
            vt = b.v_cmd if b.powered else 0.0
            wt = b.w_cmd if b.powered else 0.0
            vt = min(max(vt, 0.0), c.v_max)
            wt = min(max(wt, -c.w_max), c.w_max)
            dv = c.a_max * dt
            dw = c.aw_max * dt
            b.v = min(max(vt, b.v - dv), b.v + dv)
            b.w = min(max(wt, b.w - dw), b.w + dw)
            if b.battery <= 0.0:
                b.v = b.w = 0.0
            nx = b.x + b.v * math.cos(b.th) * dt
            ny = b.y + b.v * math.sin(b.th) * dt
            b.th = wrap(b.th + b.w * dt)
            if self._hits_wall(nx, ny, c.radius):
                key = ("wall", b.rid)
                if key not in self._contacts:
                    self._contacts.add(key)
                    self.events.append(CollisionEvent(self.t, "wall", b.rid, None, b.x, b.y))
                b.v = 0.0
            else:
                self._contacts.discard(("wall", b.rid))
                moved = math.hypot(nx - b.x, ny - b.y)
                b.odometer += moved
                b.x, b.y = nx, ny
                if b.powered:
                    b.battery -= moved * c.battery_per_m
            if b.powered:
                b.battery -= c.battery_idle_per_s * dt
                if b.v < 0.05 and self._on_dock(b):
                    b.battery += c.battery_charge_per_s * dt
                b.battery = min(100.0, max(0.0, b.battery))
        self.t += dt
        self._detect_collisions()

    def _on_dock(self, b: Body) -> bool:
        cell = (int(math.floor(b.x)), int(math.floor(b.y)))
        if cell not in self._dock_set():
            return False
        return math.hypot(b.x - cell[0] - 0.5, b.y - cell[1] - 0.5) < 0.3

    def _dock_set(self):
        s = getattr(self, "_docks", None)
        if s is None:
            s = self._docks = set(self.map.docks)
        return s

    def _detect_collisions(self) -> None:
        bodies = [b for b in self.bodies.values() if b.present]
        n = len(bodies)
        lim = 2 * self.cfg.radius - 1e-3
        for i in range(n):
            a = bodies[i]
            for j in range(i + 1, n):
                b = bodies[j]
                d = math.hypot(a.x - b.x, a.y - b.y)
                if d < self.min_robot_dist:
                    self.min_robot_dist = d
                key = ("robot", min(a.rid, b.rid), max(a.rid, b.rid))
                if d < lim:
                    if key not in self._contacts:
                        self._contacts.add(key)
                        self.events.append(CollisionEvent(self.t, "robot", key[1], key[2],
                                                          (a.x + b.x) / 2, (a.y + b.y) / 2))
                else:
                    self._contacts.discard(key)
            for bl in self.blockages.values():
                px = min(max(a.x, bl.x - bl.half), bl.x + bl.half)
                py = min(max(a.y, bl.y - bl.half), bl.y + bl.half)
                key = ("obstacle", a.rid, bl.bid)
                if math.hypot(a.x - px, a.y - py) < self.cfg.radius - 1e-3:
                    if key not in self._contacts:
                        self._contacts.add(key)
                        self.events.append(CollisionEvent(self.t, "obstacle", a.rid, bl.bid, a.x, a.y))
                else:
                    self._contacts.discard(key)

    @property
    def robot_collisions(self) -> int:
        return sum(1 for e in self.events if e.kind == "robot")

    # ------------------------------------------------------------------ lidar
    def scans(self, rids: list[int]) -> dict[int, np.ndarray]:
        """Simulated 2D lidar for several robots at once (vectorised DDA ray casting).

        Returns, per robot, `lidar_rays` ranges (float32) at angles -pi + i*2pi/K in the
        robot frame. Rays hit walls/racks, other robot bodies and blockages.
        """
        rids = [r for r in rids if r in self.bodies and self.bodies[r].present]
        if not rids:
            return {}
        c = self.cfg
        R, K = len(rids), len(self.ray_offsets)
        ox = np.array([self.bodies[r].x for r in rids])[:, None].repeat(K, 1).ravel()
        oy = np.array([self.bodies[r].y for r in rids])[:, None].repeat(K, 1).ravel()
        th = np.array([self.bodies[r].th for r in rids])[:, None]
        ang = (th + self.ray_offsets[None, :]).ravel()
        dx, dy = np.cos(ang), np.sin(ang)
        hit = self._cast_walls(ox, oy, dx, dy, c.lidar_range)
        # other robot bodies (circles)
        others = [b for b in self.bodies.values() if b.present]
        if len(others) > 1:
            cx = np.array([b.x for b in others])
            cy = np.array([b.y for b in others])
            fx = ox[:, None] - cx[None, :]
            fy = oy[:, None] - cy[None, :]
            bb = fx * dx[:, None] + fy * dy[:, None]
            cc = fx * fx + fy * fy - c.radius ** 2
            disc = bb * bb - cc
            with np.errstate(invalid="ignore"):
                t = -bb - np.sqrt(disc)
            t = np.where((disc >= 0) & (t > 1e-6) & (cc > 0), t, np.inf)
            hit = np.minimum(hit, t.min(axis=1))
        for bl in self.blockages.values():
            hit = np.minimum(hit, self._cast_box(ox, oy, dx, dy, bl.x - bl.half, bl.y - bl.half,
                                                 bl.x + bl.half, bl.y + bl.half))
        noise = self.rng.normal(0.0, c.lidar_noise, hit.shape)
        hit = np.where(hit < c.lidar_range, np.clip(hit + noise, 0.0, c.lidar_range), c.lidar_range)
        hit = hit.astype(np.float32).reshape(R, K)
        return {r: hit[i] for i, r in enumerate(rids)}

    def _cast_walls(self, ox, oy, dx, dy, max_range):
        occ = getattr(self, "_occ_pad", None)
        if occ is None:
            # 1-cell wall border: rays always stop before leaving the padded grid (no clipping)
            occ = self._occ_pad = np.pad(self.map.occ, 1, constant_values=True)
        ix = np.floor(ox).astype(np.int64) + 1
        iy = np.floor(oy).astype(np.int64) + 1
        with np.errstate(divide="ignore"):
            invx = np.where(dx != 0, 1.0 / np.abs(dx), np.inf)
            invy = np.where(dy != 0, 1.0 / np.abs(dy), np.inf)
        stepx = np.where(dx > 0, 1, -1)
        stepy = np.where(dy > 0, 1, -1)
        fx = ox - np.floor(ox)
        fy = oy - np.floor(oy)
        tmx = np.where(dx > 0, 1.0 - fx, fx) * invx
        tmy = np.where(dy > 0, 1.0 - fy, fy) * invy
        tmx = np.nan_to_num(tmx, nan=np.inf)
        tmy = np.nan_to_num(tmy, nan=np.inf)
        hit = np.full(ox.shape, max_range)
        active = np.ones(ox.shape, dtype=bool)
        for _ in range(int(2 * math.ceil(max_range) + 4)):
            cx = tmx < tmy
            t = np.where(cx, tmx, tmy)
            mx = cx & active
            my = ~cx & active
            ix = ix + stepx * mx
            iy = iy + stepy * my
            tmx = np.where(mx, tmx + invx, tmx)
            tmy = np.where(my, tmy + invy, tmy)
            occ_hit = occ[iy, ix]
            newly = active & occ_hit & (t < max_range)
            hit[newly] = t[newly]
            active &= ~occ_hit & (t < max_range)
            if not active.any():
                break
        return hit

    @staticmethod
    def _cast_box(ox, oy, dx, dy, x0, y0, x1, y1):
        with np.errstate(divide="ignore", invalid="ignore"):
            tx0 = (x0 - ox) / dx
            tx1 = (x1 - ox) / dx
            ty0 = (y0 - oy) / dy
            ty1 = (y1 - oy) / dy
        tmin = np.maximum(np.minimum(tx0, tx1), np.minimum(ty0, ty1))
        tmax = np.minimum(np.maximum(tx0, tx1), np.maximum(ty0, ty1))
        ok = (tmax >= tmin) & (tmax > 0) & np.isfinite(tmin)
        return np.where(ok & (tmin > 0), tmin, np.inf)

    # ------------------------------------------------------------------ export
    def truth(self) -> dict:
        return {
            "t": round(self.t, 3),
            "robots": {r: {"x": round(b.x, 3), "y": round(b.y, 3), "th": round(b.th, 3),
                           "battery": round(b.battery, 2), "powered": b.powered, "present": b.present}
                       for r, b in self.bodies.items()},
            "blockages": [{"id": bl.bid, "x": bl.x, "y": bl.y, "half": bl.half} for bl in self.blockages.values()],
            "collisions": self.robot_collisions,
            "wall_contacts": sum(1 for e in self.events if e.kind == "wall"),
            "obstacle_contacts": sum(1 for e in self.events if e.kind == "obstacle"),
            "min_robot_dist": None if math.isinf(self.min_robot_dist) else round(self.min_robot_dist, 3),
        }
