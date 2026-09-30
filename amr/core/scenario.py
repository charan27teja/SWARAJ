"""Scenario definitions: robots, tasks, blockages and fault schedule. Seeded and reproducible.

A scenario is loaded from `scenarios/<name>.yaml` or generated (`random_overlap`). Every robot
receives the same order book (the tasks) - this models the warehouse management system
announcing orders; *who* does which task is decided by the robots themselves (L4).
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from pathlib import Path

import yaml

from amr.core.grid_map import GridMap

ROOT = Path(__file__).resolve().parents[2]


@dataclass
class RobotSpec:
    rid: int
    cell: tuple[int, int]
    theta: float = 0.0
    battery: float = 100.0


@dataclass
class TaskSpec:
    tid: int
    pick: tuple[int, int]
    drop: tuple[int, int]
    release: float = 0.0
    assign: int | None = None      # scripted initial owner (demo geometry only)

    def to_json(self) -> dict:
        return {"id": self.tid, "pick": list(self.pick), "drop": list(self.drop),
                "release": self.release, "assign": self.assign}


@dataclass
class Scenario:
    name: str
    map_path: str
    duration: float
    robots: list[RobotSpec]
    tasks: list[TaskSpec]
    blockages: list[dict] = field(default_factory=list)
    faults: list[dict] = field(default_factory=list)
    seed: int = 0
    meta: dict = field(default_factory=dict)
    config: dict = field(default_factory=dict)      # Config overrides (e.g. to force a demo geometry)
    _map: GridMap | None = None

    @property
    def map(self) -> GridMap:
        if self._map is None:
            p = Path(self.map_path)
            self._map = GridMap.load(p if p.is_absolute() else ROOT / p)
        return self._map

    def to_json(self) -> dict:
        return {
            "name": self.name, "map": self.map_path, "duration": self.duration, "seed": self.seed,
            "robots": [{"id": r.rid, "cell": list(r.cell), "theta": r.theta, "battery": r.battery}
                       for r in self.robots],
            "tasks": [t.to_json() for t in self.tasks],
            "blockages": self.blockages, "faults": self.faults, "meta": self.meta,
            "config": self.config,
        }

    @classmethod
    def from_json(cls, d: dict) -> "Scenario":
        return cls(
            name=d["name"], map_path=d["map"], duration=float(d.get("duration", 300)),
            robots=[RobotSpec(int(r["id"]), tuple(r["cell"]), float(r.get("theta", 0.0)),
                              float(r.get("battery", 100.0))) for r in d.get("robots", [])],
            tasks=[TaskSpec(int(t["id"]), tuple(t["pick"]), tuple(t["drop"]), float(t.get("release", 0.0)),
                            t.get("assign")) for t in d.get("tasks", [])],
            blockages=list(d.get("blockages", []) or []), faults=list(d.get("faults", []) or []),
            seed=int(d.get("seed", 0)), meta=dict(d.get("meta", {}) or {}),
            config=dict(d.get("config", {}) or {}),
        )


CONGESTION = {
    # tasks per robot, fraction of aisles used for picks, release window (s)
    "low": (2, 1.0, 60.0),
    "medium": (3, 0.6, 30.0),
    "high": (4, 0.4, 0.0),
}


def random_overlap(robots: int, congestion: str = "medium", seed: int = 0,
                   map_path: str = "maps/warehouse_a.yaml", fault: str = "none",
                   duration: float | None = None) -> Scenario:
    """General benchmark scenario: overlapping pick/drop traffic, seeded."""
    gm = GridMap.load(ROOT / map_path)
    rng = random.Random(seed * 7919 + robots * 31 + {"low": 1, "medium": 2, "high": 3}[congestion])
    per_robot, aisle_frac, window = CONGESTION[congestion]
    spots = list(gm.docks) + list(gm.bays)
    if robots > len(spots):
        raise ValueError(f"map has only {len(spots)} parking spots")
    start = rng.sample(spots, robots)
    rspecs = []
    for i, c in enumerate(start):
        theta = -math.pi / 2 if c[1] > gm.height / 2 else math.pi / 2   # face the lane
        rspecs.append(RobotSpec(i + 1, c, theta, round(rng.uniform(35.0, 100.0), 1)))
    aisles = sorted({gm.section_of(p) for p in gm.picks})
    k = max(1, round(len(aisles) * aisle_frac))
    used = set(rng.sample(aisles, k))
    picks = [p for p in gm.picks if gm.section_of(p) in used]
    tasks = []
    for t in range(robots * per_robot):
        rel = 0.0 if window <= 0 else round(rng.uniform(0.0, window), 1)
        tasks.append(TaskSpec(t + 1, rng.choice(picks), rng.choice(gm.drops), rel))
    tasks.sort(key=lambda t: (t.release, t.tid))
    faults = fault_schedule(fault, robots, seed, rng)
    dur = duration if duration is not None else 900.0   # hard cap; runs end when all tasks are done
    return Scenario(name=f"random_overlap_n{robots}_{congestion}_{fault}_s{seed}", map_path=map_path,
                    duration=dur, robots=rspecs, tasks=tasks, faults=faults, seed=seed,
                    meta={"robots": robots, "congestion": congestion, "fault": fault})


# dead zone over the busy middle lane, centre third of the map
DEAD_ZONE_RECT = [10.0, 11.0, 20.0, 13.0]


def fault_schedule(fault: str, robots: int, seed: int, rng: random.Random) -> list[dict]:
    if fault == "none":
        return []
    if fault == "dead_zone":
        return [{"t": 0.0, "action": "dead_zone", "rect": DEAD_ZONE_RECT, "until": None}]
    if fault == "node_loss":
        victim = 1 + (seed % robots)
        return [{"t": 15.0, "action": "kill", "robot": victim},
                {"t": 45.0, "action": "remove_body", "robot": victim}]
    if fault == "partition":
        ids = list(range(1, robots + 1))
        rng.shuffle(ids)
        half = len(ids) // 2
        return [{"t": 10.0, "action": "partition", "groups": [sorted(ids[:half]), sorted(ids[half:])]},
                {"t": 30.0, "action": "heal"}]
    raise ValueError(f"unknown fault {fault}")


def load_scenario(name_or_path: str, **gen) -> Scenario:
    """Load `scenarios/<name>.yaml`, a path, or generate `random_overlap`."""
    if name_or_path == "random_overlap":
        return random_overlap(**gen)
    p = Path(name_or_path)
    if not p.suffix:
        p = ROOT / "scenarios" / f"{name_or_path}.yaml"
    elif not p.is_absolute() and not p.exists():
        p = ROOT / p
    d = yaml.safe_load(p.read_text(encoding="utf-8"))
    if "generate" in d:
        g = dict(d["generate"])
        sc = random_overlap(robots=int(g.get("robots", 3)), congestion=g.get("congestion", "medium"),
                            seed=int(g.get("seed", 0)), map_path=d.get("map", "maps/warehouse_a.yaml"),
                            fault=g.get("fault", "none"), duration=d.get("duration"))
        sc.name = d.get("name", sc.name)
        sc.blockages += list(d.get("blockages", []) or [])
        sc.faults += list(d.get("faults", []) or [])
        sc.config.update(d.get("config", {}) or {})
        if "duration" in d:
            sc.duration = float(d["duration"])
        return sc
    d.setdefault("name", p.stem)
    return Scenario.from_json(d)
