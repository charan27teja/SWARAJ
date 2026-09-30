"""Fast runtime (benchmark): all agents stepped in lockstep in one process, with an in-memory
message bus that emulates latency / jitter / loss / partitions / dead zones through the same
FaultShim and ReliableEndpoint code the UDP transport uses. Runs much faster than real time.

The bus only moves packets between robot endpoints (it is the "air"); it makes no decisions.
Observers (metrics) get copies, exactly like the dashboard bridge in the live runtime.
"""
from __future__ import annotations

import heapq
import json
import math
import time
from dataclasses import dataclass

from amr.core.config import Config, DEFAULT
from amr.core.scenario import Scenario
from amr.transport.faultshim import FaultShim
from amr.transport.reliable import ReliableEndpoint
from amr.world.world import World
from amr.bench.metrics import MetricsRecorder

_dumps = json.JSONEncoder(separators=(",", ":")).encode


@dataclass
class NetParams:
    latency_s: float = 0.005
    jitter_s: float = 0.005
    loss: float = 0.01


class Bus:
    def __init__(self, ids: list[int], shim: FaultShim, net: NetParams, seed: int, recorder: MetricsRecorder):
        import random
        self.ids = list(ids)
        self.shim = shim
        self.net = net
        self.rng = random.Random(seed ^ 0x5EED)
        self.eps = {r: ReliableEndpoint(r) for r in ids}
        self.q: dict[int, list] = {r: [] for r in ids}
        self.pos: dict[int, tuple[float, float]] = {}
        self.down: set[int] = set()
        self.rec = recorder
        self._n = 0

    def _xmit(self, src: int, dst: int, pkt: dict, now: float, size: int) -> None:
        self.rec.on_sent(src, size)
        if src in self.down or dst in self.down:
            return
        lat = self.shim.deliver(src, dst, self.pos.get(src), self.pos.get(dst))
        if lat is None:
            return
        if self.net.loss and self.rng.random() < self.net.loss:
            return
        t = now + self.net.latency_s + self.rng.uniform(0, self.net.jitter_s) + lat
        self._n += 1
        heapq.heappush(self.q[dst], (t, self._n, src, pkt))

    def send(self, src: int, outbox: list, now: float) -> None:
        for cls, dst, body in outbox:
            if cls == "B":
                pkt = {"t": "B", "b": body}
                size = len(_dumps(pkt))
                self.rec.beacon_bytes.append(size)
                self.rec.on_message(src, body, now)
                for d in self.ids:
                    if d != src:
                        self._xmit(src, d, pkt, now, size)
            elif cls == "E":
                self.rec.on_message(src, body, now)
                dsts = [d for d in self.ids if d != src] if dst is None else [dst]
                size = len(_dumps(body)) + 40
                for d in dsts:
                    pkt = self.eps[src].send(d, body, now)
                    self._xmit(src, d, pkt, now, size)
            elif cls == "U":                  # unicast best-effort (baseline status / commands)
                size = len(_dumps(body)) + 20
                self._xmit(src, dst, {"t": "B", "b": body}, now, size)
            elif cls == "O":
                self.rec.on_message(body.get("by", src), body, now)

    def retransmit(self, now: float) -> None:
        for src, ep in self.eps.items():
            if src in self.down:
                continue
            for pkt in ep.due(now):
                self._xmit(src, pkt["dst"], pkt, now, 60)

    def deliver(self, dst: int, now: float) -> list:
        inbox = []
        q = self.q[dst]
        ep = self.eps[dst]
        while q and q[0][0] <= now:
            _, _, src, pkt = heapq.heappop(q)
            t = pkt.get("t")
            if t == "B":
                inbox.append((src, pkt["b"]))
            else:
                body, ack = ep.on_packet(pkt)
                if ack is not None:
                    self._xmit(dst, src, ack, now, 30)
                if body is not None:
                    inbox.append((src, body))
        return inbox


class FastSim:
    def __init__(self, scenario: Scenario, system: str = "ours", cfg: Config = DEFAULT, alloc: str = "cbba",
                 net: NetParams | None = None, seed: int | None = None, dt: float | None = None,
                 on_step=None):
        self.sc = scenario
        if scenario.config:
            cfg = cfg.with_(**scenario.config)
        self.cfg = cfg
        self.system = system
        self.dt = dt or 1.0 / cfg.control_hz
        self.seed = scenario.seed if seed is None else seed
        gm = scenario.map
        self.map = gm
        self.world = World(gm, cfg, seed=self.seed)
        ids = [r.rid for r in scenario.robots]
        self.ids = ids
        for r in scenario.robots:
            self.world.add_robot(r.rid, r.cell[0] + 0.5, r.cell[1] + 0.5, r.theta, r.battery)
        self.rec = MetricsRecorder(len(scenario.tasks), ids)
        self.shim = FaultShim({}, seed=self.seed)
        self.net = net or NetParams()
        self.bus = Bus(ids, self.shim, self.net, self.seed, self.rec)
        self.alive = set(ids)
        self.t = 0.0
        self.step_n = 0
        self.agent_time = 0.0
        self.agent_steps = 0
        self.on_step = on_step
        self.central = None
        if system == "ours":
            from amr.agent.agent import Agent
            self.agents = {r: Agent(r, gm, cfg, fleet=ids, tasks=scenario.tasks, alloc=alloc, seed=self.seed)
                           for r in ids}
        elif system == "b0":
            from amr.baseline.b0 import B0Central, B0Robot, CENTRAL
            self.central = B0Central(gm, cfg, ids, scenario.tasks, seed=self.seed)
            self.agents = {r: B0Robot(r, gm, cfg) for r in ids}
            self.bus.ids.append(CENTRAL)
            self.bus.eps[CENTRAL] = ReliableEndpoint(CENTRAL)
            self.bus.q[CENTRAL] = []
        else:
            raise ValueError(system)
        self.events = sorted(
            [dict(f, _kind="fault") for f in scenario.faults]
            + [dict(b, _kind="block") for b in scenario.blockages]
            + [dict(t=f["until"], action="_end", ref=f) for f in scenario.faults if f.get("until") is not None]
            + [dict(t=b["until"], action="_unblock", ref=b) for b in scenario.blockages if b.get("until") is not None],
            key=lambda e: e["t"])
        self._ev_i = 0
        self._deferred: list[dict] = []           # conditional faults waiting for their condition
        self._sec_occ: set = set()
        self.section_violations = 0
        self._scans: dict = {}
        self._block_ids: dict[int, int] = {}
        self.rules: dict = {}

    # ------------------------------------------------------------------ faults
    def _in_section(self, rid: int) -> bool:
        b = self.world.bodies[rid]
        return self.map.section_of(self.map.cell_of(b.x, b.y)) is not None

    def _apply_events(self) -> None:
        due = [e for e in self._deferred if self._in_section(e["robot"])]
        for e in due:
            self._deferred.remove(e)
        while self._ev_i < len(self.events) and self.events[self._ev_i]["t"] <= self.t + 1e-9:
            e = self.events[self._ev_i]
            self._ev_i += 1
            if e.get("when") == "in_aisle" and not self._in_section(e["robot"]):
                self._deferred.append(e)
                continue
            due.append(e)
        for e in due:
            a = e.get("action")
            if e.get("_kind") == "block":
                c = e["cell"]
                self._block_ids[id(e)] = self.world.add_blockage(c[0] + 0.5, c[1] + 0.5, e.get("half", 0.4))
                self.rec.on_fault({"action": "blockage", "cell": c}, self.t)
                continue
            if a == "_unblock":
                bid = self._block_ids.pop(id(e["ref"]), None)
                if bid:
                    self.world.remove_blockage(bid)
                continue
            if a == "kill":
                r = e["robot"]
                self.alive.discard(r)
                self.world.kill(r)
                self.bus.down.add(r)
                if self.central:
                    self.central.robot_killed(r)
            elif a == "remove_body":
                self.world.remove_body(e["robot"])
                if self.central:
                    self.central.body_removed(e["robot"])
            elif a == "partition":
                groups = [list(g) for g in e["groups"]]
                if self.central is not None:
                    groups[0] = groups[0] + [0]      # the server sits in the first island
                self.rules["partition"] = groups
            elif a == "heal":
                self.rules.pop("partition", None)
            elif a == "dead_zone":
                self.rules.setdefault("dead_zones", []).append(e["rect"])
            elif a == "loss":
                self.rules["loss"] = e["p"]
            elif a == "delay":
                self.rules["delay_ms"] = e["ms"]
                self.rules["jitter_ms"] = e.get("jitter_ms", 0)
            elif a == "_end":
                f = e["ref"]
                fa = f.get("action")
                if fa == "dead_zone":
                    dz = self.rules.get("dead_zones", [])
                    if f["rect"] in dz:
                        dz.remove(f["rect"])
                elif fa == "loss":
                    self.rules.pop("loss", None)
                elif fa == "delay":
                    self.rules.pop("delay_ms", None)
                    self.rules.pop("jitter_ms", None)
                elif fa == "partition":
                    self.rules.pop("partition", None)
            self.shim.set_rules(self.rules)
            if a and not a.startswith("_"):
                self.rec.on_fault(e, self.t)

    # ------------------------------------------------------------------ loop
    def step(self) -> None:
        dt, cfg = self.dt, self.cfg
        self._apply_events()
        w = self.world
        for r in self.ids:
            b = w.bodies[r]
            self.bus.pos[r] = (b.x, b.y)
        if self.step_n % max(1, round(cfg.control_hz / cfg.lidar_hz)) == 0:
            sc = w.scans(sorted(self.alive))
            self._scans = {r: {"ranges": v, "pose": w.pose(r), "t": self.t} for r, v in sc.items()}
        else:
            self._scans = {}
        self.bus.retransmit(self.t)
        if self.central is not None:
            self.central.step(self.t, dt, w, self.bus, self.alive)
        t0 = time.perf_counter()
        for r in self.ids:
            if r not in self.alive:
                continue
            inbox = self.bus.deliver(r, self.t)
            b = w.bodies[r]
            cmd, out = self.agents[r].step(self.t, dt, (b.x, b.y, b.th), self._scans.get(r), inbox, b.battery)
            w.set_cmd(r, cmd[0], cmd[1])
            if out:
                self.bus.send(r, out, self.t)
        self.agent_time += time.perf_counter() - t0
        self.agent_steps += len(self.alive)
        w.step(dt)
        self.t += dt
        self.step_n += 1
        if self.step_n % 5 == 0:
            self._check_sections()
        self.rec.tick(self.t)
        if self.step_n % 20 == 0:
            self.rec.check_membership(self.t, sorted(self.alive))
        if self.on_step:
            self.on_step(self)

    def _check_sections(self) -> None:
        """Ground truth: two robot bodies (centres) inside the same critical section at once."""
        occ: dict[int, list[int]] = {}
        for r, b in self.world.bodies.items():
            if b.present:
                sid = self.map.section_of(self.map.cell_of(b.x, b.y))
                if sid is not None:
                    occ.setdefault(sid, []).append(r)
        now = {sid for sid, rs in occ.items() if len(rs) > 1}
        self.section_violations += len(now - self._sec_occ)
        self._sec_occ = now

    def run(self, until: float | None = None, stop_when_done: bool = True) -> dict:
        end = self.sc.duration if until is None else until
        wall0 = time.perf_counter()
        while self.t < end:
            self.step()
            if stop_when_done and self.rec.all_done():
                break
        res = self.rec.summary(self.t, self.world.truth(), sorted(self.alive))
        res.update({
            "system": self.system, "scenario": self.sc.name, "seed": self.seed,
            "robots": len(self.ids), "wall_s": round(time.perf_counter() - wall0, 2),
            "section_violations": self.section_violations,
            "agent_us_per_step": round(1e6 * self.agent_time / max(1, self.agent_steps), 1),
        })
        res.update({k: v for k, v in self.sc.meta.items() if k not in res})
        return res


def run_scenario(scenario: Scenario, system: str = "ours", **kw) -> dict:
    return FastSim(scenario, system=system, **kw).run()
