"""Fault shim: drop / delay / jitter / partition / dead-zone rules applied inside the transport.

This is the Windows substitute for Linux `tc netem`. It is *test tooling*: it sits in the
transport below the agent, and robot logic never reads it. Rules come from a JSON file written
by `python -m amr.faults ...` (live runtime) or from the scenario's fault schedule (fast runtime).

Rules dict (all keys optional):
    {"loss": 0.1,                       # drop probability for every robot-robot packet
     "loss_by_robot": {"3": 0.5},       # extra per-robot loss (either end)
     "delay_ms": 50, "jitter_ms": 20,   # one-way latency added
     "partition": [[1, 2], [3, 4]],     # robots in different groups cannot talk
     "dead_zones": [[x0, y0, x1, y1]],  # a robot inside any rect has no link at all
     "down": [2]}                       # robots whose link is fully down
Only robot<->robot links are affected. Observers (dashboard bridge, metrics recorder) are
instruments, not participants, and always receive the copies robots send them.
"""
from __future__ import annotations

import json
import random
from pathlib import Path


class FaultShim:
    def __init__(self, rules: dict | None = None, seed: int = 0):
        self.rules: dict = rules or {}
        self.rng = random.Random(seed)

    def set_rules(self, rules: dict | None) -> None:
        self.rules = rules or {}

    # ------------------------------------------------------------------ queries
    def in_dead_zone(self, pos: tuple[float, float] | None) -> bool:
        if pos is None:
            return False
        for x0, y0, x1, y1 in self.rules.get("dead_zones", []) or []:
            if x0 <= pos[0] <= x1 and y0 <= pos[1] <= y1:
                return True
        return False

    def link_up(self, src: int, dst: int, pos_src=None, pos_dst=None) -> bool:
        r = self.rules
        down = r.get("down") or []
        if src in down or dst in down:
            return False
        part = r.get("partition")
        if part:
            gs = gd = None
            for gi, g in enumerate(part):
                if src in g:
                    gs = gi
                if dst in g:
                    gd = gi
            if gs is not None and gd is not None and gs != gd:
                return False
        if self.in_dead_zone(pos_src) or self.in_dead_zone(pos_dst):
            return False
        return True

    def deliver(self, src: int, dst: int, pos_src=None, pos_dst=None) -> float | None:
        """Return the added one-way latency in seconds, or None if the packet is dropped."""
        if not self.rules:
            return 0.0
        if not self.link_up(src, dst, pos_src, pos_dst):
            return None
        r = self.rules
        p = float(r.get("loss", 0.0) or 0.0)
        lbr = r.get("loss_by_robot") or {}
        p = max(p, float(lbr.get(str(src), 0.0)), float(lbr.get(str(dst), 0.0)))
        if p > 0 and self.rng.random() < p:
            return None
        d = float(r.get("delay_ms", 0.0) or 0.0)
        j = float(r.get("jitter_ms", 0.0) or 0.0)
        if d or j:
            return max(0.0, (d + self.rng.uniform(-j, j)) / 1000.0)
        return 0.0


class FileRules:
    """Polls a fault-rules JSON file (mtime-based) for the live runtime."""

    def __init__(self, path: str | Path, shim: FaultShim, poll_s: float = 0.25):
        self.path = Path(path)
        self.shim = shim
        self.poll_s = poll_s
        self._mtime = None
        self._next = 0.0

    def poll(self, now: float) -> None:
        if now < self._next:
            return
        self._next = now + self.poll_s
        try:
            m = self.path.stat().st_mtime
        except OSError:
            if self._mtime is not None:
                self.shim.set_rules({})
                self._mtime = None
            return
        if m != self._mtime:
            try:
                self.shim.set_rules(json.loads(self.path.read_text(encoding="utf-8") or "{}"))
                self._mtime = m
            except (OSError, ValueError):
                pass   # half-written file; retry next poll
