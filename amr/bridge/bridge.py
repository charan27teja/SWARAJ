"""Dashboard bridge: a *passive* listener with zero authority.

    python -m amr.bridge.bridge --scenario demo

* UDP observer port 9050: receives the copies robots send (beacons, events, alerts) and the
  world's ground-truth feed. The bridge never sends a datagram to anyone.
* WebSocket 8765: pushes the aggregated fleet view to browsers at 10 Hz.
* HTTP 8080: serves the built dashboard (dashboard/dist).
Killing or restarting it has no effect on the fleet: robots send copies fire-and-forget.
"""
from __future__ import annotations

import argparse
import asyncio
import functools
import gzip
import http.server
import json
import os
import threading
import time
from pathlib import Path

import websockets

from amr.bridge.fleetview import FleetView
from amr.core.scenario import load_scenario, ROOT

_enc = json.JSONEncoder(separators=(",", ":")).encode
RUN_DIR = ROOT / "runs" / "live"
DIST = ROOT / "dashboard" / "dist"


class _Listener(asyncio.DatagramProtocol):
    def __init__(self, view: FleetView):
        self.view = view

    def datagram_received(self, data, addr):
        try:
            pkt = json.loads(data)
        except ValueError:
            return
        self.view.ingest(pkt, len(data), time.time())

    def error_received(self, exc):      # e.g. Windows ICMP resets: ignore, stay passive
        pass


def serve_static(port: int) -> None:
    root = DIST if DIST.exists() else None

    class H(http.server.SimpleHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):
            if root is None:
                body = (b"<html><body style='font-family:sans-serif;background:#0b1220;color:#e2e8f0'>"
                        b"<h2>Dashboard not built</h2><p>Run <code>cd dashboard &amp;&amp; npm install &amp;&amp; "
                        b"npm run build</code>, or use the Vite dev server (npm run dev).</p></body></html>")
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            if self.path in ("/", "") or not (root / self.path.lstrip("/").split("?")[0]).exists():
                self.path = "/index.html"
            return super().do_GET()

    handler = functools.partial(H, directory=str(root) if root else None)
    httpd = http.server.ThreadingHTTPServer(("0.0.0.0", port), handler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()


async def run(args) -> None:
    sc = load_scenario(args.scenario)
    view = FleetView(sc.map, sc.to_json(), rules_file=RUN_DIR / "faults.json")
    hello = {"type": "hello", "map": sc.map.to_json(), "scenario": {"name": sc.name, "robots": len(sc.robots),
                                                                     "tasks": len(sc.tasks)}}
    hello_txt = _enc(hello)
    loop = asyncio.get_running_loop()
    await loop.create_datagram_endpoint(lambda: _Listener(view), local_addr=("127.0.0.1", args.udp_port))
    clients: set = set()
    latest = {"txt": None}
    rec = {"frames": [], "last": 0, "t0": None, "done": False} if args.record else None

    async def ticker():
        while True:
            snap = view.snapshot(time.time())
            latest["snap"] = snap
            if rec is not None and not rec["done"]:
                record_frame(rec, snap, view, hello, args)
            for ws in list(clients):
                last = getattr(ws, "_last_alert", 0)
                msg = dict(snap)
                msg["alerts"] = view.alerts_since(last)
                if msg["alerts"]:
                    ws._last_alert = msg["alerts"][-1]["id"]
                try:
                    await ws.send(_enc(msg))
                except Exception:
                    clients.discard(ws)
            await asyncio.sleep(0.1)

    async def handler(ws):
        ws._last_alert = 0
        await ws.send(hello_txt)
        clients.add(ws)
        try:
            async for _ in ws:        # the dashboard is read-only: anything it sends is ignored
                pass
        except Exception:
            pass
        finally:
            clients.discard(ws)

    asyncio.create_task(ticker())
    async with websockets.serve(handler, "0.0.0.0", args.ws_port, max_size=2 ** 22):
        print(f"[bridge] passive: udp {args.udp_port} -> ws {args.ws_port}, http {args.http_port}", flush=True)
        await asyncio.Future()


def _slim(snap: dict) -> dict:
    """Round numbers and drop fields the dashboard does not use (keeps replay files small)."""
    s = dict(snap)
    robots = []
    for r in snap["robots"]:
        r = {k: v for k, v in r.items() if k not in ("v", "live")}
        r["x"], r["y"], r["th"] = round(r["x"], 2), round(r["y"], 2), round(r["th"], 2)
        r["age"] = round(r["age"], 1)
        robots.append(r)
    s["robots"] = robots
    s["lidar"] = {k: [round(x, 1) for x in v] for k, v in (snap.get("lidar") or {}).items()}
    return s


def record_frame(rec: dict, snap: dict, view: FleetView, hello: dict, args) -> None:
    """--record: keep the frames sent to browsers (with their time) and write them gzipped
    after --record-seconds of activity (the process may be killed later, so write early)."""
    if not snap["robots"]:
        return                                   # start the clock at the first robot beacon
    now = time.time()
    if rec["t0"] is None:
        rec["t0"] = now
    alerts = view.alerts_since(rec["last"])
    if alerts:
        rec["last"] = alerts[-1]["id"]
    frame = _slim(snap)
    frame["alerts"] = alerts
    rec["frames"].append([round(now - rec["t0"], 2), frame])
    if now - rec["t0"] >= args.record_seconds:
        rec["done"] = True
        out = {"hello": hello, "frames": rec["frames"]}
        path = Path(args.record)
        path.parent.mkdir(parents=True, exist_ok=True)
        with gzip.open(path, "wt", encoding="utf-8", compresslevel=9) as f:
            f.write(_enc(out))
        print(f"[bridge] recorded {len(rec['frames'])} frames -> {path} ({path.stat().st_size // 1024} kB)",
              flush=True)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", default="demo")
    ap.add_argument("--udp-port", type=int, default=9050)
    ap.add_argument("--ws-port", type=int, default=8765)
    ap.add_argument("--http-port", type=int, default=8080)
    ap.add_argument("--no-http", action="store_true")
    ap.add_argument("--record", default=None, help="save the frames sent to browsers to this .json.gz file")
    ap.add_argument("--record-seconds", type=float, default=80.0)
    args = ap.parse_args(argv)
    RUN_DIR.mkdir(parents=True, exist_ok=True)
    (RUN_DIR / "bridge.pid").write_text(str(os.getpid()))
    if not args.no_http:
        serve_static(args.http_port)
    asyncio.run(run(args))


if __name__ == "__main__":
    main()
