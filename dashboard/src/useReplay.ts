import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import type { Alert, ConnStatus, FleetState, Hello } from "./types";

// Recorded runs of the real system (saved by the bridge with --record) played back into the same
// state the live WebSocket feeds. Used on the static demo site (no backend).

export interface ReplayItem {
  id: string;
  file: string;
  title: string;
  desc: string;
}

export interface ReplayCtl {
  list: ReplayItem[];
  id: string | null;
  select: (id: string) => void;
  playing: boolean;
  setPlaying: (p: boolean) => void;
  speed: number;
  setSpeed: (s: number) => void;
  t: number;
  duration: number;
  seek: (t: number) => void;
  error: string | null;
  started: boolean;
  ended: boolean;
  start: () => void;
  restart: () => void;
}

type Frame = [number, FleetState];

/** Replay mode: `?replay=<id>` or any page not served from localhost (the public demo site). */
export function isReplayMode(): boolean {
  const q = new URLSearchParams(window.location.search);
  if (q.has("replay")) return true;
  if (q.has("live")) return false;
  const h = window.location.hostname;
  return !(h === "localhost" || h === "127.0.0.1" || h === "" || h === "[::1]");
}

async function fetchJsonMaybeGzip(url: string): Promise<unknown> {
  const r = await fetch(url);
  if (!r.ok) throw new Error(`${url}: HTTP ${r.status}`);
  const buf = await r.arrayBuffer();
  const b = new Uint8Array(buf);
  let text: string;
  if (b[0] === 0x1f && b[1] === 0x8b) {
    const stream = new Blob([buf]).stream().pipeThrough(new DecompressionStream("gzip"));
    text = await new Response(stream).text();
  } else {
    text = new TextDecoder().decode(buf);           // host already decoded Content-Encoding
  }
  return JSON.parse(text);
}

export function useReplay(enabled: boolean) {
  const base = import.meta.env.BASE_URL;
  const [list, setList] = useState<ReplayItem[]>([]);
  const [id, setId] = useState<string | null>(null);
  const [hello, setHello] = useState<Hello | null>(null);
  const [state, setState] = useState<FleetState | null>(null);
  const [alerts, setAlerts] = useState<Alert[]>([]);
  const [playing, setPlaying] = useState(false);
  const [started, setStarted] = useState(false);
  const [speed, setSpeed] = useState(1);
  const [t, setT] = useState(0);
  const [error, setError] = useState<string | null>(null);
  const frames = useRef<Frame[]>([]);
  const flat = useRef<Alert[]>([]);
  const prefix = useRef<number[]>([]);
  const shown = useRef(-1);
  const tRef = useRef(0);
  const introDone = useRef(false);   // true after the first Start: the Start card shows only once per page load

  // the list of recordings
  useEffect(() => {
    if (!enabled) return;
    fetchJsonMaybeGzip(`${base}replays/index.json`)
      .then((d) => {
        const items = (d as { replays: ReplayItem[] }).replays;
        setList(items);
        const want = new URLSearchParams(window.location.search).get("replay");
        // Default to "demo" scenario if no URL parameter specified
        const defaultId = want ? items.find((i) => i.id === want)?.id : items.find((i) => i.id === "demo")?.id ?? items[0]?.id;
        setId(defaultId ?? null);
      })
      .catch((e) => setError(String(e)));
  }, [enabled, base]);

  const render = useCallback((time: number) => {
    const f = frames.current;
    if (!f.length) return;
    let lo = 0, hi = f.length - 1;
    while (lo < hi) {                                  // last frame with frame time <= time
      const mid = (lo + hi + 1) >> 1;
      if (f[mid][0] <= time) lo = mid; else hi = mid - 1;
    }
    if (lo === shown.current) return;
    shown.current = lo;
    setState(f[lo][1]);
    setAlerts(flat.current.slice(Math.max(0, prefix.current[lo] - 200), prefix.current[lo]));
  }, []);

  // load one recording
  useEffect(() => {
    if (!enabled || !id) return;
    const item = list.find((i) => i.id === id);
    if (!item) return;
    let cancelled = false;
    setHello(null);
    setState(null);
    fetchJsonMaybeGzip(`${base}replays/${item.file}`)
      .then((d) => {
        if (cancelled) return;
        const rec = d as { hello: Hello; frames: Frame[] };
        frames.current = rec.frames;
        const all: Alert[] = [];
        const pre: number[] = [];
        for (const [, fr] of rec.frames) {
          for (const a of fr.alerts ?? []) all.push(a);
          pre.push(all.length);
        }
        flat.current = all;
        prefix.current = pre;
        shown.current = -1;
        tRef.current = 0;
        setT(0);
        setHello(rec.hello);
        render(0);
        // First visit: first frame shown, waiting for the Start card. Once the user has started
        // once, switching modes plays the new one straight away (no Start card again).
        setPlaying(introDone.current);
        setStarted(introDone.current);
        // the chosen mode is not written into the URL, so a reload always opens the demo
      })
      .catch((e) => !cancelled && setError(String(e)));
    return () => {
      cancelled = true;
    };
  }, [enabled, id, list, base, render]);

  const duration = hello && frames.current.length ? frames.current[frames.current.length - 1][0] : 0;

  // playback clock
  useEffect(() => {
    if (!enabled || !playing || !duration) return;
    let last = performance.now();
    const h = window.setInterval(() => {
      const now = performance.now();
      let nt = tRef.current + ((now - last) / 1000) * speed;
      last = now;
      if (nt >= duration) {
        nt = duration;
        setPlaying(false);
      }
      tRef.current = nt;
      setT(nt);
      render(nt);
    }, 50);
    return () => window.clearInterval(h);
  }, [enabled, playing, speed, duration, render]);

  const ended = started && duration > 0 && t >= duration - 0.15;    // within one slider step

  const seek = useCallback((nt: number) => {
    tRef.current = nt >= duration - 0.15 ? duration : Math.max(0, nt);   // slider snaps to 0.1 s
    setT(tRef.current);
    shown.current = -1;
    render(tRef.current);
  }, [duration, render]);

  const play = useCallback((p: boolean) => {
    if (p && tRef.current >= duration) seek(0);      // replay from the start
    setPlaying(p);
  }, [duration, seek]);

  const start = useCallback(() => {
    introDone.current = true;
    setStarted(true);
    play(true);
  }, [play]);
  const restart = useCallback(() => {
    seek(0);
    setStarted(true);
    setPlaying(true);
  }, [seek]);
  const select = useCallback((nid: string) => {
    // after the first Start, switching modes keeps playing; before it, stay on the Start card
    setStarted(introDone.current);
    setPlaying(introDone.current);
    if (nid === id) {
      seek(0);
    } else {
      setId(nid);
    }
  }, [id, seek]);

  const replay: ReplayCtl = useMemo(() => ({
    list, id, select, playing, setPlaying: play, speed, setSpeed, t, duration, seek, error,
    started, ended, start, restart,
  }), [list, id, select, playing, play, speed, t, duration, seek, error, started, ended, start, restart]);

  const status: ConnStatus = hello ? "replay" : "connecting";
  return { status, hello, state, alerts, attempt: 0, lastMsg: 0, url: "", replay };
}
