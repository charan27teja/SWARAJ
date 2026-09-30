import { useEffect, useRef, useState } from "react";
import type { Alert, ConnStatus, FleetState, Hello } from "./types";

const ALERT_CAP = 200;

function bridgeUrl(): string {
  const q = new URLSearchParams(window.location.search).get("ws");
  if (q) return q;
  const host = window.location.hostname || "localhost";
  return `ws://${host}:8765`;
}

/** Subscribes to the passive bridge. Read-only: nothing is ever sent on the socket.
 *  Reconnects forever with capped back-off; keeps the last known state while disconnected. */
export function useFleet(enabled = true) {
  const [status, setStatus] = useState<ConnStatus>("connecting");
  const [hello, setHello] = useState<Hello | null>(null);
  const [state, setState] = useState<FleetState | null>(null);
  const [alerts, setAlerts] = useState<Alert[]>([]);
  const [attempt, setAttempt] = useState(0);
  const [lastMsg, setLastMsg] = useState<number>(0);
  const wsRef = useRef<WebSocket | null>(null);
  const everLive = useRef(false);

  useEffect(() => {
    if (!enabled) return;
    let closed = false;
    let timer: number | undefined;
    let tries = 0;
    const url = bridgeUrl();

    const connect = () => {
      if (closed) return;
      tries += 1;
      setAttempt(tries);
      let ws: WebSocket;
      try {
        ws = new WebSocket(url);
      } catch {
        schedule();
        return;
      }
      wsRef.current = ws;
      ws.onopen = () => {
        tries = 0;
      };
      ws.onmessage = (ev) => {
        let m: Hello | FleetState;
        try {
          m = JSON.parse(ev.data as string);
        } catch {
          return;
        }
        if (m.type === "hello") {
          setHello(m);
          setAlerts([]);          // a (re)connected bridge replays its own alert history
          everLive.current = true;
          setStatus("live");
        } else if (m.type === "state") {
          setState(m);
          setLastMsg(Date.now());
          if (m.alerts && m.alerts.length) {
            setAlerts((prev) => {
              const seen = new Set(prev.map((a) => a.id));
              const add = m.alerts.filter((a) => !seen.has(a.id));
              const next = prev.concat(add);
              return next.length > ALERT_CAP ? next.slice(next.length - ALERT_CAP) : next;
            });
          }
        }
      };
      ws.onclose = () => {
        wsRef.current = null;
        setStatus(everLive.current ? "disconnected" : "connecting");
        schedule();
      };
      ws.onerror = () => {
        try {
          ws.close();
        } catch {
          /* ignore */
        }
      };
    };

    const schedule = () => {
      if (closed) return;
      const delay = Math.min(5000, 700 * Math.max(1, tries));
      timer = window.setTimeout(connect, delay);
    };

    connect();
    return () => {
      closed = true;
      if (timer) window.clearTimeout(timer);
      wsRef.current?.close();
    };
  }, [enabled]);

  return { status, hello, state, alerts, attempt, lastMsg, url: bridgeUrl() };
}
