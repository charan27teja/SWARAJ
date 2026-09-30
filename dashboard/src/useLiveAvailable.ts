import { useEffect, useState } from "react";

function bridgeUrl(): string {
  const q = new URLSearchParams(window.location.search).get("ws");
  if (q) return q;
  const host = window.location.hostname || "localhost";
  return `ws://${host}:8765`;
}

/** Try to detect if the live bridge is available (1.5s timeout). Returns true if connected. */
export function useLiveAvailable() {
  const [available, setAvailable] = useState(false);

  useEffect(() => {
    let timeout: number;
    let ws: WebSocket | null = null;
    const url = bridgeUrl();

    try {
      ws = new WebSocket(url);
      ws.onopen = () => {
        setAvailable(true);
        ws?.close();
        ws = null;
      };
      ws.onerror = () => {
        ws?.close();
        ws = null;
      };

      timeout = window.setTimeout(() => {
        if (ws) {
          ws.close();
          ws = null;
        }
        setAvailable(false);
      }, 1500);
    } catch {
      setAvailable(false);
    }

    return () => {
      window.clearTimeout(timeout);
      if (ws) {
        ws.close();
      }
    };
  }, []);

  return available;
}
