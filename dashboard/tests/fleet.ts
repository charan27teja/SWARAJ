import { spawn, spawnSync, type ChildProcess } from "node:child_process";
import path from "node:path";
import fs from "node:fs";
import { fileURLToPath } from "node:url";

// Starts the real Python live system (world + robot processes + passive bridge serving the built
// dashboard on :8080) for a scenario, and tears the whole process tree down afterwards.
export const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..", "..");
const VENV_PY = path.join(ROOT, ".venv", process.platform === "win32" ? "Scripts/python.exe" : "bin/python");
export const PY = process.env.PYTHON ?? (fs.existsSync(VENV_PY) ? VENV_PY : "python");

function killTree(p: ChildProcess | null) {
  if (!p || p.pid == null || p.exitCode != null) return;
  if (process.platform === "win32") spawnSync("taskkill", ["/pid", String(p.pid), "/T", "/F"]);
  else p.kill("SIGKILL");
}

export async function waitHttp(url: string, timeoutMs = 30000) {
  const end = Date.now() + timeoutMs;
  while (Date.now() < end) {
    try {
      const r = await fetch(url);
      if (r.ok) return;
    } catch {
      /* not up yet */
    }
    await new Promise((r) => setTimeout(r, 300));
  }
  throw new Error(`timeout waiting for ${url}`);
}

export class Fleet {
  proc: ChildProcess | null = null;
  bridge: ChildProcess | null = null;
  constructor(public scenario: string) {}

  async start() {
    this.proc = spawn(PY, ["-m", "amr.runtime.launcher", "--scenario", this.scenario, "--quiet"], {
      cwd: ROOT, env: { ...process.env, PYTHONPATH: ROOT }, stdio: "ignore",
    });
    await waitHttp("http://localhost:8080/");
  }

  /** kill only the bridge process (the fleet must be unaffected) */
  killBridge() {
    const r = spawnSync(PY, ["-m", "amr.faults", "kill", "bridge"], { cwd: ROOT, env: { ...process.env, PYTHONPATH: ROOT } });
    return r.status === 0;
  }

  startBridge() {
    this.bridge = spawn(PY, ["-m", "amr.bridge.bridge", "--scenario", this.scenario], {
      cwd: ROOT, env: { ...process.env, PYTHONPATH: ROOT }, stdio: "ignore",
    });
  }

  robotPids(): number[] {
    const dir = path.join(ROOT, "runs", "live");
    return fs.readdirSync(dir).filter((f) => /^robot_\d+\.pid$/.test(f))
      .map((f) => Number(fs.readFileSync(path.join(dir, f), "utf8")));
  }

  stop() {
    killTree(this.bridge);
    killTree(this.proc);
    this.bridge = null;
    this.proc = null;
  }
}

export function alive(pid: number): boolean {
  try {
    process.kill(pid, 0);
    return true;
  } catch {
    return false;
  }
}
