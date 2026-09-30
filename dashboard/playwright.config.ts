import { defineConfig } from "@playwright/test";

// The Python live system (world + robots + passive bridge serving dashboard/dist on :8080) is
// started by the specs themselves (tests/fleet.ts), so no webServer entry is needed.
export default defineConfig({
  testDir: "./tests",
  timeout: 120_000,
  workers: 1,
  fullyParallel: false,
  retries: 0,
  reporter: [["list"]],
  use: {
    baseURL: "http://localhost:8080",
    headless: true,
    viewport: { width: 1920, height: 1080 },
  },
});
