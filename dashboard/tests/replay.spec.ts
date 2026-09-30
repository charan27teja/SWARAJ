import { test, expect } from "@playwright/test";
import { spawn, spawnSync, type ChildProcess } from "node:child_process";
import { waitHttp } from "./fleet";

// Static demo site: `vite preview` of the production build, no Python backend at all.
const URL = "http://localhost:4173";
let preview: ChildProcess | null = null;

test.beforeAll(async () => {
  preview = spawn("npx", ["vite", "preview", "--port", "4173", "--strictPort"], { shell: true, stdio: "ignore" });
  await waitHttp(`${URL}/`);
});

test.afterAll(() => {
  if (preview?.pid) {
    if (process.platform === "win32") spawnSync("taskkill", ["/pid", String(preview.pid), "/T", "/F"]);
    else preview.kill();
  }
});

async function positions(page) {
  return page.locator('g[role="button"][aria-label^="Robot "]').evaluateAll((els) => els.map((e) => e.getAttribute("transform")));
}

test("a recorded replay loads with no backend, shows 5 robots and REPLAY, and robots move", async ({ page }) => {
  await page.goto(`${URL}/?replay=demo`);
  await expect(page.getByTestId("conn-status")).toHaveText(/REPLAY/, { timeout: 20000 });
  await expect(page.getByTestId("sim-controls")).toBeVisible();
  await expect(page.locator('g[role="button"][aria-label^="Robot "]')).toHaveCount(5);
  await page.getByRole("button", { name: "Start simulation" }).click();
  for (const id of [1, 2, 3, 4, 5]) await expect(page.getByTestId(`robot-card-${id}`)).toBeVisible();
  await page.getByRole("button", { name: "4×" }).click();
  const before = await positions(page);
  await page.waitForTimeout(2500);
  expect((await positions(page)).some((t, i) => t !== before[i])).toBeTruthy();
  // pause stops the clock
  await page.getByTestId("replay-play").click();
  const t1 = await page.getByLabel("Playback position").inputValue();
  await page.waitForTimeout(800);
  expect(await page.getByLabel("Playback position").inputValue()).toBe(t1);
});

test("Start button: robots move and the KPI strip appears; end shows Run complete", async ({ page }) => {
  await page.goto(`${URL}/?replay=circular_wait`);
  await expect(page.getByTestId("start-card")).toBeVisible({ timeout: 20000 });
  await expect(page.getByTestId("kpi-tasks")).toContainText("—");          // placeholders before Start
  const before = await positions(page);
  await page.waitForTimeout(1000);
  expect(await positions(page)).toEqual(before);                             // nothing plays before Start
  await page.getByRole("button", { name: "Start simulation" }).click();
  await expect(page.getByTestId("start-card")).toBeHidden();
  await expect(page.getByTestId("kpi-tasks")).toContainText(/\d/);
  await expect(page.getByTestId("kpi-collisions")).toContainText("0");
  await page.waitForTimeout(2000);
  expect((await positions(page)).some((t, i) => t !== before[i])).toBeTruthy();
  // jump to the end: "Run complete" card, no auto-loop
  const slider = page.getByLabel("Playback position");
  await slider.focus();
  await slider.press("End");
  await expect(page.getByTestId("complete-card")).toBeVisible({ timeout: 5000 });
  await page.getByRole("button", { name: "Try another scenario" }).click();
  await expect(page.getByTestId("start-card")).toBeVisible();              // back to pre-Start
});
