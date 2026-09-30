import { test, expect } from "@playwright/test";
import { Fleet, alive } from "./fleet";

test.describe.configure({ mode: "serial" });

test.describe("five robots", () => {
  const fleet = new Fleet("demo");
  test.beforeAll(async () => { await fleet.start(); });
  test.afterAll(() => fleet.stop());

  test("page loads and connects", async ({ page }) => {
    await page.goto("/");
    await expect(page.getByTestId("conn-status")).toHaveText(/LIVE/, { timeout: 20000 });
    await expect(page.getByTestId("kpi-tasks")).toBeVisible();
    await expect(page.getByTestId("kpi-collisions")).toContainText("0");
    await expect(page.getByRole("region", { name: "Live warehouse map" })).toBeVisible();
    // no page scroll at 1920x1080
    const scroll = await page.evaluate(() => document.documentElement.scrollHeight - window.innerHeight);
    expect(scroll).toBeLessThanOrEqual(0);
  });

  test("robots appear", async ({ page }) => {
    await page.goto("/");
    for (const id of [1, 2, 3, 4, 5]) {
      await expect(page.getByTestId(`robot-card-${id}`)).toBeVisible({ timeout: 20000 });
      await expect(page.getByRole("button", { name: new RegExp(`^Robot ${id},`) })).toBeVisible();
    }
    // hover tooltip and click-to-select
    await page.getByRole("button", { name: /^Robot 2,/ }).hover();
    await expect(page.getByRole("tooltip")).toContainText("Battery");
    await page.getByRole("button", { name: /^Robot 2,/ }).dispatchEvent("pointerup");
    await expect(page.getByTestId("robot-card-2")).toHaveAttribute("aria-pressed", "true");
  });

  test("disconnected banner appears when the bridge stops and clears when it restarts; fleet unaffected", async ({ page }) => {
    await page.goto("/");
    await expect(page.getByTestId("conn-status")).toHaveText(/LIVE/, { timeout: 20000 });
    const pids = fleet.robotPids();
    expect(pids.length).toBe(5);
    expect(fleet.killBridge()).toBeTruthy();
    await expect(page.getByTestId("disconnected-banner")).toBeVisible({ timeout: 10000 });
    await expect(page.getByTestId("disconnected-banner")).toContainText("fleet unaffected");
    await page.waitForTimeout(3000);
    for (const pid of pids) expect(alive(pid)).toBeTruthy();       // robots never noticed
    fleet.startBridge();
    await expect(page.getByTestId("disconnected-banner")).toBeHidden({ timeout: 20000 });
    await expect(page.getByTestId("conn-status")).toHaveText(/LIVE/);
    await expect(page.getByTestId("robot-card-1")).toBeVisible();
  });
});

test.describe("deadlock scenario", () => {
  const fleet = new Fleet("circular_wait");
  test.beforeAll(async () => { await fleet.start(); });
  test.afterAll(() => fleet.stop());

  test("an alert appears when a deadlock is detected and broken", async ({ page }) => {
    await page.goto("/");
    await expect(page.getByTestId("conn-status")).toHaveText(/LIVE/, { timeout: 20000 });
    await page.getByRole("tab", { name: /Alerts/ }).click();
    const alert = page.locator('[data-alert-type="deadlock_broken"], [data-alert-type="deadlock_detected"]').first();
    await expect(alert).toBeVisible({ timeout: 60000 });
    await expect(alert).toContainText(/deadlock|cycle/i);
    await expect(page.getByTestId("kpi-deadlocks")).not.toContainText(/^\s*0\s*$/, { timeout: 30000 });
  });
});
