import { chromium } from "@playwright/test";
const b = await chromium.launch();
const out = "../docs/screenshots/ui";
for (const [w, h] of [[1366, 768], [1920, 1080]]) {
  const p = await b.newPage({ viewport: { width: w, height: h } });
  await p.goto("http://localhost:4173/?replay=circular_wait");
  await p.waitForSelector('[data-testid="start-card"]');
  await p.waitForTimeout(600);
  await p.screenshot({ path: `${out}_prestart_${w}.png` });
  await p.getByRole("button", { name: "Start simulation" }).click();
  await p.getByRole("button", { name: "4×" }).click();
  await p.waitForTimeout(3500);
  await p.getByTestId("replay-play").click();
  await p.screenshot({ path: `${out}_playing_${w}.png` });
  const s = p.getByLabel("Playback position"); await s.focus(); await s.press("End");
  await p.waitForTimeout(600);
  await p.screenshot({ path: `${out}_complete_${w}.png` });
  console.log(w, "scroll", await p.evaluate(() => [document.documentElement.scrollHeight - innerHeight, document.documentElement.scrollWidth - innerWidth]));
  await p.close();
}
await b.close();
