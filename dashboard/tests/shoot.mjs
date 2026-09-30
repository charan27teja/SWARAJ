// Screenshot helper for the UI review loop: node tests/shoot.mjs <outPrefix> [theme] [presenter]
import { chromium } from "@playwright/test";
const [, , prefix = "../docs/screenshots/dash", theme = "dark", presenter = ""] = process.argv;
const browser = await chromium.launch();
for (const [w, h] of [[1920, 1080], [1366, 768]]) {
  const page = await browser.newPage({ viewport: { width: w, height: h } });
  await page.addInitScript((t) => localStorage.setItem("amr-theme", t), theme);
  await page.goto("http://localhost:8080/");
  await page.waitForSelector('[data-testid="kpi-tasks"]', { timeout: 20000 });
  await page.waitForTimeout(2500);
  if (presenter) await page.keyboard.press("p");
  const file = `${prefix}_${w}x${h}${theme === "light" ? "_light" : ""}${presenter ? "_presenter" : ""}.png`;
  const scroll = await page.evaluate(() => [document.documentElement.scrollHeight, window.innerHeight, document.documentElement.scrollWidth, window.innerWidth]);
  await page.screenshot({ path: file });
  console.log(file, "scroll(h,vh,w,vw)=", scroll.join(","));
  await page.close();
}
await browser.close();
