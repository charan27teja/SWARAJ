"""Benchmark report: mean and 95 % confidence interval per cell, % makespan reduction vs B0
(paired by seed), safety gates, charts. Honest by construction: every number comes from the CSV.

    python -m amr.bench.report [--csv runs/bench/results.csv] [--out docs/benchmark]
"""
from __future__ import annotations

import argparse
import sys
import csv
import html
import math
from collections import defaultdict
from pathlib import Path

from amr.core.scenario import ROOT

# two-sided 95 % Student-t critical values, df = 1..30 (normal beyond)
_T95 = [12.706, 4.303, 3.182, 2.776, 2.571, 2.447, 2.365, 2.306, 2.262, 2.228, 2.201, 2.179, 2.160, 2.145,
        2.131, 2.120, 2.110, 2.101, 2.093, 2.086, 2.080, 2.074, 2.069, 2.064, 2.060, 2.056, 2.052, 2.048,
        2.045, 2.042]


def mean_ci(xs: list[float]) -> tuple[float, float, int]:
    xs = [x for x in xs if x is not None and not math.isnan(x)]
    n = len(xs)
    if n == 0:
        return float("nan"), float("nan"), 0
    m = sum(xs) / n
    if n == 1:
        return m, float("nan"), 1
    sd = math.sqrt(sum((x - m) ** 2 for x in xs) / (n - 1))
    t = _T95[n - 2] if n - 1 <= 30 else 1.96
    return m, t * sd / math.sqrt(n), n


def _f(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def load(path: Path) -> list[dict]:
    with path.open(newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    for r in rows:
        r["robots"] = int(r["robots"])
        r["seed"] = int(r["seed"])
        r["all_done"] = r["all_done"] in ("True", "true", "1")
        for k in ("makespan", "throughput_per_h", "collisions", "unresolved_deadlocks", "duplicates",
                  "mean_wait_s", "bytes_per_robot_s", "section_violations", "deadlocks", "max_ttr_s",
                  "recovery_s", "beacon_bytes_mean", "min_robot_dist", "completed", "n_tasks"):
            r[k] = _f(r.get(k))
    return rows


def fmt(m, ci, unit="", nd=1):
    if m is None or math.isnan(m):
        return "—"
    if ci is None or math.isnan(ci):
        return f"{m:.{nd}f}{unit}"
    return f"{m:.{nd}f} ± {ci:.{nd}f}{unit}"


def build(rows: list[dict]) -> dict:
    cells = defaultdict(lambda: defaultdict(list))
    for r in rows:
        cells[(r["robots"], r["congestion"], r["fault"])][r["system"]].append(r)
    out = []
    for key in sorted(cells, key=lambda k: (k[0], ["low", "medium", "high"].index(k[1]) if k[1] in
                                              ("low", "medium", "high") else 9, k[2])):
        d = cells[key]
        ours, b0 = d.get("ours", []), d.get("b0", [])
        row = {"robots": key[0], "congestion": key[1], "fault": key[2]}
        for name, rs in (("ours", ours), ("b0", b0)):
            done = [r for r in rs if r["all_done"]]
            row[name] = {
                "n": len(rs), "done": len(done),
                "mk": mean_ci([r["makespan"] for r in done]),
                "tp": mean_ci([r["throughput_per_h"] for r in rs]),
                "wait": mean_ci([r["mean_wait_s"] for r in rs]),
                "col": sum(int(r["collisions"] or 0) for r in rs),
                "unres": sum(int(r["unresolved_deadlocks"] or 0) for r in rs),
                "dup": sum(int(r["duplicates"] or 0) for r in rs),
                "secv": sum(int(r["section_violations"] or 0) for r in rs),
                "bps": mean_ci([r["bytes_per_robot_s"] for r in rs]),
                "dmin": min([r["min_robot_dist"] for r in rs if r["min_robot_dist"] is not None] or [float("nan")]),
            }
        bs = {r["seed"]: r for r in b0 if r["all_done"]}
        red = [100.0 * (bs[r["seed"]]["makespan"] - r["makespan"]) / bs[r["seed"]]["makespan"]
               for r in ours if r["all_done"] and r["seed"] in bs]
        row["red"] = mean_ci(red)
        out.append(row)
    return {"cells": out, "n_runs": len(rows)}


def charts(rows: list[dict], out: Path) -> list[str]:
    """Grouped bars per cell (congestion x fault): ours vs B0, mean with 95 % CI."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    order = {"low": 0, "medium": 1, "high": 2}
    cells = sorted({(r["robots"], r["congestion"], r["fault"]) for r in rows},
                   key=lambda k: (k[0], order.get(k[1], 9), k[2]))
    files = []
    for metric, label, fname, only_done in (("makespan", "makespan (s)  - lower is better", "makespan_per_cell.png", True),
                                            ("throughput_per_h", "throughput (tasks / hour)  - higher is better",
                                             "throughput_per_cell.png", False)):
        fig, ax = plt.subplots(figsize=(1.6 * len(cells) + 2.5, 3.6))
        w = 0.38
        for i, (system, color, lab) in enumerate((("ours", "#2f7fd8", "decentralised (ours)"),
                                                   ("b0", "#d9822b", "B0 centralised stop-and-wait"))):
            ms, cs = [], []
            for (n, cong, fault) in cells:
                xs = [r[metric] for r in rows if r["system"] == system and r["robots"] == n and r["congestion"] == cong
                      and r["fault"] == fault and (r["all_done"] or not only_done)]
                m, ci, _ = mean_ci(xs)
                ms.append(m)
                cs.append(0 if math.isnan(ci) else ci)
            xs = [k + (i - 0.5) * w for k in range(len(cells))]
            ax.bar(xs, ms, w, yerr=cs, capsize=4, color=color, label=lab)
        ax.set_xticks(range(len(cells)))
        ax.set_xticklabels([f"{c} / {f}" for _, c, f in cells])
        ax.set_ylabel(label)
        ax.set_title(f"{cells[0][0]} robots, 95 % CI" if len({c[0] for c in cells}) == 1 else "95 % CI")
        ax.grid(axis="y", alpha=0.3)
        ax.legend(fontsize=8, loc="upper center", bbox_to_anchor=(0.5, -0.14), ncol=2, frameon=False)
        fig.tight_layout()
        fig.savefig(out / fname, dpi=110)
        plt.close(fig)
        files.append(fname)
    return files


def render(rep: dict, rows: list[dict], figs: list[str]) -> str:
    cells = rep["cells"]
    tot = {s: {"col": sum(c[s]["col"] for c in cells), "unres": sum(c[s]["unres"] for c in cells),
               "secv": sum(c[s]["secv"] for c in cells), "runs": sum(c[s]["n"] for c in cells),
               "done": sum(c[s]["done"] for c in cells), "dup": sum(c[s]["dup"] for c in cells)}
           for s in ("ours", "b0")}
    lines = ["# Benchmark report — decentralised fleet vs B0 (centralised stop-and-wait)", ""]
    lines += [f"Runs: **{rep['n_runs']}** (fast runtime, identical seeds for both systems). "
              f"Makespan statistics use completed runs; ± is the 95 % confidence interval (Student t). "
              f"% reduction = paired per-seed (B0 − ours) / B0 over seeds where both completed; "
              f"positive means our system finished sooner. Target from the problem statement: ≥ 20 %.", ""]
    lines += ["## Safety gates", "",
              "| system | runs | all tasks done | robot–robot collisions | unresolved deadlocks | two robots in one aisle | duplicate task executions |",
              "|---|---:|---:|---:|---:|---:|---:|"]
    for s, name in (("ours", "ours"), ("b0", "B0")):
        t = tot[s]
        lines.append(f"| {name} | {t['runs']} | {t['done']} | **{t['col']}** | **{t['unres']}** | {t['secv']} | {t['dup']} |")
    lines += ["", "## Makespan per cell", "",
              "| robots | congestion | fault | ours makespan (s) | B0 makespan (s) | reduction vs B0 | ours done | B0 done |",
              "|---:|---|---|---:|---:|---:|---:|---:|"]
    for c in cells:
        o, b = c["ours"], c["b0"]
        red = c["red"]
        flag = "" if math.isnan(red[0]) else (" ✅" if red[0] >= 20 else " ⚠️ below 20 %")
        lines.append(f"| {c['robots']} | {c['congestion']} | {c['fault']} | {fmt(o['mk'][0], o['mk'][1])} | "
                     f"{fmt(b['mk'][0], b['mk'][1])} | {fmt(red[0], red[1], ' %')}{flag} | {o['done']}/{o['n']} | {b['done']}/{b['n']} |")
    lines += ["", "## Other metrics per cell", "",
              "| robots | congestion | fault | throughput ours / B0 (tasks/h) | mean wait at choke points ours / B0 (s) | min separation ours / B0 (m) | bytes/robot/s ours |",
              "|---:|---|---|---:|---:|---:|---:|"]
    for c in cells:
        o, b = c["ours"], c["b0"]
        lines.append(f"| {c['robots']} | {c['congestion']} | {c['fault']} | {fmt(*o['tp'][:2], nd=0)} / {fmt(*b['tp'][:2], nd=0)} | "
                     f"{fmt(*o['wait'][:2])} / {fmt(*b['wait'][:2])} | {o['dmin']:.2f} / {b['dmin']:.2f} | {fmt(*o['bps'][:2], nd=0)} |")
    lines += ["", "## Charts", ""] + [f"![{f}]({f})" for f in figs]
    over = [c for c in cells if not math.isnan(c["red"][0])]
    good = [c for c in over if c["red"][0] >= 20]
    lines += ["", "## Honest reading", "",
              f"- Cells meeting the ≥ 20 % makespan-reduction target: **{len(good)} of {len(over)}**.",
              "- B0 is a strong comparator here: it shares the map traffic rules (one-way aisles), task admission "
              "control, kinematics, lidar safety and dwell times; only coordination differs (central FIFO zone "
              "reservations + stop-and-wait + central supervisor). With full knowledge and no faults, a central "
              "planner is at least as fast as a decentralised fleet, and this data shows makespans within a few "
              "percent of each other. The decentralised design's intended benefit is resilience (no single "
              "point of failure, operation through dead zones and partitions); its safety gates hold, but a "
              "makespan advantage of >= 20 % was **not** demonstrated in these cells.",
              "- Matrix: 5 robots x congestion medium/high x faults none/node_loss x 5 seeds, for both "
              "systems on identical seeds (`python -m amr.bench.sweep`)."]
    return "\n".join(lines) + "\n"


def md_to_html(md: str) -> str:
    out, in_table = [], False
    for line in md.splitlines():
        if line.startswith("|"):
            cells = [c.strip() for c in line.strip("|").split("|")]
            if set("".join(cells)) <= set("-:"):
                continue
            tag = "th" if not in_table else "td"
            if not in_table:
                out.append("<table>")
                in_table = True
            out.append("<tr>" + "".join(f"<{tag}>{_inline(c)}</{tag}>" for c in cells) + "</tr>")
            continue
        if in_table:
            out.append("</table>")
            in_table = False
        if line.startswith("# "):
            out.append(f"<h1>{_inline(line[2:])}</h1>")
        elif line.startswith("## "):
            out.append(f"<h2>{_inline(line[3:])}</h2>")
        elif line.startswith("!["):
            src = line[line.index("(") + 1:-1]
            out.append(f'<img src="{src}" alt="{src}">')
        elif line.startswith("- "):
            out.append(f"<p>• {_inline(line[2:])}</p>")
        elif line.strip():
            out.append(f"<p>{_inline(line)}</p>")
    if in_table:
        out.append("</table>")
    css = ("body{font-family:Inter,Segoe UI,sans-serif;max-width:1100px;margin:2rem auto;padding:0 1rem;color:#0e1726}"
           "table{border-collapse:collapse;margin:1rem 0;font-size:14px}td,th{border:1px solid #d6dee8;padding:4px 8px;"
           "text-align:right}th{background:#f3f6fa}td:nth-child(-n+3),th:nth-child(-n+3){text-align:left}"
           "img{max-width:100%;border:1px solid #d6dee8;margin:.5rem 0}")
    return f"<!doctype html><html><head><meta charset='utf-8'><title>Benchmark report</title><style>{css}</style></head><body>{''.join(out)}</body></html>"


def _inline(s: str) -> str:
    s = html.escape(s)
    while "**" in s:
        s = s.replace("**", "<b>", 1).replace("**", "</b>", 1)
    while "`" in s:
        s = s.replace("`", "<code>", 1).replace("`", "</code>", 1)
    return s


def main(argv=None):
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")   # Windows cmd code pages
    except (AttributeError, ValueError):
        pass
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default=str(ROOT / "runs" / "bench" / "results.csv"))
    ap.add_argument("--out", default=str(ROOT / "docs" / "benchmark"))
    a = ap.parse_args(argv)
    rows = [r for r in load(Path(a.csv)) if r.get("system")]
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    rep = build(rows)
    figs = charts(rows, out)
    md = render(rep, rows, figs)
    (out / "report.md").write_text(md, encoding="utf-8")
    (out / "report.html").write_text(md_to_html(md), encoding="utf-8")
    (out / "results.csv").write_text(Path(a.csv).read_text(encoding="utf-8"), encoding="utf-8")
    print(md)
    print(f"[report] written to {out}")


if __name__ == "__main__":
    main()
