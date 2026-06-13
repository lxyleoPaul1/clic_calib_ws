#!/usr/bin/env python3
"""Generate Fig. 2 (mechanism panel) via pgfplots + pdflatex."""

from __future__ import annotations

import csv
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parent
DATA = ROOT / "data"
OUT_TEX = ROOT / "fig_mechanism_standalone.tex"
OUT_PDF = ROOT / "fig_mechanism.pdf"


def read_yawcv() -> list[tuple[float, float, float]]:
    path = DATA / "yawcv_sweep.csv"
    by_az: dict[str, dict[str, list[float]]] = {}
    with path.open() as f:
        for r in csv.DictReader(f):
            if r["seed"] == "AGGREGATE":
                continue
            b = by_az.setdefault(r["azimuth_span_deg"], {"y": [], "c": [], "o": []})
            b["y"].append(float(r["yaw_cv_mean"]))
            b["c"].append(float(r["centroid_mm"]))
            b["o"].append(float(r["obs_mm"]))
    rows = []
    for az in sorted(by_az, key=float):
        b = by_az[az]
        n = len(b["y"])
        rows.append((
            sum(b["y"]) / n,
            sum(b["c"]) / n,
            sum(b["o"]) / n,
        ))
    rows.sort(key=lambda t: t[0])
    return rows


def read_ding_coords(step: int = 3) -> tuple[list[float], list[float], list[float], list[float]]:
    path = DATA / "ding_bias_scatter.csv"
    ne_u, ne_bx, sw_u, sw_bx = [], [], [], []
    with path.open() as f:
        rows = list(csv.DictReader(f))
    for i, r in enumerate(rows):
        if i % step != 0:
            continue
        if r["sensor"] == "lidar_NE":
            ne_u.append(float(r["u_B_azimuth_deg"]))
            ne_bx.append(float(r["bias_x_mm"]))
        elif r["sensor"] == "lidar_SW":
            sw_u.append(float(r["u_B_azimuth_deg"]))
            sw_bx.append(float(r["bias_x_mm"]))
    return ne_u, ne_bx, sw_u, sw_bx


def coords_table(xs: list[float], ys: list[float]) -> str:
    return "\n".join(f"      ({x:.2f},{y:.1f})" for x, y in zip(xs, ys))


def main() -> None:
    yaw = read_yawcv()
    ne_u, ne_bx, sw_u, sw_bx = read_ding_coords()
    cent_coords = "\n".join(f"      ({y:.4f},{m:.1f})" for y, m, _ in yaw)
    obs_coords = "\n".join(f"      ({y:.4f},{m:.1f})" for y, _, m in yaw)

    tex = rf"""
\documentclass[tikz,border=2pt]{{standalone}}
\usepackage{{pgfplots}}
\pgfplotsset{{compat=1.18}}
\begin{{document}}
\begin{{tikzpicture}}
  \begin{{groupplot}}[
    group style={{group size=3 by 1, horizontal sep=1.0cm}},
    width=0.34\linewidth, height=0.30\linewidth,
    grid=major,
    tick label style={{font=\scriptsize}},
    label style={{font=\scriptsize}},
    title style={{font=\scriptsize}}
  ]
  \nextgroupplot[
    title={{(a) Aspect--bias coupling}},
    xlabel={{$u_B$ az [deg]}}, ylabel={{bias$_x$ [mm]}}
  ]
    \addplot[only marks, mark size=0.8pt, blue, opacity=0.5]
      coordinates {{
{coords_table(ne_u, ne_bx)}
      }};
    \addplot[only marks, mark size=0.8pt, orange, opacity=0.5]
      coordinates {{
{coords_table(sw_u, sw_bx)}
      }};

  \nextgroupplot[
    title={{(b) Attitude averaging}},
    ylabel={{[mm]}}, symbolic x coords={{c,d}},
    xtick={{c,d}}, ymin=0, ymax=240,
    legend style={{font=\tiny, at={{(0.5,1.05)}}, anchor=south}}
  ]
    \addplot[ybar, fill=blue!55, bar width=12pt] coordinates {{(c,220) (d,51)}};
    \addplot[ybar, fill=red!55, bar width=12pt] coordinates {{(c,19.5) (d,19.5)}};
    \legend{{horiz., vert.}}

  \nextgroupplot[
    title={{(c) Identifiability transition}},
    xlabel={{yaw\_cv}}, ylabel={{trans [mm]}},
    ymin=0, ymax=480
  ]
    \addplot[blue, thick, mark=*] coordinates {{
{cent_coords}
    }};
    \addplot[red, thick, mark=square*] coordinates {{
{obs_coords}
    }};
    \draw[dashed, gray] (axis cs:0.65,0) -- (axis cs:0.65,480);
    \node[font=\tiny, anchor=south west] at (axis cs:0.67,420) {{0.65}};
  \end{{groupplot}}
\end{{tikzpicture}}
\end{{document}}
"""
    OUT_TEX.write_text(tex)
    subprocess.run(
        ["pdflatex", "-interaction=nonstopmode", OUT_TEX.name],
        cwd=ROOT,
        check=False,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    generated = ROOT / "fig_mechanism_standalone.pdf"
    if generated.exists():
        generated.replace(OUT_PDF)
    print(f"[fig2] wrote {OUT_PDF}")


if __name__ == "__main__":
    main()
