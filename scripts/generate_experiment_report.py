#!/usr/bin/env python3
"""Merge calibration + observability artifacts into experiment_report.pdf."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backends.backend_pdf import PdfPages


def load_json(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def add_summary_page(pdf: PdfPages, out_dir: Path) -> None:
    cal_path = out_dir / "calibration.json"
    obs_path = out_dir / "observability.json"
    meta_path = out_dir / "synthetic_meta.txt"

    lines = ["clic_calib — End-to-End Experiment Report", ""]
    if meta_path.is_file():
        lines.append("Synthetic data:")
        lines.extend(meta_path.read_text(encoding="utf-8").strip().splitlines())
        lines.append("")

    if cal_path.is_file():
        cal = load_json(cal_path)
        lines.append(f"Final cost: {cal.get('final_cost', 'n/a')}")
        solver = cal.get("solver", {})
        lines.append(f"Solver iterations: {solver.get('iterations', 'n/a')}")
        lines.append(f"Termination: {solver.get('termination', 'n/a')}")
        lidar = cal.get("extrinsics", {}).get("lidar", {}).get("0", {})
        if "T_LW" in lidar:
            t = lidar["T_LW"]
            lines.append(
                f"T_LW.t = [{t.get('tx', 0):.4f}, {t.get('ty', 0):.4f}, {t.get('tz', 0):.4f}] m"
            )
        lines.append(f"t_d_lidar = {lidar.get('t_d_lidar_s', 'n/a')} s")
        res = cal.get("residuals", {})
        lines.append(f"Residual RMS: {res.get('rms', 'n/a')}")
        lines.append("")

    if obs_path.is_file():
        obs = load_json(obs_path)
        lines.append(f"λ_min(F_ext): {obs.get('lambda_min', 'n/a')}")
        lines.append(f"PDOP_ext: {obs.get('pdop_ext', 'n/a')}")
        lines.append(f"Condition number: {obs.get('condition_number', 'n/a')}")
        wd = obs.get("worst_direction", [])
        if wd:
            lines.append(f"Worst direction: [{wd[0]:.3f}, {wd[1]:.3f}, {wd[2]:.3f}]")

    fig, ax = plt.subplots(figsize=(8.5, 11))
    ax.axis("off")
    ax.text(
        0.05,
        0.95,
        "\n".join(lines),
        va="top",
        ha="left",
        fontsize=11,
        family="monospace",
        transform=ax.transAxes,
    )
    pdf.savefig(fig, bbox_inches="tight")
    plt.close(fig)


def add_image_page(pdf: PdfPages, image_path: Path, title: str) -> None:
    if not image_path.is_file():
        return
    img = plt.imread(image_path)
    fig, ax = plt.subplots(figsize=(11, 8.5))
    ax.imshow(img)
    ax.axis("off")
    ax.set_title(title)
    pdf.savefig(fig, bbox_inches="tight")
    plt.close(fig)


def add_fim_inline(pdf: PdfPages, obs_path: Path) -> None:
    if not obs_path.is_file():
        return
    report = load_json(obs_path)
    fig = plt.figure(figsize=(14, 4.5))
    ax0 = fig.add_subplot(1, 3, 1)
    eig = np.asarray(report["eigenvalues"], dtype=float)
    ax0.bar([f"λ{i+1}" for i in range(len(eig))], eig, color="steelblue")
    ax0.set_yscale("log")
    ax0.set_title("F_ext eigenvalues")
    ax0.grid(True, axis="y", alpha=0.3)

    ax2 = fig.add_subplot(1, 3, 3)
    corr = np.asarray(report["correlation_matrix"], dtype=float)
    ax2.imshow(corr, vmin=-1.0, vmax=1.0, cmap="RdBu_r", origin="lower")
    ax2.set_title("Extrinsic correlations")

    fig.suptitle(
        f"λ_min={report['lambda_min']:.3e}  PDOP_ext={report['pdop_ext']:.3f}",
        fontsize=11,
    )
    fig.tight_layout()
    pdf.savefig(fig, bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "experiment_dir",
        type=Path,
        help="Directory containing calibration.json, observability.json, plots",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=None,
        help="Output PDF path (default: <experiment_dir>/experiment_report.pdf)",
    )
    args = parser.parse_args()

    if not args.experiment_dir.is_dir():
        print(f"Not a directory: {args.experiment_dir}", file=sys.stderr)
        return 1

    out_pdf = args.output or (args.experiment_dir / "experiment_report.pdf")
    obs_path = args.experiment_dir / "observability.json"

    with PdfPages(out_pdf) as pdf:
        add_summary_page(pdf, args.experiment_dir)
        add_image_page(pdf, args.experiment_dir / "residuals.png", "Calibration residuals")
        add_fim_inline(pdf, obs_path)
        add_image_page(pdf, args.experiment_dir / "fim.png", "FIM visualization")

    print(f"Wrote {out_pdf}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
