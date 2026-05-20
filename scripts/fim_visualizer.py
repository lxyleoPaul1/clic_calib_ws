#!/usr/bin/env python3
"""Visualize FIM observability report JSON from analyze_observability."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


def load_report(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def plot_eigenvalues(report: dict, ax) -> None:
    eig = np.asarray(report["eigenvalues"], dtype=float)
    labels = [f"λ{i + 1}" for i in range(len(eig))]
    ax.bar(labels, eig, color="steelblue")
    ax.set_yscale("log")
    ax.set_ylabel("Eigenvalue")
    ax.set_title("F_ext eigenvalues")
    ax.grid(True, axis="y", alpha=0.3)


def plot_translation_ellipsoid(report: dict, ax) -> None:
    """3D ellipsoid from LiDAR translation 3×3 covariance block."""
    info = np.asarray(report["information_matrix"], dtype=float)
    cov = np.linalg.pinv(info)
    # LiDAR translation local indices 3:6 in [L_q(3), L_t(3), C_q(3), C_t(3)] layout.
    cov_t = cov[3:6, 3:6]
    eigvals, eigvecs = np.linalg.eigh(cov_t)
    eigvals = np.clip(eigvals, 0.0, None)

    u = np.linspace(0.0, 2.0 * np.pi, 40)
    v = np.linspace(0.0, np.pi, 20)
    x = np.outer(np.cos(u), np.sin(v))
    y = np.outer(np.sin(u), np.sin(v))
    z = np.outer(np.ones_like(u), np.cos(v))
    sphere = np.stack([x, y, z], axis=-1)

    # One-sigma ellipsoid (χ² scale omitted for shape visualization).
    scale = np.sqrt(np.maximum(eigvals, 1e-12))
    transform = eigvecs @ np.diag(scale)
    ellipsoid = sphere @ transform.T

    ax.plot_surface(
        ellipsoid[:, :, 0],
        ellipsoid[:, :, 1],
        ellipsoid[:, :, 2],
        alpha=0.55,
        color="coral",
        linewidth=0,
    )
    ax.set_xlabel("t_x")
    ax.set_ylabel("t_y")
    ax.set_zlabel("t_z")
    ax.set_title("LiDAR translation uncertainty ellipsoid")
    lim = np.max(np.abs(ellipsoid)) * 1.1
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_zlim(-lim, lim)


def plot_correlation_heatmap(report: dict, ax) -> None:
    corr = np.asarray(report["correlation_matrix"], dtype=float)
    dim = corr.shape[0]
    labels = []
    for sensor, prefix in [("L", "LiDAR"), ("C", "Camera")]:
        for kind, name in [("r", "rot"), ("t", "trans")]:
            for i in range(3):
                labels.append(f"{prefix}_{name}{'xyz'[i]}")
        if len(labels) >= dim:
            break
    labels = labels[:dim]

    im = ax.imshow(corr, vmin=-1.0, vmax=1.0, cmap="RdBu_r", origin="lower")
    ax.set_xticks(range(dim))
    ax.set_yticks(range(dim))
    ax.set_xticklabels(labels, rotation=90, fontsize=7)
    ax.set_yticklabels(labels, fontsize=7)
    ax.set_title("Extrinsic parameter correlations")
    plt.colorbar(im, ax=ax, fraction=0.046, pad=0.04)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("report_json", type=Path, help="Observability report JSON")
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=None,
        help="Save figure to path instead of showing interactively",
    )
    args = parser.parse_args()

    if not args.report_json.is_file():
        print(f"File not found: {args.report_json}", file=sys.stderr)
        return 1

    report = load_report(args.report_json)

    fig = plt.figure(figsize=(14, 4.5))
    ax0 = fig.add_subplot(1, 3, 1)
    ax1 = fig.add_subplot(1, 3, 2, projection="3d")
    ax2 = fig.add_subplot(1, 3, 3)

    plot_eigenvalues(report, ax0)
    plot_translation_ellipsoid(report, ax1)
    plot_correlation_heatmap(report, ax2)
    fig.suptitle(
        f"λ_min={report['lambda_min']:.3e}  PDOP_ext={report['pdop_ext']:.3f}",
        fontsize=11,
    )
    fig.tight_layout()

    if args.output:
        fig.savefig(args.output, dpi=150, bbox_inches="tight")
        print(f"Wrote {args.output}")
    else:
        plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
