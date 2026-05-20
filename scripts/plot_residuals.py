#!/usr/bin/env python3
"""Plot calibration residual histogram from calibrate_offline JSON."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def load_calibration(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("calibration_json", type=Path)
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=None,
        help="Save figure instead of interactive display",
    )
    args = parser.parse_args()

    if not args.calibration_json.is_file():
        print(f"File not found: {args.calibration_json}", file=sys.stderr)
        return 1

    cal = load_calibration(args.calibration_json)
    residuals = np.asarray(cal.get("residuals", {}).get("values", []), dtype=float)
    if residuals.size == 0:
        print("No residuals in calibration JSON", file=sys.stderr)
        return 1

    rms = cal["residuals"].get("rms", float(np.sqrt(np.mean(residuals**2))))
    max_abs = cal["residuals"].get("max_abs", float(np.max(np.abs(residuals))))
    final_cost = cal.get("final_cost", cal.get("solver", {}).get("final_cost", 0.0))

    fig, axes = plt.subplots(1, 2, figsize=(10, 4))

    axes[0].hist(residuals, bins=min(50, max(10, residuals.size // 5)), color="steelblue")
    axes[0].set_xlabel("Whitened residual")
    axes[0].set_ylabel("Count")
    axes[0].set_title("Residual histogram")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(np.arange(residuals.size), residuals, ".", markersize=2, alpha=0.6)
    axes[1].axhline(0.0, color="k", linewidth=0.5)
    axes[1].set_xlabel("Residual index")
    axes[1].set_ylabel("Value")
    axes[1].set_title("Per-residual scatter")
    axes[1].grid(True, alpha=0.3)

    fig.suptitle(
        f"final_cost={final_cost:.3f}  RMS={rms:.4f}  max|.|={max_abs:.4f}",
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
