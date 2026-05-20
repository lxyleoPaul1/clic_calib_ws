#!/usr/bin/env python3
"""Synthetic UAV trajectory + observations for end-to-end pipeline tests."""

from __future__ import annotations

import argparse
import csv
import math
import struct
import sys
from pathlib import Path

import numpy as np
import yaml


def load_noise_model(config_dir: Path) -> dict:
    path = config_dir / "noise_model.yaml"
    with path.open(encoding="utf-8") as f:
        node = yaml.safe_load(f)
    return {
        "rtk_sigma_h": float(node["rtk"]["sigma_horizontal_m"]),
        "rtk_sigma_v": float(node["rtk"]["sigma_vertical_m"]),
        "lidar_sigma_r": float(node["lidar"]["ranging_sigma_m"]),
        "camera_sigma_pix": float(node["camera"]["pixel_sigma"]),
    }

# Lever arms from config/lever_arms.yaml (FRD body frame).
L_B_TO_A = np.array([0.12, 0.05, -0.58])
L_B_TO_G = np.array([0.00, 0.00, -0.55])
L_G_TO_M0 = np.array([0.10, 0.00, 0.00])
R_BALL = 0.10
T_D_L = 0.030
T_D_C = -0.015

K = np.array([600.0, 600.0, 320.0, 240.0])  # fx, fy, cx, cy
TAG_CORNERS = np.array(
    [
        [-0.025, -0.025, 0.0],
        [0.025, -0.025, 0.0],
        [0.025, 0.025, 0.0],
        [-0.025, 0.025, 0.0],
    ]
)

LAT0 = 30.0
LON0 = 120.0
ALT0 = 50.0


def rot_z(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rot_y(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_x(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def se3(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def apply_se3(T: np.ndarray, p: np.ndarray) -> np.ndarray:
    return T[:3, :3] @ p + T[:3, 3]


def pose_wb(t: float, multilayer: bool, range_m: float) -> tuple[np.ndarray, np.ndarray]:
    """Body pose T_WB at time t (world = ENU, UAV at ~range_m east)."""
    s = t
    R = rot_z(0.05 * s)
    if multilayer:
        R = R @ rot_y(0.12 * math.sin(s))
        z = 10.0 + 4.0 * math.sin(s)
    else:
        z = 10.0
    p = np.array([range_m + 0.5 * s, 0.3 * math.sin(s), z])
    return R, p


def enu_to_lla(e: float, n: float, u: float) -> tuple[float, float, float]:
    dlat = n / 111320.0
    dlon = e / (111320.0 * math.cos(math.radians(LAT0)))
    return LAT0 + dlat, LON0 + dlon, ALT0 + u


def project_radtan(p_c: np.ndarray) -> np.ndarray:
    fx, fy, cx, cy = K
    if abs(p_c[2]) < 1e-6:
        return np.array([cx, cy])
    return np.array([fx * p_c[0] / p_c[2] + cx, fy * p_c[1] / p_c[2] + cy])


def write_clicob(
    path: Path,
    lidar_obs: list[tuple[float, int, list[np.ndarray]]],
    tag_obs: list[tuple[float, int, int, float, list[np.ndarray]]],
) -> None:
    """Write CLICOB01 binary archive."""
    with path.open("wb") as f:
        f.write(b"CLICOB01")
        f.write(struct.pack("<I", 1))  # version
        f.write(struct.pack("<I", len(lidar_obs)))
        f.write(struct.pack("<I", len(tag_obs)))

        for t_sensor, sensor_id, points in lidar_obs:
            f.write(struct.pack("<d", t_sensor))
            f.write(struct.pack("<i", sensor_id))
            f.write(struct.pack("<I", len(points)))
            for p in points:
                f.write(struct.pack("<ddd", float(p[0]), float(p[1]), float(p[2])))
            f.write(struct.pack("<I", 0))  # per_point_dt empty

        for t_sensor, sensor_id, tag_id, conf, corners in tag_obs:
            f.write(struct.pack("<d", t_sensor))
            f.write(struct.pack("<i", sensor_id))
            f.write(struct.pack("<i", tag_id))
            f.write(struct.pack("<d", conf))
            for c in corners:
                f.write(struct.pack("<dd", float(c[0]), float(c[1])))


def generate(
    output_dir: Path,
    multilayer: bool,
    range_m: float,
    include_camera: bool,
    noise: dict,
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)

    # Must match config/sensor_rig.yaml initial_T_LW (prior mean = GT for E2E).
    T_LW = se3(rot_y(-0.15), np.array([3.0, -1.0, 0.5]))
    T_CW = se3(rot_x(0.1), np.array([2.0, 1.5, 0.2]))

    rng = np.random.default_rng(123)
    rtk_sigmas = np.array([noise["rtk_sigma_h"], noise["rtk_sigma_h"], noise["rtk_sigma_v"]])
    rtk_rows: list[list] = []
    lidar_obs: list[tuple[float, int, list[np.ndarray]]] = []
    tag_obs: list[tuple[float, int, int, float, list[np.ndarray]]] = []

    for t in np.arange(0.2, 4.9, 0.1):
        R, p_wb = pose_wb(t, multilayer, range_m)
        p_a = p_wb + R @ L_B_TO_A
        noise_vec = rng.normal(0.0, rtk_sigmas)
        p_a_noisy = p_a + noise_vec
        lat, lon, alt = enu_to_lla(p_a_noisy[0], p_a_noisy[1], p_a_noisy[2])
        rtk_rows.append([f"{t:.3f}", f"{lat:.9f}", f"{lon:.9f}", f"{alt:.4f}",
                         f"{noise['rtk_sigma_h']:.4f}", f"{noise['rtk_sigma_h']:.4f}",
                         f"{noise['rtk_sigma_v']:.4f}", "FIXED"])

    for t in np.arange(0.5, 4.6, 0.4):
        R, p_wb = pose_wb(t, multilayer, range_m)
        p_g_w = p_wb + R @ L_B_TO_G
        p_g_l = apply_se3(T_LW, p_g_w)
        points = []
        for k in range(24):
            phi = 2.0 * math.pi * k / 24.0
            direction = np.array([math.cos(phi), math.sin(phi), 0.0])
            if not multilayer:
                direction[2] = 0.0
            direction = direction / np.linalg.norm(direction)
            points.append(p_g_l + R_BALL * direction)
        lidar_obs.append((t + T_D_L, 0, points))

    if include_camera:
        for t in np.arange(0.6, 4.5, 0.35):
            R, p_wb = pose_wb(t, multilayer, range_m)
            T_WB = se3(R, p_wb)
            corners_px = []
            for corner in TAG_CORNERS:
                l_corner = L_B_TO_G + L_G_TO_M0 + corner
                p_m_w = apply_se3(T_WB, l_corner)
                p_m_c = apply_se3(T_CW, p_m_w)
                corners_px.append(project_radtan(p_m_c))
            tag_obs.append((t + T_D_C, 0, 0, 1.0, corners_px))

    rtk_path = output_dir / "rtk.csv"
    with rtk_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(
            ["timestamp", "lat", "lon", "alt", "sigma_n", "sigma_e", "sigma_u", "fix_status"]
        )
        writer.writerows(rtk_rows)

    obs_path = output_dir / "obs.clicob"
    write_clicob(obs_path, lidar_obs, tag_obs)

    meta_path = output_dir / "synthetic_meta.txt"
    meta_path.write_text(
        f"multilayer={multilayer}\nrange_m={range_m}\ninclude_camera={include_camera}\n"
        f"noise_model={noise}\n",
        encoding="utf-8",
    )
    print(f"Wrote {rtk_path} ({len(rtk_rows)} RTK rows)")
    print(f"Wrote {obs_path} ({len(lidar_obs)} lidar scans, {len(tag_obs)} tags)")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir",
        type=Path,
        required=True,
        help="Directory for obs.clicob and rtk.csv",
    )
    parser.add_argument(
        "--range-m",
        type=float,
        default=200.0,
        help="Nominal horizontal standoff [m] (default 200)",
    )
    parser.add_argument(
        "--multilayer",
        action="store_true",
        default=True,
        help="Multi-layer vertical motion (default)",
    )
    parser.add_argument(
        "--coplanar",
        action="store_true",
        help="Coplanar flight (fixed altitude, horizontal-only LiDAR dirs)",
    )
    parser.add_argument(
        "--no-camera",
        action="store_true",
        help="Omit AprilTag observations",
    )
    args = parser.parse_args()

    script_dir = Path(__file__).resolve().parent
    config_dir = script_dir.parent / "config"
    noise = load_noise_model(config_dir)
    print(f"[simulate_uav_trajectory] noise model: {noise}")

    multilayer = not args.coplanar
    generate(
        args.output_dir,
        multilayer=multilayer,
        range_m=args.range_m,
        include_camera=not args.no_camera,
        noise=noise,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
