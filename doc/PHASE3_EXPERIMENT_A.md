# Phase 3 (A) — Dual LiDAR via Phase-1.5 reuse

**Status:** u_B aspect diagnosis (丁) + flight 戊 POI @ seed **13025**.

## Platform

- Stage-2 dual = two independent `CalibrateBodyGatedObservedMean` (Phase-1.5 path).
- Alignment check **PASS**: NearFieldHighAspect centroid 63.5 mm / iter-obs 18.5 mm.

## Gate (yaw_cv ≥ 0.65 + residual guard)

- **Aspect gate:** `yaw_cv ≥ 0.65` only (pitch-only no longer opens gate).
- **Guard:** monotonic Ceres `final_cost` per iter; accept obs-mean only if
  `|trans| ≤ centroid` and `final_cost ≤ centroid cost`.
- **乙:** yaw_cv≈0.32 → gate OFF → obs=centroid **164.98 mm** (no 1243 mm divergence).

## Four flights @ seed 13025

| flt | NE yaw_cv / pitch° | NE cent / obs | SW yaw_cv / pitch° | SW cent / obs | rel cent | rel obs |
|-----|-------------------:|--------------:|-------------------:|--------------:|---------:|--------:|
| 甲 | 0.00 / 0.0 | 31.5 / 31.5† | 0.00 / 0.0 | 19.4 / 19.4† | 73.8 | 73.8 |
| 乙 | 0.31 / 16.2 | 165.0 / 165.0† | 0.31 / 16.2 | 163.2 / 163.2† | 567.8 | 567.8 |
| 丙 | 0.00 / 14.2 | 76.7 / 76.7† | 0.00 / 14.2 | 49.4 / 49.4† | 133.4 | 133.4 |
| **丁** | **0.78 / 16.2** | 138.6 / **60.3** | **0.78 / 16.2** | 148.3 / **89.9** | 2366 | **310.9** |

† gate OFF or guard fallback → obs = centroid.

### 飞行丁 design

- **Translation:** dual-sector wide orbit (165° azimuth, 18–38 m).
- **Attitude:** Phase-1.5 `NearFieldHighAspect` law on sector-local time — **heading
  not locked to −p_WB** of the dual orbit (乙 locks heading to velocity).
- Per-sensor yaw_cv **0.783** matches Phase 1.5; observed-mean **improves** NE/SW
  vs centroid but **not** to ~20 mm / ~2–3 cm relative.

## u_B aspect + flight 戊

See `doc/PHASE3_ASPECT_BIAS.md`. Range hypotheses (a)(b)(c) excluded; true variable
is **u_B** (LiDAR look azimuth in body). Flight **戊** = per-sector POI lock toward
active LiDAR (DJI POI analogue).

## (B) entry

Criterion: **戊 @ 10 Hz** — each obs ≤ 35 mm, rel rot ≤ 0.1°. Center registration
reported separately. Run `test_phase3_dual_lidar_experiment_a` for live table.

## Deferred

- **(B)** Multi-LiDAR pipeline — hold until 戊 @ 10 Hz criterion met.
