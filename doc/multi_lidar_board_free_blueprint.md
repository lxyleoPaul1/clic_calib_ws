# Multi-LiDAR board-free blueprint

**Status:** Phase 3 (A) frozen @ `doc/board_free_results_frozen.md` — **Phase (B) ACTIVE.**

Simulation gate: `kSimMaxObsMm=42` + SW **known synthetic sector artifact, exempted**
(threshold not raised).

## Separability (Stage-2)

Dual LiDAR calibration under board-free Stage-2 is **two independent single-LiDAR
calibrations**:

1. Shared Stage-1 trajectory (RTK + attitude) from the full mission.
2. Per sensor: body-cluster observations in that sensor's FOV sector only.
3. Per sensor: run the **same** Phase-1.5 path (`FromGeometric` + tags,
   `RunBodyPath` / `RunBodyPathIterativeObservedMean`) — see
   `phase15_ablation_common.hpp` / `dual_lidar_scenario_common.hpp`.
4. Relative extrinsic `T_NE,SW` composed **post hoc** from per-sensor `T_LW` estimates.

Do **not** fork a separate dual-LiDAR init/refine (e.g. `FromBodyCentroidsOnly` without
tags). That path diverged from validated single-LiDAR results.

## Observed-mean `p_B` aspect gate

Under **concentrated aspect** (low yaw circular variance, low pitch std), a single
body-frame `p_B` is **not identifiable** from body-centroid factors alone. Applying
observed-mean `p_B` in that regime is **ill-conditioned** and can inject large errors
(e.g. SW `p_B` drift ~134 mm with only ~21 mm `b_const`).

**Policy:** gate observed-mean on **yaw circular variance** (data-calibrated):

| Condition | Action |
|-----------|--------|
| `yaw_cv ≥ 0.65` | try observed-mean `p_B` ×3 iter (monotonic Ceres cost) |
| `u_B` az std ∈ (0.1°, 5°] **and** `pitch_std ≥ 14°` | POI tidal-lock path (flight 戊) |
| otherwise | **centroid-only**; do not apply `p_B` |

**Residual guardrail** (mandatory after gate passes):

1. Each observed-mean iteration must **strictly lower** Stage-2 `final_cost`; else
   stop and keep previous step.
2. Accept observed-mean only if `|trans| ≤ centroid-only |trans|` **and**
   `final_cost ≤ centroid final_cost`.
3. Never output observed-mean worse than centroid-only (prevents e.g. 乙 → 1243 mm).

Implementation: `ShouldApplyObservedMeanPB` / `RunBodyPathIterativeObservedMean` /
`CalibrateBodyGatedObservedMean` in `test/experiments/phase15_ablation_common.hpp`.

Reference @ seed 13025:

- Phase 1.5 diverse: yaw_cv ≈ **0.78** → gate ON, obs ~18 mm ✓
- Dual 乙: yaw_cv ≈ **0.32** → gate OFF (pitch-only spread insufficient)
- Dual 甲: yaw_cv ≈ 0 → gate OFF

## Phase (B) entry — simulation vs field

**Entered @ 72050cc+** after flight 戊 mechanism replay @ seed 13025:

| Gate | Simulation (0.5 Hz 戊) | Field (real dual-Ruby) |
|------|------------------------|-------------------------|
| Per-sensor obs | max ≤ **~42 mm**, same order | each ≤ **35 mm** @ 10 Hz |
| rel rot | ≤ **0.15°** | ≤ **0.1°** @ 10 Hz |
| Stage-1 PW | 10 Hz RMSE **27 mm** (< 36 mm pre-PW) | — |
| SW asymmetry | **42 mm synthetic**; stop sim iteration | validate on hardware |

**Phase (B) work:** multi-LiDAR `CalibrationEstimator` + §7.2 relative extrinsic
covariance / RTK common-mode (shared trajectory perturbation MC).

Center registration reported separately — not gated on `rel |trans|`.

### Flight 戊 — serial POI (real mission)

Diagonal dual-LiDAR POI is **time-multiplexed** (one lock at a time):

- **t ∈ [0, 45) s:** POI lock toward **NE** post / LiDAR sector.
- **t ∈ [45, 90) s:** POI lock toward **SW** post / LiDAR sector.

Full RTK trajectory spans both segments; each sector is an independent POI orbit
around its roadside sensor.

Deprecated: flight 丁 + `rel |trans| ≤ 35 mm` (range/u_B decoupling root cause).
See `doc/PHASE3_ASPECT_BIAS.md`.

## Active in (B)

- `CalibrationEstimator` multi-sensor blocks, residual reweighting, weak-parameter passes.
