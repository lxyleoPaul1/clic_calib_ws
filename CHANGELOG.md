# CHANGELOG

## Production estimator refactor — two-stage architecture (`refactor/two-stage-estimator`, 2026-05-23)

### Summary

- **CalibrationEstimator** refactored to validated **two-stage + attitude + closed-form-init** architecture per `doc/architecture/two_stage_solver_blueprint.md`.
- Probe Config C numbers **reproduced in production modules** (Stage-1, Umeyama/IPPE init, 50/50 cm-level pipeline, UQ decoupling scalars).
- Five **non-negotiable fixes** preserved and verified (H2 transpose, Ceres ownership scope, tag-local IPPE, knot subset trim, prior-free Stage-2).
- Real-data **interface contract** only (`AttitudeReader`, `RealDataSession`, `--attitude` CLI) — no field rosbag processing in this PR.
- **Separate from PR #1** — no changes to PR #1 scope.

### Added (production)

- `Stage1TrajectoryFitter`, `ExtrinsicInitializer`, `ExtrinsicRefiner`, `TwoStagePipeline`, `stage2_extrinsic_fim`, `uq_decomposition`
- `trajectory_support`, `ceres_so3_scope`, `attitude_stream_config`, `real_data_session`, attitude factors + fixed-traj factors
- `AttitudeReader`, `AttitudeObservation`, `AttitudeStreamConfig`
- `doc/architecture/calibration_estimator_refactor.md` — port map, reproduction table, fix verification
- `scripts/run_production_regression.sh` — production parity regression gate

### Added (tests)

- `test_stage1_trajectory_fitter`, `test_extrinsic_initializer`, `test_two_stage_pipeline`
- `test_uq_decomposition`, `test_attitude_factor_jacobian`, `test_real_data_interface`

### Changed

- `CalibrationEstimator::RunTwoStageSolve()` when attitude stream present
- `ceres_local_param.h` — Lie analytic Jacobian **transpose** (H2 assembly fix)
- `calibrate_offline` — `--attitude`; `config/spline.yaml` attitude stride; `config/noise_model.yaml` Σ_att defaults
- `scripts/compile_local_tests.sh` — builds production regression binaries

### Validated @ N=50 (seeds 13000–13049) / UQ N=100

| Gate | Production result |
|------|-------------------|
| Stage-1 | pos **25.63 mm**, roll/pitch **~0.158°**, yaw **3.35°** |
| Closed-form T_LW | **0.0168° / 6.56 mm** |
| Closed-form T_CW | **0.0002° / 0.163 mm** |
| Config C | **50/50** cm-level, **0/50** wrong-basin |
| UQ MC | **9/14** ratio band (probe 11–12/14); LW_tz decouple **5.68×**; trans/rot **233×** |
| Stage-2 FIM | rank **11/14**, cond ~**10¹¹** @ rep seed |

---

## Synthetic phase frozen (`probe/uq-decomposition`, 2026-05-23)

### Status

- **Synthetic phase FINAL / FROZEN** — no further synthetic-precision tuning before ICRA submission.
- **§2 UQ validated** empirically via three-arm noise-source decomposition (MC N=100): **10/14** DoF zero-mean-qualified (**3/3 LiDAR–world translations** + 7 rotation/time DoF); decoupling cost quantified (e.g., LW_tz 99.6% from trajectory propagation).
- **Camera lateral translation bias disclosed** as limitation (CW_tx ≈ −2.90 ± 1.92 mm); not headline accuracy; root cause not isolated — out of scope for synthetic phase.
- Prior Schur-marginal FIM "UQ deferred" conclusion **withdrawn** (rank-deficient F_θθ).
- **Next:** `CalibrationEstimator` refactor per `doc/architecture/two_stage_solver_blueprint.md` (no PR #1 changes) and real-world experiments.

### Documentation (FINAL 2026-05-23)

- `doc/diagnostics/uq_decomposition.md`, `doc/results/synthetic_evaluation.md` (§8 limitations), `doc/results/section2_claim.md`, `doc/diagnostics/synthetic_phase_summary.md`, `doc/architecture/two_stage_solver_blueprint.md`, `doc/diagnostics/SYNTHETIC_PHASE_SEAL.md`

---

## Experiments — noise-regime evaluation (`experiments/noise-regime`, 2026-05-20)

### Added

- `config/noise_model.yaml` + `NoiseModel` C++ loader — single source for RTK / LiDAR / camera σ.
- `test/test_pipeline_noise_sweep.cpp` — N=20 full-modality noise characterization.
- `test/test_patent_z_accuracy.cpp` — paired multi-layer vs coplanar @ 200 m (seeds 2000–2019).
- `test/test_prior_ablation.cpp` + `set_extrinsic_prior_std()` / `set_extrinsic_prior_enabled()`.
- `test/experiments/noise_regime_common.hpp` — shared sweep scenario builders.
- `doc/results/synthetic_evaluation.md` — paper tables + honest claim routing.
- `scripts/run_regression_tests.sh` — smoke + noise sweeps + observability + diagnostics layout.

### Changed

- `test_full_pipeline_synthetic` demoted to noise-free smoke (`SmokeTest.*`).
- Simulator and estimator both read `noise_model.yaml`; duplicate noise keys removed from other yaml.
- `doc/diagnostics/time_offset_observability.md`, `doc/supplementary_section4.tex` updated.
- `README.md` — full project documentation (structure, config, test tiers, limitations).

### Findings (synthetic, N=20)

- Joint pipeline traj RMS **91.7 ± 16.3 mm** under realistic noise — not cm-accurate.
- 200 m Z-error: coplanar **9.996 mm** vs multi-layer **2.973 mm** (**3.36×**).
- Weak / no extrinsic prior → **~3.3×** larger \|T_LW\| error vs default prior.

## Debug session — time-offset observability (`debug/time-offset`, 2026-05-20)

### Root cause

- Python E2E recovered `t_d_lidar ≈ -1 s` because simulation GT `T_LW` disagreed with
  `sensor_rig.yaml` prior mean (~3 m translation gauge) and `t_d` bounds were ±1 s.
- Cross-modal noise mismatch (noisy RTK, exact LiDAR/camera) prevented honest strict
  local regression until RTK noise was removed from `test_full_pipeline_synthetic`.

### Fix

- Align sim GT with yaml; `t_d_max_abs_s: 0.1`; unified yaml prior + RTK warm-start in `solve()`.
- Analytic Jacobian corrections committed (STEP 2): `EvaluateRp`, AprilTag / RTK sign fixes,
  `ExtrinsicPriorFactor` quat+t parameterization.
- Strict tolerances reinstated and passing (local noise-free): rot 0.5°/0.3°, trans 5/3 cm,
  t_d ±2 ms, traj RMS <3 cm.

### Added

- `doc/diagnostics/time_offset_observability.md` — single-variable, gauge, and precision record.
- `test/diagnostic/test_td_single_variable.cpp`, `test/diagnostic/step3_gauge_analysis.cpp`.
- `scripts/compile_local_tests.sh` — g++-direct test build when catkin unavailable.

## Phase 3 — Batch calibration estimator (landed in `5e9ec8d`, 2026-05-20)

No dedicated branch; implementation shipped with Phase 5 commit. Documented here for traceability.

### Added

- `CalibrationEstimator` — §4.7 batch Ceres assembly: RTK + LiDAR sphere + AprilTag + smoothness + extrinsic priors.
- `BodyTrajectory` spline knots seeded from RTK; `t_d^L`, `t_d^C` released with spline-support bounds.
- `calibrate_offline` stub wired to estimator (full JSON I/O in Phase 6).
- GTest `test_full_pipeline_synthetic` — joint recovery of extrinsics, time offsets, trajectory.

### Libraries

- `clic_calib_estimator_lib` — estimator + observation/problem wiring.

## Phase 6 — End-to-end validation & documentation (`refactor/phase6-validation`, 2026-05-20)

### Added

- `calibrate_offline` — CLI batch calibration with JSON output (`calibration.json`).
- `CalibrationResult` I/O — residual collection + JSON writer for plotting.
- `scripts/run_full_experiment.sh` — synthetic or rosbag → calibrate → observability → PDF report.
- `scripts/simulate_uav_trajectory.py` — CLICOB01 + RTK CSV generator (200 m standoff, multi-layer / coplanar).
- `scripts/plot_residuals.py`, `scripts/generate_experiment_report.py` — residual plots + merged `experiment_report.pdf`.
- `scripts/run_regression_tests.sh` — all phase regression suites.
- `doc/supplementary_section4.tex` — paper-ready LaTeX for §4.1–§4.8.
- GTest `test_patent_z_accuracy` — 200 m range: multi-layer Z error < 0.1 m, coplanar > 1 m.

### Documentation

- `README.md` quickstart (build, full experiment, regression).
- `doc/DERIVATIONS.md` — LaTeX supplementary pointer + inline summary.

## Phase 4 — Target detection front-ends (`refactor/phase4-detection`, 2026-05-20)

### Added

- `SphereExtractor` — PCL pipeline: intensity filter → ROI crop → Euclidean cluster → RANSAC sphere; outputs raw inlier `LiDARTargetObservation` points (§4.3 implicit factor).
- `AprilTagDetectorWrapper` — libapriltag (tag36h11/25h9/16h5) + OpenCV; outputs `AprilTagObservation` per tag.
- `ObservationArchive` — binary cache format `CLICOB01` for preprocessed observations.
- `preprocess_rosbag` — offline bag → `.clicob` (LiDAR sphere + camera AprilTag).
- `config/target_detection.yaml` — detection thresholds (intensity, RANSAC, apriltag).
- GTest: `test_sphere_extractor`, `test_apriltag_wrapper`; `scripts/run_detection_tests.sh`.

### Dependencies

- PCL, OpenCV, libapriltag, rosbag, cv_bridge, pcl_conversions.

## Phase 2 — Analytic Ceres factors (`refactor/phase2-factors`, 2026-05-20)

### Implemented (§4.2–§4.6, header-only + GTest)

- `RTKPositionFactor` — whitened 3-D position residual, 4+4 spline knots
- `SphereImplicitFactor` — scalar point-to-sphere + `t_d^L` chain rule (sign fix on ∂/∂t_d)
- `AprilTagReprojFactor` — 2-D radtan reprojection per corner
- `TrajectorySmoothnessFactor` — accel + body-rate penalty at sample time
- `ExtrinsicPriorFactor` — `Log(T^{-1} T_prior)` on quaternion (4) + translation (3) blocks

### Fixes (this branch vs initial aa77022)

- RTK factor: Jacobian zero-loop out-of-bounds (segfault in GTest)
- `rd/so3_spline_view.h`: include `spline_segment.h` for standalone builds
- `apriltag_reproj_factor.h`: include `sphere_implicit_factor.h` for `sphere_dp_gw_dt`
- GTest: correct Ceres param layout (4 rot knots then 4 pos knots); SO(3) `LieLocalParameterization` numeric perturbation
- Residual geometry tests: sphere on-surface / 5 cm off-surface pass
- **Jacobian alignment (g++ GTest, 1e-5):** spline rotation uses `EvaluateRp` (legacy clic convention) with residual-specific sign; AprilTag extrinsic uses `R_CW * hat(p_M_W)`; prior uses decoupled log Jacobian with `abs_tol=0.05` in test

### Tests

- `test_rtk_factor_jacobian`, `test_sphere_factor_jacobian`,
  `test_apriltag_factor_jacobian`, `test_prior_smoothness_jacobian`
- `scripts/run_factor_tests.sh` (catkin devel or rosrun)
- All four factor Jacobian suites pass locally (g++ + Ceres + GTest)

## Phase 1 — Data layer & lever-arm infrastructure (`refactor/phase1-data-layer`, 2026-05-20)

### Added

- Sensor types: `RTKMeasurement`, `LiDARTargetObservation`, `AprilTagObservation`, `Frame` enum.
- `LeverArmConfig::from_yaml` with `L_B_to_A`, `L_G_to_M` map; inline `antenna_position_world`, `sphere_center_world`, `marker_position_world`.
- `RTKReader` hierarchy: `NMEAReader`, `CSVReader`, `DJIDatLogReader`.
- Library `clic_calib_data_lib` (lever arms + RTK I/O).
- GTests: `test_lever_arm`, `test_rtk_reader`; fixtures under `test/data/`.

### Unchanged

- No Ceres factors or optimization wired yet (Phase 2).

## Phase 0 — Setup and recon (`refactor/phase0-setup`, 2026-05-20)

**Acceptance:** `catkin_make --make-args clic_calib_spline_lib`; no `namespace clic` under `include/clic_calib/` or `src/clic_calib/`; SLAM code under `legacy_clic/` only.

### Build (this branch)

- Catkin target **`clic_calib_spline_lib`** only (spline + utils headers + Sophus vendored code).
- `clic_calib_core`, `calibrate_offline`, `analyze_observability`, and GTest **commented out** in `CMakeLists.txt`.
- No behavioral change to preserved spline math.

### Preserved (from APRIL-ZJU/clic)

- Basalt-derived split SE(3) B-spline: `include/clic_calib/spline/{se3,so3,rd,spline_common,spline_segment,ceres_spline_helper*}.h`
- New `BodyTrajectory` (`trajectory.h/.cpp`) for **T_WB(t)** only
- `include/clic_calib/utils/sophus_utils.hpp`, `eigen_utils.hpp`
- Analytic spline views: `so3_spline_view.h`, `rd_spline_view.h`, `split_spline_view.h`
- `ceres_local_param.h` (Sophus local parameterization)
- Vendored `src/sophus_lib/`

### Refactored

- Namespace `clic` → `clic_calib`
- Headers relocated to `include/clic_calib/` per §3 target layout
- `TrajectoryEstimator` → `CalibrationEstimator` (interface stub only)

### Stripped → `legacy_clic/`

- `imu_preintegration/` (was `src/inertial/` + visual `integration_base` context)
- `lidar_odometry/` (LOAM front-end)
- `visual_odometry/` (VINS KLT front-end)
- `loop_closure/`
- `marginalization/` (`marginalization_factor.*`)

See `legacy_clic/README.md` for rationale.

### Added (sources present; not built in Phase 0)

- Catkin package `clic_calib` (executables and core lib deferred to Phase 1)
- Config stubs: `lever_arms.yaml`, `sensor_rig.yaml`, `target_geometry.yaml`, `spline.yaml`
- `sensor_data/`, `factor/` (stubs), `io/` and `target/` (Phase 1)
- `test/test_spline_recovery.cpp` (GTest)
- `RECON_REPORT.md` (copied from reconnaissance phase)
- Placeholder scripts under `scripts/`

### Documentation (post–Phase 0)

- `doc/DERIVATIONS.md` — canonical §4.1–§4.8 mathematical specification for all factors
- `doc/MIGRATION.md` — §5 module-by-module lookup table with implementation status
- Factor header comments aligned to §4.2–§4.6; config keys for `sigma_r`, `sigma_pix`, `alpha_p`, `alpha_R`, priors
- `include/clic_calib/factor/trajectory_value_factor.h` preserved (reference; not in build)
- `legacy_clic/factor_reference/`, `legacy_clic/app/odometry_node.cpp`, `legacy_clic/sensor_data/imu_data_reference.h`

### Not in this phase

- `RtkPositionFactor` and other analytic factors (Phase 1; must match `DERIVATIONS.md` verbatim)
- apriltag, rosbag loaders, sphere RANSAC
- Online FIM / PDOP (`ObservabilityAnalyzer` stub only)
