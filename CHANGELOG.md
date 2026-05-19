# CHANGELOG

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
