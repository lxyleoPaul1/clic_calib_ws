# §5 — Module-by-Module Migration Plan

Lookup table from APRIL-ZJU/clic → `clic_calib`.  
**Status** as of Phase 0 + §4 documentation pass.

Legend: ✅ done · 🟡 partial / stub · ⬜ Phase 1+ · — not applicable in upstream tree

| Source module (clic) | Action | Destination | Status | Notes |
|----------------------|--------|-------------|--------|-------|
| `src/spline/trajectory.h` | PRESERVE | `include/clic_calib/spline/trajectory.h` | ✅ | `BodyTrajectory` = UAV anchor **T_WB(t)**; namespace `clic_calib::`. §4.1 helpers on class. |
| `src/spline/se3_spline.h` | PRESERVE | `include/clic_calib/spline/se3_spline.h` | ✅ | Unchanged logic; namespace updated. |
| `src/spline/ceres_spline_helper.h` | PRESERVE | `include/clic_calib/spline/ceres_spline_helper.h` | ✅ | Jet variant also copied (`ceres_spline_helper_jet.h`). |
| `src/spline/ceres_spline_helper_jet.h` | PRESERVE | `include/clic_calib/spline/ceres_spline_helper_jet.h` | ✅ | Test / numeric Jacobian only; not for production trajectory residuals. |
| `src/spline/{so3,rd,spline_common,spline_segment,assert}.h` | PRESERVE | `include/clic_calib/spline/` | ✅ | Required by `se3_spline`. |
| `src/utils/sophus_utils.hpp` | PRESERVE | `include/clic_calib/utils/sophus_utils.hpp` | ✅ | + `eigen_utils.hpp`. |
| `src/estimator/factor/analytic_diff/trajectory_value_factor.h` | PRESERVE | `include/clic_calib/factor/trajectory_value_factor.h` | 🟡 | Copied; **not in CMake** until `IntegrationBase` / IMU types removed. IMU factors unused per §5. |
| `src/estimator/factor/analytic_diff/{so3,rd,split}_spline_view.h` | PRESERVE | `include/clic_calib/factor/` | ✅ | Used by preserved factors + future §4 factors. |
| `src/estimator/factor/ceres_local_param.h` | PRESERVE | `include/clic_calib/factor/ceres_local_param.h` | ✅ | Right-trivialized Sophus local parametrization. |
| `src/estimator/factor/analytic_diff/image_feature_factor.h` | REFACTOR | `include/clic_calib/factor/apriltag_reproj_factor.h` | 🟡 | Reference copy: `legacy_clic/factor_reference/image_feature_factor.h`. Implement §4.4 in `apriltag_reproj_factor`. |
| `src/estimator/factor/analytic_diff/lidar_feature_factor.h` | REFACTOR | `include/clic_calib/factor/sphere_implicit_factor.h` | 🟡 | Reference: `legacy_clic/factor_reference/lidar_feature_factor.h`. Implement §4.3 in `sphere_implicit_factor`. |
| `src/estimator/factor/analytic_diff/trajectory_value_factor.h` (IMU blocks) | STRIP | `legacy_clic/imu_preintegration/` | ✅ | `IMUFactor`, `PreIntegrationFactor`, etc. not built in `clic_calib`. |
| `src/estimator/factor/analytic_diff/marginalization_factor.*` | STRIP | `legacy_clic/marginalization/` | ✅ | Batch offline calibration; no fixed-lag marg in v1. |
| `src/lidar_odometry/*` (§5: `odometry/lidar_handler`) | STRIP | `legacy_clic/lidar_odometry/` | ✅ | No scan-to-map LOAM in calib. Upstream has no `include/odometry/`. |
| `src/visual_odometry/*` | STRIP | `legacy_clic/visual_odometry/` | ✅ | No KLT / VIO. |
| `src/inertial/*` (§5: `inertial_initializer`) | STRIP | `legacy_clic/imu_preintegration/` | ✅ | RTK initializes trajectory. |
| `src/estimator/trajectory_manager.h` | REFACTOR | `include/clic_calib/estimator/calibration_estimator.h` | 🟡 | **Phase 3** — wire §4.7 cost; currently stub. |
| `src/estimator/trajectory_estimator.h` | REFACTOR | (absorbed into `calibration_estimator`) | 🟡 | Phase 3. |
| `src/utils/parameter_struct.h` (`IMUData`, …) | STRIP (reference) | `legacy_clic/sensor_data/imu_data_reference.h` | ✅ | Replaced by `rtk_measurement.h`. |
| — (no `lidar_data.h` in clic) | REFACTOR | `sensor_data/lidar_target_observation.h` | 🟡 | Raw points + timestamp; no LOAM feature clouds. |
| — (visual feature msgs) | REFACTOR | `sensor_data/apriltag_observation.h` | 🟡 | AprilTag corners per §4.4. |
| `src/estimator/`, `src/lidar_odometry/`, `src/loop_closure/` | STRIP | `legacy_clic/*` | ✅ | Full trees under `legacy_clic/`. |
| `src/scan_registration/` | STRIP | `legacy_clic/` | — | **Not present** in upstream clic repo. |
| `src/app/odometry_node.cpp` | STRIP | `legacy_clic/app/odometry_node.cpp` | ✅ | Replaced by `app/calibrate_offline.cpp`. |
| `config/*.yaml` | REFACTOR | `config/{lever_arms,sensor_rig,target_geometry,spline}.yaml` | 🟡 | New schema; dataset-specific `ct_odometry_*.yaml` not ported. |
| `CMakeLists.txt` | REFACTOR | `clic_calib/CMakeLists.txt` | 🟡 | `clic_calib_spline`, `clic_calib_core`; no odometry targets. Phase 1 adds factor sources. |
| `package.xml` | REFACTOR | `clic_calib/package.xml` | ✅ | Package name `clic_calib`. |

---

## New modules (no clic source)

| Module | Destination | Status |
|--------|-------------|--------|
| RTK residual §4.2 | `factor/rtk_position_factor.h` | 🟡 stub |
| Sphere residual §4.3 | `factor/sphere_implicit_factor.h` | 🟡 stub |
| AprilTag §4.4 | `factor/apriltag_reproj_factor.h` | 🟡 stub |
| Smoothness §4.5 | `factor/trajectory_smoothness_factor.h` | 🟡 stub |
| Extrinsic prior §4.6 | `factor/prior_factor.h` | 🟡 stub |
| FIM / PDOP §4.8 | `estimator/observability_analyzer.h` | 🟡 stub |
| Sphere RANSAC | `target/sphere_extractor.h` | 🟡 stub |
| AprilTag detect | `target/apriltag_detector_wrapper.h` | 🟡 stub |
| RTK / bag I/O | `io/rtk_reader.h`, `io/rosbag_loader.h` | 🟡 stub |

---

## Phase roadmap (cross-reference)

| Phase | Focus |
|-------|--------|
| **0** | Skeleton, spline preserve, `legacy_clic/`, configs, `DERIVATIONS.md` §4 |
| **1** | Implement §4.2–§4.4 factors + GTest Jacobians; config schema finalize |
| **2** | `trajectory_smoothness_factor`, `prior_factor`, rosbag + target front-ends |
| **3** | `calibration_estimator` full §4.7 assembly; `calibrate_offline` pipeline |
| **4** | `observability_analyzer` §4.8; flight recommendation |

---

## Build targets (current)

| Target | Type | Migration note |
|--------|------|----------------|
| `clic_calib_spline` | library | PRESERVE spline + `BodyTrajectory` |
| `clic_calib_core` | library | estimator + lever_arm |
| `calibrate_offline` | executable | replaces `odometry_node` |
| `analyze_observability` | executable | new |
| `test_spline_recovery` | gtest | Phase 0 |
| `odometry_node`, `feature_tracker`, … | — | **removed** (in `legacy_clic` only) |
