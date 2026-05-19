# legacy_clic

Quarantine for APRIL-ZJU/clic SLAM code removed from the `clic_calib` build.  
**Do not link** these sources in Phase 0. They remain for reference when implementing §4 factors.

| Directory | Original clic path | Purpose | Why removed |
|-----------|-------------------|---------|-------------|
| `imu_preintegration/` | `src/inertial/` | Static/dynamic IMU init, `ImuStateEstimator` | RTK provides absolute position; no preintegration |
| `lidar_odometry/` | `src/lidar_odometry/` | LOAM features, scan-to-map, keyframes | Calib uses point-to-sphere, not LOAM odometry |
| `visual_odometry/` | `src/visual_odometry/` | VINS KLT, camera models, VIO init | Calib uses AprilTag reprojection per marker |
| `loop_closure/` | `src/loop_closure/` | Loop detection, pose graph, ICP | Not used in offline extrinsic calibration |
| `marginalization/` | `estimator/factor/analytic_diff/marginalization_*` | Fixed-lag Schur marginalization | Batch calibration; no sliding-window marg |
| `factor_reference/` | `image_feature_factor.h`, `lidar_feature_factor.h` | Analytic factors for LICO | Refactor into `apriltag_reproj` / `sphere_implicit` |
| `app/odometry_node.cpp` | `src/app/odometry_node.cpp` | Main SLAM node | Replaced by `app/calibrate_offline.cpp` (Phase 1) |
| `launch/` | `launch/odometry.launch`, `clic.rviz` | SLAM launch + RViz | Not used in calibration pipeline |
| `msg/` | `msg/*.msg` | Custom ROS messages for SLAM | Phase 1 may add calib-specific msgs |
| `sensor_data/imu_data_reference.h` | `src/utils/parameter_struct.h` | `IMUData`, biases | Replaced by `rtk_measurement.h` |

Upstream full tree: `../clic-master/` (sibling directory in this workspace).

See also: `doc/MIGRATION.md` (§5), `RECON_REPORT.md`.
