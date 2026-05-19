# RECON_REPORT.md

Reconnaissance report for the APRIL-ZJU/clic fork before the `clic_calib` refactor.

Scope: `f:\PAPER\clicpro\clic-master`

Generated from codebase inspection only. No source files were modified during this reconnaissance pass.

## 0. Git / Branch Status

- `f:\PAPER\clicpro\clic-master` is not a Git repository: `git status` reports `fatal: not a git repository`.
- `f:\PAPER\clicpro` is also not a Git repository.
- Because no `.git` metadata exists in the inspected workspace, the requested branch `refactor/recon` could not be created and this report could not be checked in as a commit.
- Local commits beyond upstream HEAD cannot be determined without Git metadata.

## 1. Top-level Layout

Directory tree to depth 3 with inferred purpose:

```text
clic-master/
├── CMakeLists.txt
│   └── Catkin build definition for libraries, ROS messages, executable nodes, and tests.
├── package.xml
│   └── ROS package manifest for package `clic`.
├── README.md
│   └── Original project description, dependencies, install/run instructions, citation, and acknowledgements.
├── RECON_REPORT.md
│   └── This reconnaissance report.
├── config/
│   ├── ct_odometry_*.yaml
│   │   └── Dataset-level odometry configs selecting mode, bag path, spline interval, sensor YAMLs, weights, noise, and extrinsics.
│   ├── livox/
│   │   └── Livox dataset IMU/LiDAR configs.
│   ├── lvi/
│   │   └── LVI dataset camera/IMU/LiDAR configs and fisheye mask.
│   ├── ncd/
│   │   └── Newer College Dataset camera/IMU/LiDAR configs.
│   └── ntu/
│       └── NTU VIRAL camera/IMU/LiDAR configs.
├── launch/
│   ├── odometry.launch
│   │   └── Starts `odometry_node` with config path and RViz.
│   └── clic.rviz
│       └── RViz visualization layout.
├── msg/
│   ├── feature_cloud.msg
│   │   └── Custom feature cloud message for LiDAR feature streams.
│   ├── imu_array.msg
│   │   └── Custom array message for IMU batches.
│   └── pose_array.msg
│       └── Custom array message for pose batches.
└── src/
    ├── app/
    │   └── ROS/node entry points and small tools.
    ├── estimator/
    │   └── Optimization orchestration, trajectory manager, Ceres factors, marginalization, viewers, and message manager.
    ├── inertial/
    │   └── IMU static/dynamic initialization and state estimator.
    ├── lidar_odometry/
    │   └── LiDAR feature extraction, LOAM-style local map, feature association, and keyframes.
    ├── loop_closure/
    │   └── Loop detection, ICP-style scan matching support, pose graph, and ground-truth alignment helpers.
    ├── sophus_lib/
    │   └── Vendored Sophus Lie group implementation.
    ├── spline/
    │   └── Basalt-derived continuous-time SO(3), R^3, and SE(3) B-spline implementation.
    ├── utils/
    │   └── Shared math, YAML, point cloud, logging, parameter, timing, and Eigen helpers.
    └── visual_odometry/
        └── VINS-Mono-derived feature tracking, camera models, visual initialization, feature manager, and visual odometry.
```

No `include/` directory exists in the inspected tree. Public headers are currently under `src/`, and `CMakeLists.txt` exports `INCLUDE_DIRS src`.

## 2. B-spline Infrastructure

Files under `src/` touching B-spline evaluation, derivatives, Ceres helpers, or spline-backed trajectory state:

| Path | Exported classes / functions | Data types | Analytic Jacobians |
|---|---|---|---|
| `src/spline/spline_common.h` | blending matrix and spline basis helpers | `Eigen::Matrix`, scalar polynomial basis | Basis derivatives only; not a Ceres cost |
| `src/spline/spline_segment.h` | `SplineSegmentMeta`, `SplineMeta`, segment indexing helpers | integer nanosecond times, spline segment metadata | Metadata only |
| `src/spline/rd_spline.h` | `RdSpline<DIM,N,Scalar>`, `evaluate`, `velocity`, `acceleration`, knot access | `Eigen::Matrix<Scalar,DIM,1>` knots; `Eigen` Jacobian structs | Provides derivative/Jacobian structs for Euclidean spline value and derivatives |
| `src/spline/so3_spline.h` | `So3Spline<N,Scalar>`, `pose`, `velocityBody`, `accelerationBody`, knot access | `Sophus::SO3<Scalar>`, `Eigen::Vector3` tangent | Provides SO(3) Jacobian structs for cumulative spline evaluation and derivatives |
| `src/spline/se3_spline.h` | `Se3Spline<N,Scalar>` combining `So3Spline` + `RdSpline` | `Sophus::SE3`, `Sophus::SO3`, `Eigen::Vector3` | Provides pose/velocity/acceleration Jacobian structs through child splines |
| `src/spline/trajectory.h` | `SensorType`, `Trajectory` | `Se3Spline<SplineOrder,double>`, `SE3d`, `SO3d`, `Eigen::Vector3d`, `ExtrinsicParam` | Exposes spline-backed poses and velocities; no Ceres cost itself |
| `src/spline/trajectory.cpp` | `Trajectory::GetIMUState`, `GetSensorPose`, scan undistortion, TUM output | `IMUState`, `SE3d`, `PosCloud`, `Eigen::Vector3d` | Uses spline evaluation; no Ceres cost itself |
| `src/spline/ceres_spline_helper.h` | Ceres helper templates for spline interpolation | `Eigen` matrices, spline basis | Helper derivatives for Ceres-style code |
| `src/spline/ceres_spline_helper_jet.h` | Jet/autodiff-oriented spline helper | `ceres::Jet`, `Eigen` | Auto-diff helper, not acceptable for future large trajectory residuals |
| `src/spline/assert.h` | Basalt-style assertion macros | N/A | No |
| `src/estimator/factor/analytic_diff/so3_spline_view.h` | `So3SplineView`, `JacobianStruct`, `EvaluateRp`, `EvaluateRTp`, `VelocityBody`, `accelerationBody` | raw Ceres parameter blocks for `Sophus::SO3d` knots, `Eigen::Vector3d` | Yes, explicitly fills analytic SO(3) knot Jacobians |
| `src/estimator/factor/analytic_diff/rd_spline_view.h` | `RdSplineView`, `JacobianStruct`, `evaluate`, `velocity`, `acceleration` | raw Ceres parameter blocks for `Eigen::Vector3d` knots | Yes, explicit R^3 knot Jacobians |
| `src/estimator/factor/analytic_diff/split_spline_view.h` | `SplitSpineView`, `SplineIMUData`, combined IMU evaluation | SO(3) + R^3 spline views, gravity vector | Yes, combines analytic SO(3)/R^3 Jacobians |
| `src/estimator/factor/ceres_local_param.h` | `LieLocalParameterization`, `LieAnalyticLocalParameterization` | `Sophus` group types, Ceres local parameterization | Provides local parameterization Jacobian, right-trivialized `T * exp(delta)` |
| `src/estimator/trajectory_estimator.h/.cpp` | `TrajectoryEstimator`, `ResidualSummary`, factor addition methods, solve/marginalization helpers | `Trajectory::Ptr`, Ceres `Problem`, spline metadata, sensor factors | Owns and wires analytic factors; also contains some auto-diff additions |
| `src/estimator/trajectory_manager.h/.cpp` | `TrajectoryManager`, `TimeParam` | `Trajectory`, IMU buffers, LiDAR correspondences, visual features | Calls analytic factor construction and marginalization |
| `src/app/test_analytic_factor.cpp` | `FactorTest` and factor Jacobian checks | Ceres cost functions, spline test data | Unit-style analytic/numeric checks for existing factors |

Important preservation note for `clic_calib`: the reusable mathematical core is mostly in `src/spline/*`, `src/utils/sophus_utils.hpp`, and the analytic spline views under `src/estimator/factor/analytic_diff/*_spline_view.h`.

## 3. Sensor Factor Inventory

Primary Ceres factors live under `src/estimator/factor/analytic_diff` and `src/estimator/factor/auto_diff`. Additional Ceres factors exist in visual camera calibration code and loop closure.

### 3.1 Analytic trajectory / sensor factors

| Factor | Residual dim | Parameter blocks | Math description | Path |
|---|---:|---|---|---|
| `analytic_derivative::GravityFactor` | 3 | gravity `3` | Difference between optimized gravity and prior gravity, weighted by diagonal sqrt information | `src/estimator/factor/analytic_diff/trajectory_value_factor.h` |
| `analytic_derivative::BiasFactor` | 6 | gyro bias i `3`, gyro bias j `3`, accel bias i `3`, accel bias j `3` | Random-walk continuity residual for IMU biases | `src/estimator/factor/analytic_diff/trajectory_value_factor.h` |
| `analytic_derivative::IMUFactor` | 6 | SO(3) knots `4 x N`, R^3 knots `3 x N`, gyro bias `3`, accel bias `3`, gravity `3`, time offset `1` | Gyro and accelerometer residuals against spline angular velocity and body-frame acceleration | `src/estimator/factor/analytic_diff/trajectory_value_factor.h` |
| `analytic_derivative::IMUPoseFactor` | 6 | SO(3) knots `4 x N`, R^3 knots `3 x N`, time offset `1` | Absolute pose residual at one timestamp | `src/estimator/factor/analytic_diff/trajectory_value_factor.h` |
| `analytic_derivative::LocalVelocityFactor` | 3 | SO(3) knots `4 x N`, R^3 knots `3 x N`, time offset `1` | Local velocity transformed by spline orientation compared with spline world velocity | `src/estimator/factor/analytic_diff/trajectory_value_factor.h` |
| `analytic_derivative::Local6DoFVelocityFactor` | 6 | SO(3) knots `4 x N`, R^3 knots `3 x N`, time offset `1` | Intended 6-DoF local velocity factor; `Evaluate` currently returns `false` | `src/estimator/factor/analytic_diff/trajectory_value_factor.h` |
| `analytic_derivative::PreIntegrationFactor` | 15 | SO(3) knots `4 x M`, R^3 knots `3 x M`, gyro bias a/b `3+3`, accel bias a/b `3+3` | IMU preintegration residual between two times using spline pose/velocity | `src/estimator/factor/analytic_diff/trajectory_value_factor.h` |
| `analytic_derivative::RelativeOrientationFactor` | 3 | SO(3) knots `4 x M` | Relative rotation residual between two spline times | `src/estimator/factor/analytic_diff/trajectory_value_factor.h` |
| `analytic_derivative::LoamFeatureFactor` | 1 | SO(3) knots `4 x N`, R^3 knots `3 x N` | LOAM point-to-plane or point-to-line residual from LiDAR point to local map geometry | `src/estimator/factor/analytic_diff/lidar_feature_factor.h` |
| `analytic_derivative::LoamFeatureOptMapPoseFactor` | 1 | SO(3) knots `4 x N`, R^3 knots `3 x N`, map rotation `4`, map position `3` | LOAM residual while also optimizing map pose | `src/estimator/factor/analytic_diff/lidar_feature_factor.h` |
| `analytic_derivative::RalativeLoamFeatureFactor` | 1 | SO(3) knots `4 x M`, R^3 knots `3 x M` | Relative LOAM feature residual involving point time and map time | `src/estimator/factor/analytic_diff/lidar_feature_factor.h` |
| `analytic_derivative::ImageFeatureFactor` | 2 | SO(3) knots `4 x M`, R^3 knots `3 x M`, inverse depth `1`, time offset `1` | Two-frame inverse-depth reprojection residual with time-offset derivative | `src/estimator/factor/analytic_diff/image_feature_factor.h` |
| `analytic_derivative::Image3D2DFactor` | 2 | SO(3) knots `4 x N`, R^3 knots `3 x N`, world point `3`, time offset `1` | 3D point to camera reprojection residual with time-offset derivative | `src/estimator/factor/analytic_diff/image_feature_factor.h` |
| `analytic_derivative::ImageFeatureOnePoseFactor` | 2 | SO(3) knots `4 x N`, R^3 knots `3 x N`, inverse depth `1` | Reprojection residual with first pose held fixed and second pose from spline | `src/estimator/factor/analytic_diff/image_feature_factor.h` |
| `analytic_derivative::ImageDepthFactor` | 2 | inverse depth `1` | Reprojection residual across known relative camera transform, optimizing only inverse depth | `src/estimator/factor/analytic_diff/image_feature_factor.h` |
| `analytic_derivative::EpipolarFactor` | 1 | SO(3) knots `4 x N`, R^3 knots `3 x N` | Epipolar residual from two normalized bearing vectors and spline pose | `src/estimator/factor/analytic_diff/image_feature_factor.h` |
| `MarginalizationFactor` | dynamic | previous marginalization parameter blocks | Linearized prior from Schur complement / marginalization | `src/estimator/factor/analytic_diff/marginalization_factor.h/.cpp` |

### 3.2 Auto-diff factors

These are incompatible with the future rule that new trajectory residuals must be analytic, but they are present in the original code:

| Factor | Residual dim | Parameter blocks | Math description | Path |
|---|---:|---|---|---|
| `auto_diff::GyroFactor` | dynamic | SO(3) spline knots plus bias/time related blocks | Gyro residual via templated spline helper | `src/estimator/factor/auto_diff/trajectory_value_factor.h` |
| `auto_diff::GyroAcceWithConstantBiasFactor` | dynamic | spline knots, constant biases, gravity/time | IMU gyro/accel residual with constant biases | same |
| `auto_diff::IMUFactor` | dynamic | spline knots, biases, gravity/time | IMU residual | same |
| `auto_diff::GyroAcceBiasFactor` | dynamic | spline knots, biases | IMU residual with bias random walk | same |
| `auto_diff::IMUPoseFactor` | dynamic | spline knots, time offset | Pose residual | same |
| `auto_diff::IMUGlobalVelocityFactor` | dynamic | R^3 spline knots | Global velocity residual | same |
| `auto_diff::IMULocalVelocityFactor` | dynamic | SO(3)/R^3 spline knots | Local velocity residual | same |
| `auto_diff::VelocityConstraintFactor` | dynamic | spline knots | Velocity constraint | same |
| `auto_diff::LiDARPoseFactor` | dynamic | spline knots, LiDAR pose | LiDAR pose residual | same |
| `auto_diff::PreIntegrationFactor` | dynamic | spline knots, biases | IMU preintegration residual | same |
| `auto_diff::RelativeOrientationFactor` | dynamic | SO(3) spline knots | Relative orientation residual | same |
| `auto_diff::IMUDeltaOrientationFactor` | dynamic | SO(3) spline knots | IMU delta orientation residual | `src/estimator/factor/auto_diff/trajectory_relative_value_factor.h` |
| `auto_diff::PointFeatureFactor` | dynamic | pose/extrinsic blocks | Point feature residual | `src/estimator/factor/auto_diff/lidar_feature_factor.h` |
| `auto_diff::LoamFeatureOptMapPoseFactor` | dynamic | pose/map pose blocks | LOAM residual with map pose | same |
| `auto_diff::ImageFeatureFactor` | dynamic | camera pose / feature blocks | Image reprojection | `src/estimator/factor/auto_diff/image_feature_factor.h` |
| `auto_diff::Image3D2DFactor` | dynamic | pose + 3D point | 3D-2D reprojection | same |
| `auto_diff::ImageFeatureOnePoseFactor` | dynamic | pose + inverse depth | One-pose reprojection | same |
| `auto_diff::ImageDepthFactor` | dynamic | inverse depth | Depth-only reprojection | same |
| `auto_diff::EpipolarFactor` | dynamic | pose blocks | Epipolar residual | same |

### 3.3 Other Ceres functors outside `estimator/factor`

| Functor / factory | Residual dim | Purpose | Path |
|---|---:|---|---|
| `ReprojectionError3D` | 2 | Visual SFM reprojection for initialization; auto-diff | `src/visual_odometry/initial/initial_sfm.h/.cpp` |
| `CostFunctionFactory::{generateCostFunction...}` | 2 or 4 depending mono/stereo | Camera model reprojection errors for calibration/visual module; auto-diff | `src/visual_odometry/visual_feature/camera_models/CostFunctionFactory.h/.cc` |
| `PoseGraph3dErrorTerm` | 6 | Loop-closure pose graph edge; auto-diff | `src/loop_closure/pose_graph/pose_graph_3d_error_term.h` |

## 4. Trajectory Estimator Entry Point

Two classes jointly own the optimization pipeline:

- `TrajectoryEstimator` owns the Ceres `Problem`, factor insertion, local parameterization, marginalization assembly, and solving.
- `TrajectoryManager` owns the active `Trajectory`, IMU buffers, bias state, fixed-lag windows, and high-level update calls.

Quoted public interface from `src/estimator/trajectory_estimator.h`:

```cpp
class TrajectoryEstimator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<TrajectoryEstimator> Ptr;

  TrajectoryEstimator(Trajectory::Ptr trajectory,
                      TrajectoryEstimatorOptions& option,
                      std::string descri = "");

  ~TrajectoryEstimator();

  void SetKeyScanConstant(double max_time);
  bool MeasuredTimeToNs(const SensorType& sensor_type, const double& timestamp,
                        int64_t& time_ns) const;
  void SetFixedIndex(int idx);
  int GetFixedControlIndex() const;
  void SetTimeoffsetState();

  void AddStartTimePose(const PoseData& pose);
  void AddPoseMeasurementAnalytic(const PoseData& pose_data,
                                  const Eigen::Matrix<double, 6, 1>& info_vec);
  void AddIMUMeasurementAnalytic(const IMUData& imu_data, double* gyro_bias,
                                 double* accel_bias, double* gravity,
                                 const Eigen::Matrix<double, 6, 1>& info_vec,
                                 bool marg_this_factor = false);
  void AddBiasFactor(double* bias_gyr_i, double* bias_gyr_j, double* bias_acc_i,
                     double* bias_acc_j, double dt,
                     const Eigen::Matrix<double, 6, 1>& info_vec,
                     bool marg_this_factor = false, bool marg_all_bias = false);
  void AddGravityFactor(double* gravity, const Eigen::Vector3d& info_vec,
                        bool marg_this_factor = false);
  void AddPreIntegrationAnalytic(double ti, double tj,
                                 IntegrationBase* pre_integration,
                                 double* gyro_bias_i, double* gyro_bias_j,
                                 double* accel_bias_i, double* accel_bias_j,
                                 bool marg_this_factor = false);
  void AddRelativeRotationAnalytic(double ta, double tb, const SO3d& S_BtoA,
                                   const Eigen::Vector3d& info_vec);
  bool AddGlobalVelocityMeasurement(const double timestamp,
                                    const Eigen::Vector3d& global_v,
                                    double vel_weight);
  void AddLocalVelocityMeasurementAnalytic(const double timestamp,
                                           const Eigen::Vector3d& local_v,
                                           double weight);
  void AddLoamMeasurementAnalytic(const PointCorrespondence& pc,
                                  const SO3d& S_GtoM,
                                  const Eigen::Vector3d& p_GinM,
                                  const SO3d& S_LtoI,
                                  const Eigen::Vector3d& p_LinI, double weight,
                                  bool marg_this_factor = false);
  void AddImageFeatureAnalytic(const double ti, const Eigen::Vector3d& pi,
                               const double tj, const Eigen::Vector3d& pj,
                               double* inv_depth, bool fixed_depth = false,
                               bool marg_this_fearure = false);
  void AddImageDepthAnalytic(const Eigen::Vector3d& p_i,
                             const Eigen::Vector3d& p_j, const SO3d& S_CitoCj,
                             const Eigen::Vector3d& p_CiinCj,
                             double* inv_depth);
  void AddMarginalizationFactor(
      MarginalizationInfo::Ptr last_marginalization_info,
      std::vector<double*>& last_marginalization_parameter_blocks);

  void AddPoseMeasurementAutoDiff(const PoseData& pose_data, double pos_weight,
                                  double rot_weight);
  void Add6DofLocalVelocityAutoDiff(
      const double timestamp, const Eigen::Matrix<double, 6, 1>& local_gyro_vel,
      double gyro_weight, double velocity_weight);

  void AddCallback(const std::vector<std::string>& descriptions,
                   const std::vector<size_t>& block_size,
                   std::vector<double*>& param_block);

  ceres::Solver::Summary Solve(int max_iterations = 50, bool progress = false,
                               int num_threads = -1);

  void PrepareMarginalizationInfo(ResidualType r_type,
                                  ceres::CostFunction* cost_function,
                                  ceres::LossFunction* loss_function,
                                  std::vector<double*>& parameter_blocks,
                                  std::vector<int>& drop_set);
  void SaveMarginalizationInfo(MarginalizationInfo::Ptr& marg_info_out,
                               std::vector<double*>& marg_param_blocks_out);
  const ResidualSummary& GetResidualSummary() const;

  TrajectoryEstimatorOptions options;
};
```

Quoted public interface from `src/estimator/trajectory_manager.h`:

```cpp
class TrajectoryManager {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<TrajectoryManager> Ptr;

  TrajectoryManager(const YAML::Node& node, Trajectory::Ptr trajectory);
  void InitFactorInfo(const ExtrinsicParam& Ep_CtoI,
                      const ExtrinsicParam& Ep_LtoI,
                      const double image_feature_weight = 0);
  void SetTrajectory(Trajectory::Ptr trajectory);
  void SetSystemState(const SystemState& sys_state);
  void SetOriginalPose(Eigen::Quaterniond q,
                       Eigen::Vector3d p = Eigen::Vector3d::Zero());
  void AddIMUData(const IMUData& data);
  size_t GetIMUDataSize() const;
  void SetUpdatedLoop();
  void PropagateTrajectory(double scan_time_min, double scan_time_max);
  void UpdateLIOPrior(
      const Eigen::aligned_vector<PointCorrespondence>& point_corrs);
  void UpdateVisualOffsetPrior(const std::list<FeaturePerId>& features,
                               const std::vector<int>& good_feature_ids,
                               const double timestamps[]);
  void ClearLIOPrior();
  bool UpdateTrajectoryWithLoamFeature(
      const Eigen::aligned_vector<PointCorrespondence>& point_corrs,
      const std::list<FeaturePerId>& features = {},
      const std::vector<int>& good_feature_ids = {},
      const double timestamps[] = nullptr, const int iteration = 50);
  void UpdateLiDARAttribute(double scan_time_min, double scan_time_max);
  void Log(std::string descri) const;
  const ImuStateEstimator::Ptr GetIMUStateEstimator() const;
  void ExtendTrajectory(int64_t max_time_ns);
  IMUBias GetLatestBias() const;
  const VPointCloud& GetMargCtrlPoint() const;
  const VPointCloud& GetInitCtrlPoint() const;
  Eigen::Quaterniond GetGlobalFrame() const;
  const std::map<int, double>& GetFeatureInvDepths() const;
};
```

## 5. IMU Pipeline

Files involved in IMU initialization, state propagation, or preintegration and candidates for `legacy_clic/` unless reused only as reference:

- `src/inertial/inertial_initializer.h`
- `src/inertial/inertial_initializer.cpp`
- `src/inertial/imu_state_estimator.h`
- `src/inertial/imu_state_estimator.cpp`
- `src/visual_odometry/integration_base.h`
- `src/visual_odometry/vio_initial.h`
- `src/visual_odometry/vio_initial.cpp`
- `src/visual_odometry/initial/initial_alignment.h`
- `src/visual_odometry/initial/initial_aligment.cpp`
- `src/visual_odometry/initial/initial_sfm.h`
- `src/visual_odometry/initial/initial_sfm.cpp`
- `src/visual_odometry/initial/solve_5pts.h`
- `src/visual_odometry/initial/solve_5pts.cpp`
- `src/estimator/factor/analytic_diff/trajectory_value_factor.h` (`IMUFactor`, `IMUPoseFactor`, `PreIntegrationFactor`, bias/gravity factors)
- `src/estimator/factor/auto_diff/trajectory_value_factor.h`
- `src/estimator/factor/auto_diff/trajectory_relative_value_factor.h`
- `src/estimator/trajectory_manager.h`
- `src/estimator/trajectory_manager.cpp`
- `src/estimator/trajectory_estimator.h`
- `src/estimator/trajectory_estimator.cpp`
- `src/estimator/msg_manager.h`
- `src/estimator/msg_manager.cpp`
- `src/utils/parameter_struct.h` (`IMUData`, `IMUBias`, `IMUState`, `SystemState`)
- `msg/imu_array.msg`

Observation for `clic_calib`: the current IMU pipeline is deeply integrated into trajectory propagation and priors. It should be moved or isolated before RTK-based absolute-position trajectory initialization is introduced.

## 6. LiDAR Front-end

Files for LiDAR feature extraction, scan registration, surface/edge fitting, local map, and loop-related point cloud handling:

- `src/lidar_odometry/lidar_feature.h` — LiDAR feature containers and point correspondence structures.
- `src/lidar_odometry/lidar_odometry.h`
- `src/lidar_odometry/lidar_odometry.cpp` — LOAM-style local map, keyframes, downsampling, feature association, surrounding map extraction.
- `src/lidar_odometry/livox_feature_extraction.h`
- `src/lidar_odometry/livox_feature_extraction.cpp` — Livox feature extraction.
- `src/lidar_odometry/velodyne_feature_extraction.h`
- `src/lidar_odometry/velodyne_feature_extraction.cpp` — Velodyne scan organization and feature extraction.
- `src/estimator/msg_manager.h/.cpp` — Converts ROS LiDAR messages into raw/surface/corner clouds and invokes extractors.
- `src/utils/cloud_tool.h`
- `src/utils/mypcl_cloud_type.h`
- `src/estimator/factor/analytic_diff/lidar_feature_factor.h`
- `src/estimator/factor/auto_diff/lidar_feature_factor.h`
- `src/app/recovery_vlp16_timestamp.cpp` — utility for VLP-16 timestamp recovery.

No explicit NDT implementation was found. ICP-like alignment appears in loop closure / pose graph support and PCL usage rather than a standalone NDT/ICP module.

## 7. Visual Front-end

Files for KLT/image-feature handling, camera models, and visual-SLAM front-end:

- `src/visual_odometry/visual_feature/feature_tracker.h`
- `src/visual_odometry/visual_feature/feature_tracker.cpp` — VINS-style feature tracking; uses OpenCV feature detection and optical flow.
- `src/visual_odometry/visual_feature/feature_tracker_node.h`
- `src/visual_odometry/visual_feature/feature_tracker_node.cpp` — ROS wrapper for image callback and feature publication.
- `src/visual_odometry/visual_feature/parameters.h`
- `src/visual_odometry/visual_feature/parameters.cpp`
- `src/visual_odometry/visual_feature/camera_models/Camera.h/.cc`
- `src/visual_odometry/visual_feature/camera_models/CameraFactory.h/.cc`
- `src/visual_odometry/visual_feature/camera_models/CostFunctionFactory.h/.cc`
- `src/visual_odometry/visual_feature/camera_models/PinholeCamera.h/.cc`
- `src/visual_odometry/visual_feature/camera_models/CataCamera.h/.cc`
- `src/visual_odometry/visual_feature/camera_models/EquidistantCamera.h/.cc`
- `src/visual_odometry/visual_feature/camera_models/ScaramuzzaCamera.h/.cc`
- `src/visual_odometry/visual_feature/camera_models/gpl.h/.cc`
- `src/visual_odometry/visual_odometry.h`
- `src/visual_odometry/visual_odometry.cpp`
- `src/visual_odometry/feature_manager.h`
- `src/visual_odometry/feature_manager.cpp`
- `src/visual_odometry/visual_struct.h`
- `src/visual_odometry/parameters.h/.cpp`
- `src/visual_odometry/utility.h`
- `src/visual_odometry/initial/*`
- `src/estimator/factor/analytic_diff/image_feature_factor.h`
- `src/estimator/factor/auto_diff/image_feature_factor.h`

No ORB descriptor extraction or descriptor matching module was found. The front-end is KLT/VINS-style feature tracking, not descriptor-based SLAM.

## 8. Loop Closure / Map Management

Related files:

- `src/loop_closure/loop_closure.h`
- `src/loop_closure/loop_closure.cpp`
- `src/loop_closure/loop_closure_data.h`
- `src/loop_closure/pose_graph.h`
- `src/loop_closure/pose_graph.cpp`
- `src/loop_closure/pose_graph/types.h`
- `src/loop_closure/pose_graph/pose_graph_3d_error_term.h`
- `src/loop_closure/gt_loader.h`
- `src/lidar_odometry/lidar_odometry.h/.cpp` — keyframe cloud positions and map updates used by loop closure.
- `src/estimator/odometry_manager.cpp` — invokes `LoopClosureHandler`, clears prior on loop closure, updates map flags.

Observation for `clic_calib`: this is original SLAM loop-closure/map maintenance logic and is semantically unrelated to static roadside sensor calibration, except perhaps as a reference for pose graph / ICP coding patterns.

## 9. Marginalization

Implementation:

- `src/estimator/factor/analytic_diff/marginalization_factor.h`
  - `ResidualBlockInfo`
  - `ThreadsStruct`
  - `MarginalizationInfo`
  - `MarginalizationFactor`
- `src/estimator/factor/analytic_diff/marginalization_factor.cpp`
  - `MarginalizationInfo::addResidualBlockInfo`
  - `MarginalizationInfo::preMarginalize`
  - `MarginalizationInfo::marginalize`
  - `MarginalizationInfo::getParameterBlocks`
  - `MarginalizationFactor::Evaluate`

Callers:

- `src/estimator/trajectory_estimator.h/.cpp`
  - `TrajectoryEstimator::PrepareMarginalizationInfo`
  - `TrajectoryEstimator::SaveMarginalizationInfo`
  - `TrajectoryEstimator::AddMarginalizationFactor`
- `src/estimator/trajectory_manager.h/.cpp`
  - `TrajectoryManager::UpdateLIOPrior`
  - `TrajectoryManager::UpdateVisualOffsetPrior`
  - `TrajectoryManager::UpdateTrajectoryWithLoamFeature`
  - stores `lidar_marg_info`, `cam_marg_info`, parameter block vectors
- `src/estimator/odometry_manager.cpp`
  - calls `UpdateLIOPrior`, `UpdateVisualOffsetPrior`, and `ClearLIOPrior`

Observation for `clic_calib`: fixed-lag marginalization infrastructure may be reusable, but all residual selection and drop-set semantics are currently tied to LIO/LICO odometry windows.

## 10. Build System

Build files:

- `CMakeLists.txt`
- `package.xml`

Generated ROS messages:

- `feature_cloud.msg`
- `imu_array.msg`
- `pose_array.msg`

Libraries:

- `spline_lib`
  - Sources: `src/spline/trajectory.cpp`
  - Purpose: continuous-time trajectory implementation.
- `lidar_lib`
  - Sources: all `src/lidar_odometry/*.cpp`
  - Purpose: LiDAR feature extraction, local mapping, feature association.
- `feature_tracker_lib`
  - Sources: `src/visual_odometry/visual_feature/*.cpp` and camera model `.cc`.
  - Purpose: visual feature tracking and camera models.
- `visual_lib`
  - Sources: `src/visual_odometry/*.cpp`, `src/visual_odometry/initial/*.cpp`
  - Purpose: visual odometry, VIO initialization, visual feature management.

Executables:

- `feature_tracker`
  - Sources: visual feature tracker files.
  - Purpose: standalone visual feature tracker node.
- `odometry_node`
  - Sources: `src/app/odometry_node.cpp`, inertial, estimator, marginalization, utils parameter, loop closure.
  - Purpose: main CLIC odometry/SLAM node.
- `recovery_vlp16_timestamp`
  - Source: `src/app/recovery_vlp16_timestamp.cpp`.
  - Purpose: VLP-16 timestamp recovery utility.
- `test_analytic_factor`
  - Sources: `src/app/test_analytic_factor.cpp`, `src/utils/parameter_struct.cpp`, `src/visual_odometry/parameters.cpp`.
  - Purpose: executable test for analytic factors.

## 11. Configuration Inventory

YAML files under `config/`:

| Path | Inferred purpose |
|---|---|
| `config/ct_odometry_ntu.yaml` | Top-level NTU VIRAL odometry config; offline mode, bag path, spline params, LIO/LICO mode, IMU noise, sensor YAML paths, camera extrinsics. |
| `config/ct_odometry_lvi.yaml` | Top-level LVI dataset config; selects LVI camera/IMU/LiDAR YAMLs and odometry settings. |
| `config/ct_odometry_ncd.yaml` | Top-level Newer College Dataset config; selects NCD camera/IMU/LiDAR YAMLs and odometry settings. |
| `config/ct_odometry_livox.yaml` | Top-level Livox dataset config; selects Livox LiDAR/IMU YAMLs and odometry settings. |
| `config/ntu/cam_ntu.yaml` | NTU camera model, topic, image size, intrinsics/extrinsics/mask-like feature tracker settings. |
| `config/ntu/imu_ntu.yaml` | NTU IMU topic/noise/initialization configuration. |
| `config/ntu/lidar_ntu.yaml` | NTU LiDAR topic/type/extrinsics, feature extraction, local map, loop closure parameters. |
| `config/lvi/cam_lvi.yaml` | LVI camera model and tracking configuration, including fisheye mask reference. |
| `config/lvi/imu_lvi.yaml` | LVI IMU configuration. |
| `config/lvi/lidar_lvi.yaml` | LVI LiDAR configuration. |
| `config/ncd/cam_ncd.yaml` | NCD camera model and tracker configuration. |
| `config/ncd/imu_ncd.yaml` | NCD IMU configuration. |
| `config/ncd/lidar_ncd.yaml` | NCD LiDAR configuration. |
| `config/livox/imu_livox.yaml` | Livox dataset IMU configuration. |
| `config/livox/lidar_livox.yaml` | Livox LiDAR topic/type/extrinsics, feature extraction, local map, loop closure parameters. |

Non-YAML config asset:

- `config/lvi/fisheye_mask_720x540.jpg` — fisheye feature mask used by LVI camera config.

No `config/lever_arms.yaml` exists yet; it must be added for `clic_calib` according to the project brief.

## 12. External Dependencies

Dependencies found in `CMakeLists.txt` and package manifest:

| Dependency | Found in | Current role | Preserve for `clic_calib`? |
|---|---|---|---|
| `catkin` | `package.xml`, `CMakeLists.txt` | ROS build system | Yes; invariant requires catkin/ROS Noetic compatibility |
| `roscpp` | `CMakeLists.txt` | ROS nodes | Yes |
| `std_msgs` | both | custom message generation and ROS messages | Likely yes |
| `sensor_msgs` | `CMakeLists.txt` | IMU, image, point cloud subscriptions | Yes |
| `geometry_msgs` | `CMakeLists.txt` | poses/transforms/visualization | Likely yes |
| `nav_msgs` | `CMakeLists.txt` | odometry/path outputs | Possibly, depending on final ROS API |
| `visualization_msgs` | `CMakeLists.txt` | RViz markers | Useful for calibration visualization |
| `eigen_conversions` | `CMakeLists.txt` | Eigen/ROS conversions | Likely yes |
| `pcl_conversions` | `CMakeLists.txt` | ROS PointCloud2 to PCL conversion | Yes for LiDAR target extraction |
| `pcl_ros` | `CMakeLists.txt` | PCL ROS utilities | Yes for LiDAR target extraction/visualization |
| `cv_bridge` | both | ROS image to OpenCV | Yes for AprilTag/image processing |
| `roslib` | `CMakeLists.txt` | ROS package path helpers | Maybe; can preserve if config path lookup remains |
| `rosbag` | both | offline bag reading | Yes for experiments/reproducibility |
| `tf` | `CMakeLists.txt` | TF publication | Likely yes |
| `message_generation` | both | custom ROS messages | Maybe; depends whether custom messages are retained |
| `Eigen3` | `CMakeLists.txt` | all fixed-size math | Yes |
| `yaml-cpp` | `CMakeLists.txt` | YAML config loading | Yes |
| `Ceres` | `CMakeLists.txt` | global nonlinear optimization | Yes |
| `OpenCV` | `CMakeLists.txt` | image tracking/camera models | Yes, but original KLT SLAM front-end may be replaced by AprilTag detector |
| `OpenMP` | `CMakeLists.txt` | optional parallelization | Optional; preserve only if used in retained code |
| `PCL` | referenced through `${PCL_LIBRARIES}` without explicit `find_package(PCL REQUIRED)` | point cloud operations | Yes; build should be made explicit in refactor |
| `Boost` | referenced through `${Boost_LIBRARIES}` without explicit `find_package(Boost REQUIRED)` | filesystem and possible utility use | Likely yes; build should be made explicit if preserved |
| `Sophus` | vendored under `src/sophus_lib`; external `find_package(Sophus)` commented out | Lie groups and local parameterization | Yes, but use vendored or explicit package consistently |
| `livox_ros_driver` | included in `msg_manager.h`, dependency mentioned in README but not package manifest | Livox `CustomMsg` input | Probably legacy unless target roadside LiDAR requires Livox |
| `ov_core` | `package.xml` only | Declared dependency, not obviously referenced in CMake target sources | Likely not needed; flag for removal after verification |
| `apriltag` | not present | Required by `clic_calib` project brief but absent from current build | Must be added in future phase with justification |
| `GTest` | not present | Required by new factor-test standard but absent from current build | Must be added for `test/` in future phase |

## 13. Missing / Suspicious References

- No Git repository metadata exists at the inspected root or parent workspace, blocking branch/commit requirements.
- No `include/` directory exists, although future `clic_calib` standard requires headers under `include/clic_calib/`.
- No `config/lever_arms.yaml` exists.
- `CMakeLists.txt` links `${PCL_LIBRARIES}` and `${Boost_LIBRARIES}` but does not explicitly call `find_package(PCL REQUIRED)` or `find_package(Boost REQUIRED)`.
- `package.xml` declares `ov_core`, but the inspected CMake build does not list `ov_core` as a catkin component.
- `CMakeLists.txt` includes `livox_ros_driver/CustomMsg.h` usage through source headers but does not declare `livox_ros_driver` in `find_package(catkin REQUIRED COMPONENTS ...)` or `package.xml`.
- `Local6DoFVelocityFactor::Evaluate` in `src/estimator/factor/analytic_diff/trajectory_value_factor.h` returns `false`, so it is incomplete or intentionally disabled.
- File name `src/visual_odometry/initial/initial_aligment.cpp` appears misspelled (`aligment` vs `alignment`) but is referenced by glob and builds if present.

## 14. Refactor Implications for Phase 0

Recommended preservation / move decision from this reconnaissance:

- Preserve and rename/refit:
  - `src/spline/*`
  - `src/sophus_lib/*`
  - `src/utils/sophus_utils.hpp`
  - `src/estimator/factor/ceres_local_param.h`
  - `src/estimator/factor/analytic_diff/so3_spline_view.h`
  - `src/estimator/factor/analytic_diff/rd_spline_view.h`
  - `src/estimator/factor/analytic_diff/split_spline_view.h`
  - selected parts of `TrajectoryEstimator` after removing LIO/VIO-specific factors.
- Move to `legacy_clic/` first:
  - original LiDAR odometry / LOAM local map
  - original visual odometry / VINS KLT front-end
  - IMU initialization and preintegration pipeline
  - loop closure and pose graph
  - original `odometry_node` orchestration
- Add for `clic_calib`:
  - RTK absolute position factor at antenna lever arm
  - roadside camera AprilTag reprojection factor
  - roadside LiDAR point-to-sphere factor
  - sensor time-offset parameter blocks with analytic temporal derivatives
  - `config/lever_arms.yaml`
  - GTest-based analytic Jacobian tests
  - `CHANGELOG.md`

