#pragma once

#include <clic_calib/estimator/extrinsic_initializer.h>
#include <clic_calib/estimator/extrinsic_refiner.h>
#include <clic_calib/estimator/stage1_trajectory_fitter.h>
#include <clic_calib/estimator/two_stage_types.h>
#include <clic_calib/sensor_data/apriltag_observation.h>
#include <clic_calib/sensor_data/attitude_observation.h>
#include <clic_calib/sensor_data/lidar_target_observation.h>
#include <clic_calib/sensor_data/rtk_measurement.h>
#include <clic_calib/spline/trajectory.h>
#include <clic_calib/utils/lever_arm.h>
#include <clic_calib/utils/noise_model.h>

#include <ceres/ceres.h>
#include <memory>
#include <vector>

namespace clic_calib {

struct TwoStagePipelineConfig {
  Stage1TrajectoryConfig stage1;
  ExtrinsicInitializerConfig init;
  ExtrinsicRefinerConfig refine;
};

struct TwoStagePipelineResult {
  std::shared_ptr<BodyTrajectory> trajectory;
  GeometricInitReport geometric_init;
  ExtrinsicRefinerResult extrinsics;
  ceres::Solver::Summary stage1_summary;
  bool stage1_ok = false;
  bool stage2_ok = false;
};

/**
 * Production two-stage solve: Stage-1 trajectory → closed-form init → Stage-2 refine.
 * Observations use LiDARTargetObservation / AprilTagObservation schema (synthetic or real).
 */
class TwoStagePipeline {
 public:
  static TwoStagePipelineResult Run(
      const std::vector<RTKMeasurement>& rtk,
      const std::vector<AttitudeObservation>& attitude,
      const std::vector<LiDARTargetObservation>& lidar,
      const std::vector<AprilTagObservation>& tags,
      const LeverArmConfig& levers, const NoiseModel& noise,
      const TwoStagePipelineConfig& cfg);
};

}  // namespace clic_calib
