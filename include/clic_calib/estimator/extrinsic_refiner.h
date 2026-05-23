#pragma once

#include <clic_calib/estimator/two_stage_types.h>
#include <clic_calib/sensor_data/apriltag_observation.h>
#include <clic_calib/sensor_data/lidar_target_observation.h>
#include <clic_calib/spline/trajectory.h>
#include <clic_calib/utils/camera_projection.h>
#include <clic_calib/utils/lever_arm.h>
#include <clic_calib/utils/noise_model.h>

#include <ceres/ceres.h>

#include <vector>

namespace clic_calib {

struct ExtrinsicRefinerConfig {
  double sphere_radius_m = 0.10;
  double t_d_max_abs_s = 0.1;
  double lidar_cauchy_scale = 1.0;
  double camera_huber_delta_px = 2.0;
  PinholeIntrinsics camera_K{600.0, 600.0, 320.0, 240.0};
  RadtanDistortion camera_dist;
  int marker_id = 0;
  int max_iterations = 500;
};

struct ExtrinsicRefinerResult {
  ExtrinsicOptimizeState lidar;
  ExtrinsicOptimizeState camera;
  ceres::Solver::Summary summary;
  bool converged = false;
};

/**
 * Stage-2 prior-free extrinsic refinement on a fixed trajectory.
 * Port of probe SolveStage2Extrinsics().
 */
class ExtrinsicRefiner {
 public:
  static ExtrinsicRefinerResult Refine(
      const BodyTrajectory& fixed_traj,
      const std::vector<LiDARTargetObservation>& lidar,
      const std::vector<AprilTagObservation>& tags,
      const LeverArmConfig& levers, const NoiseModel& noise,
      const CoarseExtrinsicInit& init, const ExtrinsicRefinerConfig& cfg);
};

}  // namespace clic_calib
