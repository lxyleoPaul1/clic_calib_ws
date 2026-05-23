#pragma once

#include <clic_calib/estimator/stage1_trajectory_fitter.h>

namespace clic_calib {

struct TrajectoryStageConfig {
  double knot_interval_s = 0.05;
  double alpha_p = 0.01;
  double alpha_R = 0.01;
  int attitude_stride = 25;
};

struct TrajectoryStageResult {
  std::shared_ptr<BodyTrajectory> trajectory;
  ceres::Solver::Summary summary;
};

/** Thin wrapper retained for CalibrationEstimator call sites. */
class TrajectoryStage {
 public:
  static TrajectoryStageResult Estimate(
      const std::vector<RTKMeasurement>& rtk,
      const std::vector<AttitudeObservation>& attitude,
      const LeverArmConfig& levers, const TrajectoryStageConfig& cfg,
      double t_obs_lo, double t_obs_hi);
};

}  // namespace clic_calib
