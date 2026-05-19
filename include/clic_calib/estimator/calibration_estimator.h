/*
 * clic_calib — global batch estimator (Phase 0: interface only).
 * Refactored from clic::TrajectoryEstimator; SLAM-specific factors removed.
 */

#pragma once

#include <clic_calib/spline/trajectory.h>
#include <clic_calib/sensor_data/rtk_measurement.h>

#include <ceres/ceres.h>
#include <memory>
#include <string>
#include <vector>

namespace clic_calib {

struct CalibrationEstimatorOptions {
  double knot_interval_s = 0.05;
  int max_iterations = 50;
  bool verbose = false;
};

class CalibrationEstimator {
 public:
  explicit CalibrationEstimator(const CalibrationEstimatorOptions& options);

  void SetBodyTrajectory(BodyTrajectory::Ptr trajectory);
  void AddRtkMeasurements(const std::vector<RtkMeasurement>& measurements);

  ceres::Solver::Summary Solve();

 private:
  CalibrationEstimatorOptions options_;
  BodyTrajectory::Ptr trajectory_;
  std::unique_ptr<ceres::Problem> problem_;
};

}  // namespace clic_calib
