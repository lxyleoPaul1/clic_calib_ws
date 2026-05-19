#include <clic_calib/estimator/calibration_estimator.h>

#include <iostream>

namespace clic_calib {

CalibrationEstimator::CalibrationEstimator(
    const CalibrationEstimatorOptions& options)
    : options_(options) {}

void CalibrationEstimator::SetBodyTrajectory(BodyTrajectory::Ptr trajectory) {
  trajectory_ = std::move(trajectory);
}

void CalibrationEstimator::AddRtkMeasurements(
    const std::vector<RTKMeasurement>& measurements) {
  (void)measurements;
  // Phase 1: wire RtkPositionFactor
}

ceres::Solver::Summary CalibrationEstimator::Solve() {
  ceres::Solver::Summary summary;
  if (!problem_) {
    std::cerr << "[CalibrationEstimator] no Ceres problem constructed.\n";
    return summary;
  }
  ceres::Solver::Options opts;
  opts.max_num_iterations = options_.max_iterations;
  opts.minimizer_progress_to_stdout = options_.verbose;
  ceres::Solve(opts, problem_.get(), &summary);
  if (!summary.IsSolutionUsable()) {
    std::cerr << "[CalibrationEstimator] failed:\n"
              << summary.FullReport() << std::endl;
  }
  return summary;
}

}  // namespace clic_calib
