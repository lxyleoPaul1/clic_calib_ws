/*
 * clic_calib — global batch calibration estimator (Phase 3).
 * Assembles §4.7 cost: RTK + LiDAR sphere + AprilTag + smoothness + extrinsic priors.
 */

#pragma once

#include <clic_calib/sensor_data/apriltag_observation.h>
#include <clic_calib/sensor_data/lidar_target_observation.h>
#include <clic_calib/sensor_data/rtk_measurement.h>
#include <clic_calib/spline/trajectory.h>
#include <clic_calib/utils/sophus_utils.hpp>

#include <clic_calib/estimator/observability_analyzer.h>

#include <ceres/ceres.h>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace clic_calib {

/**
 * @brief Owns trajectory T_WB(t), roadside extrinsics, time offsets, and Ceres problem.
 *
 * Parameter blocks share the same B-spline knot storage inside @p trajectory_.
 */
class CalibrationEstimator {
 public:
  /** @param config_path Directory containing lever_arms.yaml, sensor_rig.yaml, spline.yaml, target_geometry.yaml */
  explicit CalibrationEstimator(const std::string& config_path);
  ~CalibrationEstimator();

  void add_rtk_measurements(const std::vector<RTKMeasurement>& rtk);
  void add_lidar_target_observations(
      int sensor_id, const std::vector<LiDARTargetObservation>& obs);
  void add_apriltag_observations(int sensor_id,
                                 const std::vector<AprilTagObservation>& obs);

  void set_initial_extrinsic_T_LW(int sensor_id, const SE3d& T_LW);
  void set_initial_extrinsic_T_CW(int sensor_id, const SE3d& T_CW);

  /** @brief RTK + smoothness only; initializes spline knots before full solve. */
  void initialize_trajectory_from_rtk();

  ceres::Solver::Summary solve(int max_iters = 100);

  SE3d get_T_LW(int sensor_id) const;
  SE3d get_T_CW(int sensor_id) const;
  double get_t_d_lidar(int sensor_id) const;
  double get_t_d_camera(int sensor_id) const;
  std::shared_ptr<Trajectory> get_trajectory() const;

  /** @brief Assemble full Ceres problem without running the solver. */
  void build_problem_for_analysis();

  /** @brief Valid after @ref build_problem_for_analysis or @ref solve. */
  const ceres::Problem& problem() const;

  /** @brief Classify local parameter indices for §4.8 Schur complement. */
  AnalysisParameterLayout analysis_parameter_layout() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;

  friend class ObservabilityAnalyzer;
};

}  // namespace clic_calib
