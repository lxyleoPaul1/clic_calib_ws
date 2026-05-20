#pragma once

#include <clic_calib/utils/sophus_utils.hpp>

#include <ceres/ceres.h>
#include <map>
#include <string>
#include <vector>

namespace clic_calib {

/** @brief Serialized offline calibration output for plot_residuals / reports. */
struct CalibrationResult {
  double final_cost = 0.0;
  ceres::Solver::Summary solver_summary;

  std::map<int, SE3d> T_LW;
  std::map<int, SE3d> T_CW;
  std::map<int, double> t_d_lidar;
  std::map<int, double> t_d_camera;

  std::vector<double> residual_values;
  double residual_rms = 0.0;
  double residual_max_abs = 0.0;
};

/** @brief Evaluate all Ceres residuals after @ref solve. */
void CollectProblemResiduals(ceres::Problem& problem,
                             CalibrationResult* result);

/** @brief Write @p result to JSON (manual, no external JSON library). */
bool WriteCalibrationJson(const CalibrationResult& result,
                          const std::string& path);

}  // namespace clic_calib
