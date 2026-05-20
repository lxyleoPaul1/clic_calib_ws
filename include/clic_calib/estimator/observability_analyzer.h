/*
 * clic_calib — §4.8 Observability monitor (canonical).
 *
 * F_ext = F_{ext,ext} - F_{ext,θ_rest} * F_{θ_rest,θ_rest}^{-1} * F_{θ_rest,ext}
 * Report λ_min, condition_number, PDOP_ext.
 *
 * See doc/DERIVATIONS.md §4.8.
 */

#pragma once

#include <Eigen/Core>
#include <string>
#include <vector>

namespace clic_calib {

class CalibrationEstimator;

/** @brief Local-parameter index sets for Schur complement on extrinsics. */
struct AnalysisParameterLayout {
  int num_local_parameters = 0;
  /** Local indices for T_LW / T_CW (q+t blocks, 6 DoF per sensor). */
  std::vector<int> extrinsic_local_indices;
  /** Trajectory knots + time offsets t_d. */
  std::vector<int> rest_local_indices;
  /** Local indices for first LiDAR sensor rotation / translation blocks. */
  std::vector<int> lidar_rot_local_indices;
  std::vector<int> lidar_trans_local_indices;
  /** Indices into the 12×12 F_ext / worst_eigenvector for LiDAR blocks. */
  std::vector<int> lidar_rot_fext_indices;
  std::vector<int> lidar_trans_fext_indices;
};

struct ObservabilityReport {
  /** Marginal 12×12 information matrix F_ext (1 LiDAR + 1 camera). */
  Eigen::MatrixXd information_matrix;
  Eigen::VectorXd eigenvalues;
  double lambda_min = 0.0;
  double lambda_max = 0.0;
  double condition_number = 0.0;
  double pdop_ext = 0.0;
  /** Unit eigenvector of Cov_ext = F_ext^{-1} for smallest λ (largest uncertainty). */
  Eigen::VectorXd worst_eigenvector;
  /** Translation part (LiDAR sensor) of @p worst_eigenvector for planning hooks. */
  Eigen::Vector3d worst_direction = Eigen::Vector3d::Zero();
  /** Normalized pairwise correlation from Cov_ext. */
  Eigen::MatrixXd correlation_matrix;
};

class ObservabilityAnalyzer {
 public:
  /** @brief Build F from Ceres Jacobian, Schur-complement trajectory/time offsets. */
  ObservabilityReport analyze(const CalibrationEstimator& estimator) const;

  /** @brief Write report as JSON for @ref scripts/fim_visualizer.py. */
  static bool WriteReportJson(const ObservabilityReport& report,
                              const std::string& path);
};

}  // namespace clic_calib
