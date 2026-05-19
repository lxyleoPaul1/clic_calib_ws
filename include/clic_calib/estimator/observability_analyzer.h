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

namespace clic_calib {

struct ObservabilityReport {
  Eigen::MatrixXd information_matrix;
  double lambda_min = 0.0;
  double condition_number = 0.0;
  double pdop_ext = 0.0;
  Eigen::Vector3d worst_direction = Eigen::Vector3d::Zero();
};

class ObservabilityAnalyzer {
 public:
  ObservabilityReport AnalyzeCurrentProblem() const;
};

}  // namespace clic_calib
