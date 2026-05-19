#pragma once

#include <ceres/ceres.h>

#include <Eigen/Core>
#include <cmath>
#include <vector>

namespace clic_calib {
namespace test {

inline double RelativeJacobianError(double analytic, double numeric,
                                    double abs_tol = 1e-8) {
  const double denom = std::max(std::abs(analytic), std::abs(numeric));
  if (denom < abs_tol) {
    return std::abs(analytic - numeric);
  }
  return std::abs(analytic - numeric) / denom;
}

/** @brief 5-point central difference w.r.t. each scalar parameter component. */
inline void NumericalJacobian(ceres::CostFunction* cost,
                              const std::vector<double*>& param_ptrs,
                              const std::vector<int>& block_sizes,
                              double* residuals_nominal,
                              Eigen::MatrixXd* numeric_jacobian,
                              double step = 1e-6) {
  const int num_residuals = cost->num_residuals();
  int total_params = 0;
  for (int s : block_sizes) {
    total_params += s;
  }
  numeric_jacobian->resize(num_residuals, total_params);
  numeric_jacobian->setZero();

  std::vector<std::vector<double>> backup(block_sizes.size());
  for (size_t b = 0; b < block_sizes.size(); ++b) {
    backup[b].assign(param_ptrs[b], param_ptrs[b] + block_sizes[b]);
  }

  int col = 0;
  for (size_t b = 0; b < block_sizes.size(); ++b) {
    for (int d = 0; d < block_sizes[b]; ++d) {
      double r_m2[32], r_m1[32], r_p1[32], r_p2[32];
      const double h = step;

      param_ptrs[b][d] = backup[b][d] - 2 * h;
      cost->Evaluate(param_ptrs.data(), r_m2, nullptr);
      param_ptrs[b][d] = backup[b][d] - h;
      cost->Evaluate(param_ptrs.data(), r_m1, nullptr);
      param_ptrs[b][d] = backup[b][d] + h;
      cost->Evaluate(param_ptrs.data(), r_p1, nullptr);
      param_ptrs[b][d] = backup[b][d] + 2 * h;
      cost->Evaluate(param_ptrs.data(), r_p2, nullptr);
      param_ptrs[b][d] = backup[b][d];

      for (int r = 0; r < num_residuals; ++r) {
        (*numeric_jacobian)(r, col) =
            (-r_p2[r] + 8 * r_p1[r] - 8 * r_m1[r] + r_m2[r]) / (12 * h);
      }
      ++col;
    }
  }
  (void)residuals_nominal;
}

/** @brief Compare Ceres analytic Jacobians (row-major blocks) to numeric. */
inline bool CompareJacobians(
    ceres::CostFunction* cost, const std::vector<double*>& param_ptrs,
    const std::vector<int>& block_sizes, double tol = 1e-5) {
  const int num_residuals = cost->num_residuals();
  std::vector<double> residuals(num_residuals);
  std::vector<double*> jac_ptrs(param_ptrs.size());
  std::vector<std::vector<double>> jac_storage(param_ptrs.size());
  for (size_t b = 0; b < param_ptrs.size(); ++b) {
    jac_storage[b].resize(num_residuals * block_sizes[b]);
    jac_ptrs[b] = jac_storage[b].data();
  }

  cost->Evaluate(param_ptrs.data(), residuals.data(), jac_ptrs.data());

  Eigen::MatrixXd num_jac;
  NumericalJacobian(cost, param_ptrs, block_sizes, residuals.data(), &num_jac);

  int col = 0;
  for (size_t b = 0; b < block_sizes.size(); ++b) {
    for (int r = 0; r < num_residuals; ++r) {
      for (int d = 0; d < block_sizes[b]; ++d) {
        const double analytic = jac_storage[b][r * block_sizes[b] + d];
        const double numeric = num_jac(r, col + d);
        if (RelativeJacobianError(analytic, numeric) > tol) {
          return false;
        }
      }
    }
    col += block_sizes[b];
  }
  return true;
}

}  // namespace test
}  // namespace clic_calib
