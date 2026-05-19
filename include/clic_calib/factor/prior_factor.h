/*
 * clic_calib — §4.6 Extrinsic prior factor (canonical).
 * See doc/DERIVATIONS.md §4.6.
 *
 * Parameter block: 6-D se(3) tangent ξ with T_XW = Exp(ξ) (right-trivialized).
 */

#pragma once

#include <clic_calib/utils/sophus_utils.hpp>

#include <ceres/ceres.h>
#include <Eigen/Core>

namespace clic_calib {
namespace analytic_derivative {

class ExtrinsicPriorFactor : public ceres::SizedCostFunction<6, 6> {
 public:
  ExtrinsicPriorFactor(const SE3d& T_XW_prior,
                       const Eigen::Matrix<double, 6, 1>& sqrt_info)
      : T_prior_(T_XW_prior), sqrt_info_(sqrt_info) {}

  virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const override {
    Eigen::Map<const Eigen::Matrix<double, 6, 1>> xi(parameters[0]);
    const SE3d T_XW = SE3d::exp(xi);

    const SE3d T_err = T_XW.inverse() * T_prior_;
    const Eigen::Matrix<double, 6, 1> log_err = T_err.log();

    Eigen::Map<Eigen::Matrix<double, 6, 1>> r(residuals);
    r = sqrt_info_.asDiagonal() * log_err;

    if (!jacobians || !jacobians[0]) {
      return true;
    }

    Eigen::Matrix<double, 6, 6> J_log;
    Sophus::rightJacobianInvSE3Decoupled(log_err, J_log);

    Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> J_xi(jacobians[0]);
    J_xi = sqrt_info_.asDiagonal() * J_log * (-T_XW.inverse().Adj());
    return true;
  }

 private:
  SE3d T_prior_;
  Eigen::Matrix<double, 6, 1> sqrt_info_;
};

}  // namespace analytic_derivative
}  // namespace clic_calib
