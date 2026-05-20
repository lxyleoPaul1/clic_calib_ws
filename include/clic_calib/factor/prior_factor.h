/*
 * clic_calib — §4.6 Extrinsic prior factor (canonical).
 * See doc/DERIVATIONS.md §4.6.
 *
 * Parameter blocks: unit quaternion (4) + translation (3), matching other
 * extrinsic factors and LieLocalParameterization SO(3) numeric tests.
 */

#pragma once

#include <clic_calib/utils/sophus_utils.hpp>

#include <ceres/ceres.h>
#include <Eigen/Core>

namespace clic_calib {
namespace analytic_derivative {

class ExtrinsicPriorFactor : public ceres::CostFunction {
 public:
  ExtrinsicPriorFactor(const SE3d& T_XW_prior,
                       const Eigen::Matrix<double, 6, 1>& sqrt_info)
      : T_prior_(T_XW_prior), sqrt_info_(sqrt_info) {
    set_num_residuals(6);
    mutable_parameter_block_sizes()->push_back(4);
    mutable_parameter_block_sizes()->push_back(3);
  }

  virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const override {
    const Eigen::Map<const Eigen::Quaterniond> q_XW(parameters[0]);
    const Eigen::Map<const Eigen::Vector3d> t_XW(parameters[1]);
    const SE3d T_XW(q_XW, t_XW);

    const SE3d T_err = T_XW.inverse() * T_prior_;
    const Eigen::Matrix<double, 6, 1> log_err = T_err.log();

    Eigen::Map<Eigen::Matrix<double, 6, 1>> r(residuals);
    r = sqrt_info_.asDiagonal() * log_err;

    if (!jacobians) {
      return true;
    }

    Eigen::Matrix<double, 6, 6> J_log;
    Sophus::rightJacobianInvSE3Decoupled(log_err, J_log);
    const Eigen::Matrix<double, 6, 6> Adj = T_XW.inverse().Adj();

    Eigen::Matrix<double, 6, 3> E_omega;
    E_omega.setZero();
    E_omega.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 6, 3> E_upsilon;
    E_upsilon.setZero();
    E_upsilon.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

    const Eigen::Matrix<double, 6, 3> J_omega =
        sqrt_info_.asDiagonal() * J_log * (-E_omega);
    const Eigen::Matrix<double, 6, 3> J_upsilon =
        sqrt_info_.asDiagonal() * J_log * (-Adj * E_upsilon);

    if (jacobians[0]) {
      Eigen::Map<Eigen::Matrix<double, 6, 4, Eigen::RowMajor>> J_q(
          jacobians[0]);
      J_q.setZero();
      J_q.block<6, 3>(0, 0) = J_omega;
    }

    if (jacobians[1]) {
      Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>> J_t(
          jacobians[1]);
      J_t = J_upsilon;
    }

    return true;
  }

 private:
  SE3d T_prior_;
  Eigen::Matrix<double, 6, 1> sqrt_info_;
};

}  // namespace analytic_derivative
}  // namespace clic_calib
