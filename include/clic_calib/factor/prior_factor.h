/*
 * clic_calib — §4.6 Extrinsic prior factor (canonical).
 * See doc/DERIVATIONS.md §4.6.
 */

#pragma once

#include <clic_calib/utils/sophus_utils.hpp>

#include <ceres/ceres.h>
#include <Eigen/Core>

namespace clic_calib {
namespace analytic_derivative {

class ExtrinsicPriorFactor : public ceres::SizedCostFunction<6, 4, 3> {
 public:
  using Mat3 = Eigen::Matrix3d;
  ExtrinsicPriorFactor(const SE3d& T_XW_prior,
                       const Eigen::Matrix<double, 6, 1>& sqrt_info)
      : T_prior_(T_XW_prior), sqrt_info_(sqrt_info) {}

  virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const override {
    const Eigen::Map<const Eigen::Quaterniond> q(parameters[0]);
    const Eigen::Map<const Eigen::Vector3d> t(parameters[1]);
    const SE3d T_XW(q, t);

    const SE3d T_err = T_XW.inverse() * T_prior_;
    const Eigen::Matrix<double, 6, 1> log_err = T_err.log();

    Eigen::Map<Eigen::Matrix<double, 6, 1>> r(residuals);
    r = sqrt_info_.asDiagonal() * log_err;

    if (!jacobians) {
      return true;
    }

    Eigen::Matrix<double, 6, 6> J_log;
    Sophus::rightJacobianInvSE3Decoupled(log_err, J_log);

    const Mat3 R_err = T_err.so3().matrix();

    if (jacobians[0]) {
      Eigen::Map<Eigen::Matrix<double, 6, 4, Eigen::RowMajor>> J_q(jacobians[0]);
      J_q.setZero();
      J_q.block<3, 3>(0, 0) =
          sqrt_info_.head<3>().asDiagonal() * J_log.block<3, 3>(0, 0) * (-R_err);
      J_q.block<3, 3>(3, 0) =
          sqrt_info_.tail<3>().asDiagonal() * J_log.block<3, 3>(3, 0) * (-R_err);
    }

    if (jacobians[1]) {
      Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>> J_t(jacobians[1]);
      J_t.block<3, 3>(0, 0) =
          sqrt_info_.head<3>().asDiagonal() * J_log.block<3, 3>(0, 3);
      J_t.block<3, 3>(3, 0) =
          sqrt_info_.tail<3>().asDiagonal() * J_log.block<3, 3>(3, 3);
    }

    return true;
  }

 private:
  SE3d T_prior_;
  Eigen::Matrix<double, 6, 1> sqrt_info_;
};

}  // namespace analytic_derivative
}  // namespace clic_calib
