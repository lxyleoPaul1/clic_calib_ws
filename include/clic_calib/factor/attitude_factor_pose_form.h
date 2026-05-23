#pragma once

#include <clic_calib/factor/so3_spline_view.h>
#include <clic_calib/spline/spline_segment.h>
#include <clic_calib/utils/sophus_utils.hpp>

#include <ceres/ceres.h>
#include <Eigen/Core>

namespace clic_calib {
namespace analytic_derivative {

/**
 * Stage-1 attitude factor (probe): r = Log(R(t)·R_obs^{-1}), body Σ rotated to
 * tangent frame via R_obs. Jacobian matches trajectory_value_factor pose pattern.
 */
class AttitudeFactorPoseForm : public ceres::CostFunction, public So3SplineView {
 public:
  using SO3View = So3SplineView;
  using Vec3 = Eigen::Vector3d;
  using Mat3 = Eigen::Matrix3d;

  AttitudeFactorPoseForm(int64_t t_ns, const SO3d& R_WB_observed,
                         const Eigen::Matrix3d& covariance_body,
                         const SplineSegmentMeta<SplineOrder>& spline_meta)
      : t_ns_(t_ns),
        R_obs_(R_WB_observed),
        spline_meta_(spline_meta) {
    set_num_residuals(3);
    for (int i = 0; i < SplineOrder; ++i) {
      mutable_parameter_block_sizes()->push_back(4);
    }
    sqrt_info_ = ComputeSqrtInfo(covariance_body, R_obs_);
  }

  bool Evaluate(double const* const* parameters, double* residuals,
                double** jacobians) const override {
    typename SO3View::JacobianStruct J_R;
    SO3d R_WB;
    if (jacobians) {
      R_WB = SO3View::EvaluateRp(t_ns_, spline_meta_, parameters, &J_R);
    } else {
      R_WB = SO3View::EvaluateRp(t_ns_, spline_meta_, parameters);
    }

    const SO3d R_err = R_WB * R_obs_.inverse();
    const Vec3 e = R_err.log();
    Eigen::Map<Vec3> r(residuals);
    r = sqrt_info_ * e;

    if (!jacobians) {
      return true;
    }

    Mat3 Jr_inv;
    Sophus::rightJacobianInvSO3(e, Jr_inv);
    // Right-trivialized δ on R_WB: R' = R_WB exp(δ) ⇒ R_err' = R_err exp(Ad(R_obs) δ).
    const Mat3 chain = sqrt_info_ * Jr_inv * R_obs_.matrix();

    for (size_t i = 0; i < SplineOrder; ++i) {
      if (jacobians[i]) {
        Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> J_knot(
            jacobians[i]);
        J_knot.setZero();
        J_knot.block<3, 3>(0, 0) = chain * J_R.d_val_d_knot[i];
      }
    }
    return true;
  }

 private:
  using WhiteningMatrix = Eigen::Matrix3d;

  static WhiteningMatrix ComputeSqrtInfo(const Eigen::Matrix3d& covariance_body,
                                         const SO3d& R_obs) {
    const Mat3 cov_tangent = R_obs.matrix() * covariance_body * R_obs.matrix().transpose();
    Eigen::LLT<Eigen::Matrix3d> llt(cov_tangent);
    return llt.matrixL().solve(Eigen::Matrix3d::Identity());
  }

  int64_t t_ns_;
  SO3d R_obs_;
  SplineSegmentMeta<SplineOrder> spline_meta_;
  WhiteningMatrix sqrt_info_;
};

}  // namespace analytic_derivative
}  // namespace clic_calib
