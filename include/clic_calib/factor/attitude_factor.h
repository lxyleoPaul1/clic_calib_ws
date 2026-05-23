/*
 * clic_calib — Stage-1 anisotropic attitude residual on SO(3) B-spline knots.
 * r = Log(R_WB(t)^{-1} · R_obs) whitened by chol(Σ_att)^{-1} in body tangent.
 */

#pragma once

#include <clic_calib/factor/so3_spline_view.h>
#include <clic_calib/spline/spline_segment.h>
#include <clic_calib/utils/sophus_utils.hpp>

#include <ceres/ceres.h>
#include <Eigen/Core>

namespace clic_calib {
namespace analytic_derivative {

class AttitudeFactor : public ceres::CostFunction, public So3SplineView {
 public:
  using SO3View = So3SplineView;
  using Vec3 = Eigen::Vector3d;
  using Mat3 = Eigen::Matrix3d;

  AttitudeFactor(int64_t t_ns, const SO3d& R_WB_observed,
                 const Eigen::Matrix3d& covariance,
                 const SplineSegmentMeta<SplineOrder>& spline_meta)
      : t_ns_(t_ns),
        R_obs_(R_WB_observed),
        spline_meta_(spline_meta) {
    set_num_residuals(3);
    for (int i = 0; i < SplineOrder; ++i) {
      mutable_parameter_block_sizes()->push_back(4);
    }
    sqrt_info_ = ComputeSqrtInfo(covariance);
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

    const SO3d R_err = R_WB.inverse() * R_obs_;
    const Vec3 e = R_err.log();
    Eigen::Map<Vec3> r(residuals);
    r = sqrt_info_ * e;

    if (!jacobians) {
      return true;
    }

    Mat3 Jr_inv;
    Sophus::rightJacobianInvSO3(e, Jr_inv);
    const Mat3 chain = -sqrt_info_ * Jr_inv;

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

  static WhiteningMatrix ComputeSqrtInfo(const Eigen::Matrix3d& covariance) {
    Eigen::LLT<Eigen::Matrix3d> llt(covariance);
    return llt.matrixL().solve(Eigen::Matrix3d::Identity());
  }

  int64_t t_ns_;
  SO3d R_obs_;
  SplineSegmentMeta<SplineOrder> spline_meta_;
  WhiteningMatrix sqrt_info_;
};

}  // namespace analytic_derivative
}  // namespace clic_calib
