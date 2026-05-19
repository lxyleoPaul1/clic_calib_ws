/*
 * clic_calib — §4.2 RTK position residual (canonical).
 * See doc/DERIVATIONS.md §4.2.
 */

#pragma once

#include <clic_calib/factor/rd_spline_view.h>
#include <clic_calib/factor/so3_spline_view.h>
#include <clic_calib/spline/spline_segment.h>
#include <clic_calib/utils/sophus_utils.hpp>

#include <ceres/ceres.h>
#include <Eigen/Core>

namespace clic_calib {
namespace analytic_derivative {

class RTKPositionFactor : public ceres::CostFunction,
                          public So3SplineView,
                          public RdSplineView {
 public:
  using SO3View = So3SplineView;
  using R3View = RdSplineView;
  using Vec3 = Eigen::Vector3d;
  using Mat3 = Eigen::Matrix3d;

  RTKPositionFactor(int64_t t_ns, const Vec3& p_A_W_observed,
                    const Eigen::Matrix3d& covariance,
                    const Vec3& L_B_to_A,
                    const SplineSegmentMeta<SplineOrder>& spline_meta)
      : t_ns_(t_ns),
        p_obs_(p_A_W_observed),
        L_B_to_A_(L_B_to_A),
        spline_meta_(spline_meta) {
    set_num_residuals(3);
    for (int i = 0; i < SplineOrder; ++i) {
      mutable_parameter_block_sizes()->push_back(4);
    }
    for (int i = 0; i < SplineOrder; ++i) {
      mutable_parameter_block_sizes()->push_back(3);
    }
    sqrt_info_ = ComputeSqrtInfo(covariance);
  }

  virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const override {
    typename SO3View::JacobianStruct J_R;
    typename R3View::JacobianStruct J_p;

    static constexpr size_t knot_num = SplineOrder;
    const size_t p_offset = knot_num;

    SO3d R_WB;
    Vec3 p_WB;
    if (jacobians) {
      R_WB = SO3View::EvaluateRotation(t_ns_, spline_meta_, parameters, &J_R);
      p_WB = R3View::evaluate(t_ns_, spline_meta_, parameters + p_offset, &J_p);
    } else {
      R_WB = SO3View::EvaluateRotation(t_ns_, spline_meta_, parameters);
      p_WB = R3View::evaluate(t_ns_, spline_meta_, parameters + p_offset);
    }

    const Vec3 p_A_pred = p_WB + R_WB * L_B_to_A_;
    const Vec3 r = p_obs_ - p_A_pred;
    Eigen::Map<Vec3> residual(residuals);
    residual = sqrt_info_ * r;

    if (!jacobians) {
      return true;
    }

    for (size_t i = 0; i < 2 * knot_num; ++i) {
      if (jacobians[i]) {
        Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> J_rot(
            jacobians[i]);
        J_rot.setZero();
      }
      if (jacobians[i + knot_num]) {
        Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> J_pos(
            jacobians[i + knot_num]);
        J_pos.setZero();
      }
    }

    const Mat3 R_hat_L = R_WB.matrix() * SO3d::hat(L_B_to_A_);

    for (int i = 0; i < SplineOrder; ++i) {
      const size_t idx_r = J_R.start_idx + i;
      if (jacobians[idx_r]) {
        Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> J_knot(
            jacobians[idx_r]);
        J_knot.setZero();
        J_knot.block<3, 3>(0, 0) =
            sqrt_info_ * (-R_hat_L * J_R.d_val_d_knot[i]);
      }

      const size_t idx_p = knot_num + J_p.start_idx + i;
      if (jacobians[idx_p]) {
        Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> J_knot(
            jacobians[idx_p]);
        J_knot.setZero();
        J_knot.block<3, 3>(0, 0) =
            sqrt_info_ * (-J_p.d_val_d_knot[i] * Mat3::Identity());
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
  Vec3 p_obs_;
  Vec3 L_B_to_A_;
  SplineSegmentMeta<SplineOrder> spline_meta_;
  WhiteningMatrix sqrt_info_;
};

using RtkPositionFactor = RTKPositionFactor;

}  // namespace analytic_derivative
}  // namespace clic_calib
