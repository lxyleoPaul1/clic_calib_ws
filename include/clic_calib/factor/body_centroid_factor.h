/*
 * clic_calib — §5.3 body-centroid residual (Stage-1 / full-joint).
 */

#pragma once

#include <clic_calib/factor/fixed_traj_sphere_factor.h>
#include <clic_calib/factor/rd_spline_view.h>
#include <clic_calib/factor/so3_spline_view.h>
#include <clic_calib/factor/sphere_implicit_factor.h>
#include <clic_calib/spline/spline_segment.h>
#include <clic_calib/utils/sophus_utils.hpp>

#include <ceres/ceres.h>
#include <Eigen/Core>

namespace clic_calib {
namespace analytic_derivative {

class BodyCentroidFactor : public ceres::CostFunction,
                           public So3SplineView,
                           public RdSplineView {
 public:
  using Vec3 = Eigen::Vector3d;
  using Mat3 = Eigen::Matrix3d;
  using SO3View = So3SplineView;
  using R3View = RdSplineView;

  BodyCentroidFactor(int64_t bar_t_ns, const Vec3& c_L, const Vec3& L_B_to_body,
                     const Mat3& sqrt_info,
                     const SplineSegmentMeta<SplineOrder>& spline_meta)
      : bar_t_ns_(bar_t_ns),
        c_L_(c_L),
        L_B_to_body_(L_B_to_body),
        sqrt_info_(sqrt_info),
        spline_meta_(spline_meta) {
    set_num_residuals(3);
    mutable_parameter_block_sizes()->push_back(1);
    mutable_parameter_block_sizes()->push_back(4);
    mutable_parameter_block_sizes()->push_back(3);
    for (int i = 0; i < SplineOrder; ++i) {
      mutable_parameter_block_sizes()->push_back(4);
    }
    for (int i = 0; i < SplineOrder; ++i) {
      mutable_parameter_block_sizes()->push_back(3);
    }
  }

  bool Evaluate(double const* const* parameters, double* residuals,
                double** jacobians) const override {
    const double t_d_s = parameters[0][0];
    const int64_t t_ns = bar_t_ns_ - static_cast<int64_t>(t_d_s * S_TO_NS);

    const Eigen::Map<const Eigen::Quaterniond> q_LW(parameters[1]);
    const Eigen::Map<const Vec3> t_LW(parameters[2]);
    const SE3d T_LW(q_LW, t_LW);

    static constexpr size_t knot_num = SplineOrder;
    const double* const* rot_knots = parameters + 3;
    const double* const* pos_knots = parameters + 3 + knot_num;

    typename SO3View::JacobianStruct J_R;
    typename R3View::JacobianStruct J_p;

    SO3d R_WB;
    Vec3 p_WB;
    if (jacobians) {
      R_WB = SO3View::EvaluateRp(t_ns, spline_meta_, rot_knots, &J_R);
      p_WB = R3View::evaluate(t_ns, spline_meta_, pos_knots, &J_p);
    } else {
      R_WB = SO3View::EvaluateRp(t_ns, spline_meta_, rot_knots);
      p_WB = R3View::evaluate(t_ns, spline_meta_, pos_knots);
    }

    const Vec3 p_W = p_WB + R_WB * L_B_to_body_;
    const Vec3 c_hat = T_LW * p_W;
    const Vec3 r = c_L_ - c_hat;
    Eigen::Map<Vec3> e(residuals);
    e = sqrt_info_ * r;

    if (!jacobians) {
      return true;
    }

    const Mat3 R_LW = T_LW.so3().matrix();
    const Mat3 R_hat_L = R_WB.matrix() * SO3d::hat(L_B_to_body_);

    if (jacobians[0]) {
      R3View::JacobianStruct J_v;
      So3SplineView::JacobianStruct J_rot_unused, J_omega;
      const Vec3 v_W =
          sphere_dp_gw_dt(t_ns, spline_meta_, rot_knots, pos_knots, L_B_to_body_,
                          &J_v, &J_rot_unused, &J_omega);
      Eigen::Map<Eigen::Matrix<double, 3, 1>> J_td(jacobians[0]);
      J_td = sqrt_info_ * (R_LW * v_W);
    }

    if (jacobians[1]) {
      Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> J_q(jacobians[1]);
      J_q.setZero();
      J_q.block<3, 3>(0, 0) = sqrt_info_ * (R_LW * SO3d::hat(p_W));
    }

    if (jacobians[2]) {
      Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> J_t(jacobians[2]);
      J_t = sqrt_info_ * (-Mat3::Identity());
    }

    for (int i = 0; i < SplineOrder; ++i) {
      const size_t idx_r = 3 + J_R.start_idx + i;
      if (jacobians[idx_r]) {
        Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> J_knot(
            jacobians[idx_r]);
        J_knot.setZero();
        J_knot.block<3, 3>(0, 0) =
            sqrt_info_ * (R_LW * R_hat_L * J_R.d_val_d_knot[i]);
      }

      const size_t idx_p = 3 + knot_num + J_p.start_idx + i;
      if (jacobians[idx_p]) {
        Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> J_knot(
            jacobians[idx_p]);
        J_knot.setZero();
        J_knot.block<3, 3>(0, 0) =
            sqrt_info_ * (-R_LW * J_p.d_val_d_knot[i] * Mat3::Identity());
      }
    }

    return true;
  }

 private:
  int64_t bar_t_ns_;
  Vec3 c_L_;
  Vec3 L_B_to_body_;
  Mat3 sqrt_info_;
  SplineSegmentMeta<SplineOrder> spline_meta_;
};

}  // namespace analytic_derivative
}  // namespace clic_calib
