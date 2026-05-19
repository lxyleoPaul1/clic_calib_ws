/*
 * clic_calib — §4.3 LiDAR point-to-sphere-surface residual (canonical).
 * See doc/DERIVATIONS.md §4.3.
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

/** @brief §4.3: d p_G^W / dt (analytic spline derivatives). */
inline Eigen::Vector3d sphere_dp_gw_dt(
    int64_t t_ns, const SplineSegmentMeta<SplineOrder>& meta,
    double const* const* rot_knots, double const* const* pos_knots,
    const Eigen::Vector3d& L_B_to_G,
    RdSplineView::JacobianStruct* J_pos = nullptr,
    So3SplineView::JacobianStruct* J_rot = nullptr,
    So3SplineView::JacobianStruct* J_omega = nullptr) {
  SO3d R_WB = So3SplineView::EvaluateRotation(t_ns, meta, rot_knots, J_rot);
  Eigen::Vector3d v_WB = RdSplineView::velocity(t_ns, meta, pos_knots, J_pos);
  Eigen::Vector3d omega_b =
      So3SplineView::VelocityBody(t_ns, meta, rot_knots, J_omega);
  return v_WB + R_WB.matrix() * SO3d::hat(omega_b) * L_B_to_G;
}

class SphereImplicitFactor : public ceres::CostFunction,
                             public So3SplineView,
                             public RdSplineView {
 public:
  using Vec3 = Eigen::Vector3d;
  using Mat3 = Eigen::Matrix3d;
  using SO3View = So3SplineView;
  using R3View = RdSplineView;

  SphereImplicitFactor(int64_t bar_t_ns, const Vec3& q_L, const Vec3& L_B_to_G,
                       double R_ball, double sigma_r,
                       const SplineSegmentMeta<SplineOrder>& spline_meta)
      : bar_t_ns_(bar_t_ns),
        q_L_(q_L),
        L_B_to_G_(L_B_to_G),
        R_ball_(R_ball),
        inv_sigma_r_(1.0 / sigma_r),
        spline_meta_(spline_meta) {
    set_num_residuals(1);
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

  virtual bool Evaluate(double const* const* parameters, double* residuals,
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
      R_WB = SO3View::EvaluateRotation(t_ns, spline_meta_, rot_knots, &J_R);
      p_WB = R3View::evaluate(t_ns, spline_meta_, pos_knots, &J_p);
    } else {
      R_WB = SO3View::EvaluateRotation(t_ns, spline_meta_, rot_knots);
      p_WB = R3View::evaluate(t_ns, spline_meta_, pos_knots);
    }

    const Vec3 p_G_W = p_WB + R_WB * L_B_to_G_;
    const Vec3 p_G_L = T_LW * p_G_W;
    const Vec3 diff = q_L_ - p_G_L;
    const double dist = diff.norm();
    const double inv_dist = dist > 1e-12 ? 1.0 / dist : 0.0;
    const Vec3 n = inv_dist * diff;

    residuals[0] = inv_sigma_r_ * (dist - R_ball_);
    const Vec3 jac_p_G_L = -n;

    if (!jacobians) {
      return true;
    }

    const Mat3 R_LW = T_LW.so3().matrix();
    const Mat3 R_hat_L = R_WB.matrix() * SO3d::hat(L_B_to_G_);

    if (jacobians[0]) {
      R3View::JacobianStruct J_v;
      So3SplineView::JacobianStruct J_rot_unused, J_omega;
      const Vec3 dp_dt =
          sphere_dp_gw_dt(t_ns, spline_meta_, rot_knots, pos_knots, L_B_to_G_,
                          &J_v, &J_rot_unused, &J_omega);
      Eigen::Map<Eigen::Matrix<double, 1, 1>> J_td(jacobians[0]);
      J_td(0, 0) = -inv_sigma_r_ * jac_p_G_L.dot(R_LW * dp_dt);
    }

    if (jacobians[1]) {
      Eigen::Map<Eigen::Matrix<double, 1, 4, Eigen::RowMajor>> J_q(jacobians[1]);
      J_q.setZero();
      J_q.block<1, 3>(0, 0) =
          inv_sigma_r_ * jac_p_G_L.transpose() * (-R_LW * SO3d::hat(p_G_W));
    }

    if (jacobians[2]) {
      Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> J_t(jacobians[2]);
      J_t = inv_sigma_r_ * jac_p_G_L.transpose();
    }

    for (int i = 0; i < SplineOrder; ++i) {
      const size_t idx_r = 3 + J_R.start_idx + i;
      if (jacobians[idx_r]) {
        Eigen::Map<Eigen::Matrix<double, 1, 4, Eigen::RowMajor>> J_knot(
            jacobians[idx_r]);
        J_knot.setZero();
        J_knot.block<1, 3>(0, 0) =
            inv_sigma_r_ * jac_p_G_L.transpose() *
            (-R_LW * R_hat_L * J_R.d_val_d_knot[i]);
      }

      const size_t idx_p = 3 + knot_num + J_p.start_idx + i;
      if (jacobians[idx_p]) {
        Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> J_knot(
            jacobians[idx_p]);
        J_knot = inv_sigma_r_ * jac_p_G_L.transpose() * R_LW *
                 (J_p.d_val_d_knot[i] * Mat3::Identity());
      }
    }

    return true;
  }

 private:
  int64_t bar_t_ns_;
  Vec3 q_L_;
  Vec3 L_B_to_G_;
  double R_ball_;
  double inv_sigma_r_;
  SplineSegmentMeta<SplineOrder> spline_meta_;
};

}  // namespace analytic_derivative
}  // namespace clic_calib
