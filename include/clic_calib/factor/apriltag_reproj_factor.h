/*
 * clic_calib — §4.4 AprilTag reprojection residual (canonical).
 * See doc/DERIVATIONS.md §4.4.
 */

#pragma once

#include <clic_calib/factor/rd_spline_view.h>
#include <clic_calib/factor/so3_spline_view.h>
#include <clic_calib/factor/sphere_implicit_factor.h>
#include <clic_calib/spline/spline_segment.h>
#include <clic_calib/utils/camera_projection.h>
#include <clic_calib/utils/sophus_utils.hpp>

#include <ceres/ceres.h>
#include <Eigen/Core>

namespace clic_calib {
namespace analytic_derivative {

/** @brief §4.4 time-offset chain: d p_M^W / dt. */
inline Eigen::Vector3d apriltag_dp_mw_dt(
    int64_t t_ns, const SplineSegmentMeta<SplineOrder>& meta,
    double const* const* rot_knots, double const* const* pos_knots,
    const Eigen::Vector3d& L_B_to_G_M,
    RdSplineView::JacobianStruct* J_pos = nullptr,
    So3SplineView::JacobianStruct* J_rot = nullptr,
    So3SplineView::JacobianStruct* J_omega = nullptr) {
  return sphere_dp_gw_dt(t_ns, meta, rot_knots, pos_knots, L_B_to_G_M, J_pos,
                         J_rot, J_omega);
}

/** @brief One corner reprojection factor (2-D residual). */
class AprilTagReprojFactor : public ceres::CostFunction,
                             public So3SplineView,
                             public RdSplineView {
 public:
  using Vec2 = Eigen::Vector2d;
  using Vec3 = Eigen::Vector3d;
  using Mat3 = Eigen::Matrix3d;
  using SO3View = So3SplineView;
  using R3View = RdSplineView;

  AprilTagReprojFactor(int64_t bar_t_ns, const Vec2& u_obs,
                       const Vec3& L_B_to_G_M, const PinholeIntrinsics& K,
                       const RadtanDistortion& dist, double sigma_pix,
                       const SplineSegmentMeta<SplineOrder>& spline_meta)
      : bar_t_ns_(bar_t_ns),
        u_obs_(u_obs),
        L_B_to_G_M_(L_B_to_G_M),
        K_(K),
        dist_(dist),
        inv_sigma_pix_(1.0 / sigma_pix),
        spline_meta_(spline_meta) {
    set_num_residuals(2);
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

    const Eigen::Map<const Eigen::Quaterniond> q_CW(parameters[1]);
    const Eigen::Map<const Vec3> t_CW(parameters[2]);
    const SE3d T_CW(q_CW, t_CW);

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

    const Vec3 p_M_W = p_WB + R_WB * L_B_to_G_M_;
    const Vec3 p_M_C = T_CW * p_M_W;

    Eigen::Matrix<double, 2, 3> J_proj_pC;
    const Vec2 u_pred =
        ProjectRadtan(p_M_C, K_, dist_, jacobians ? &J_proj_pC : nullptr);

    Eigen::Map<Vec2> residual(residuals);
    residual = inv_sigma_pix_ * (u_obs_ - u_pred);

    if (!jacobians) {
      return true;
    }

    const Mat3 R_CW = T_CW.so3().matrix();
    const Mat3 R_hat = R_WB.matrix() * SO3d::hat(L_B_to_G_M_);
    const Eigen::Matrix<double, 2, 3> J_p_MW = J_proj_pC * R_CW;
    const Eigen::Matrix<double, 2, 3> J_p_MW_R =
        J_proj_pC * R_CW * R_hat;

    if (jacobians[0]) {
      R3View::JacobianStruct J_v;
      So3SplineView::JacobianStruct J_rot_unused, J_omega;
      const Vec3 dp_dt =
          apriltag_dp_mw_dt(t_ns, spline_meta_, rot_knots, pos_knots,
                            L_B_to_G_M_, &J_v, &J_rot_unused, &J_omega);
      Eigen::Map<Eigen::Matrix<double, 2, 1>> J_td(jacobians[0]);
      J_td = inv_sigma_pix_ * J_proj_pC * R_CW * dp_dt;
    }

    if (jacobians[1]) {
      Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> J_q(jacobians[1]);
      J_q.setZero();
      J_q.block<2, 3>(0, 0) =
          inv_sigma_pix_ * J_proj_pC * (R_CW * SO3d::hat(p_M_W));
    }

    if (jacobians[2]) {
      Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> J_t(jacobians[2]);
      J_t = -inv_sigma_pix_ * J_proj_pC;
    }

    for (int i = 0; i < SplineOrder; ++i) {
      const size_t idx_r = 3 + J_R.start_idx + i;
      if (jacobians[idx_r]) {
        Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> J_knot(
            jacobians[idx_r]);
        J_knot.setZero();
        J_knot.block<2, 3>(0, 0) = inv_sigma_pix_ * J_p_MW_R * J_R.d_val_d_knot[i];
      }

      const size_t idx_p = 3 + knot_num + J_p.start_idx + i;
      if (jacobians[idx_p]) {
        Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> J_knot(
            jacobians[idx_p]);
        J_knot = -inv_sigma_pix_ * J_p_MW * (J_p.d_val_d_knot[i] * Mat3::Identity());
      }
    }

    return true;
  }

 private:
  int64_t bar_t_ns_;
  Vec2 u_obs_;
  Vec3 L_B_to_G_M_;
  PinholeIntrinsics K_;
  RadtanDistortion dist_;
  double inv_sigma_pix_;
  SplineSegmentMeta<SplineOrder> spline_meta_;
};

}  // namespace analytic_derivative
}  // namespace clic_calib
