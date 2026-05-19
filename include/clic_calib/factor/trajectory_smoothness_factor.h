/*
 * clic_calib — §4.5 Trajectory smoothness regularizer (canonical).
 * See doc/DERIVATIONS.md §4.5.
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

class TrajectorySmoothnessFactor : public ceres::CostFunction,
                                   public So3SplineView,
                                   public RdSplineView {
 public:
  using Vec3 = Eigen::Vector3d;

  TrajectorySmoothnessFactor(int64_t t_ns, double alpha_p, double alpha_R,
                             double dt_s,
                             const SplineSegmentMeta<SplineOrder>& spline_meta)
      : t_ns_(t_ns),
        sqrt_alpha_p_dt_(std::sqrt(alpha_p * dt_s)),
        sqrt_alpha_R_dt_(std::sqrt(alpha_R * dt_s)),
        spline_meta_(spline_meta) {
    set_num_residuals(6);
    for (int i = 0; i < SplineOrder; ++i) {
      mutable_parameter_block_sizes()->push_back(4);
    }
    for (int i = 0; i < SplineOrder; ++i) {
      mutable_parameter_block_sizes()->push_back(3);
    }
  }

  virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const override {
    static constexpr size_t knot_num = SplineOrder;
    const double* const* rot_knots = parameters;
    const double* const* pos_knots = parameters + knot_num;

    typename RdSplineView::JacobianStruct J_acc;
    typename So3SplineView::JacobianStruct J_omega;

    const Vec3 accel =
        RdSplineView::acceleration(t_ns_, spline_meta_, pos_knots, &J_acc);
    const Vec3 omega =
        So3SplineView::VelocityBody(t_ns_, spline_meta_, rot_knots, &J_omega);

    Eigen::Map<Eigen::Matrix<double, 6, 1>> r(residuals);
    r.head<3>() = sqrt_alpha_p_dt_ * accel;
    r.tail<3>() = sqrt_alpha_R_dt_ * omega;

    if (!jacobians) {
      return true;
    }

    for (size_t i = 0; i < 2 * knot_num; ++i) {
      if (jacobians[i]) {
        if (i < knot_num) {
          Eigen::Map<Eigen::Matrix<double, 6, 4, Eigen::RowMajor>> J(
              jacobians[i]);
          J.setZero();
        } else {
          Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>> J(
              jacobians[i]);
          J.setZero();
        }
      }
    }

    for (int i = 0; i < SplineOrder; ++i) {
      const size_t idx_r = J_omega.start_idx + i;
      if (jacobians[idx_r]) {
        Eigen::Map<Eigen::Matrix<double, 6, 4, Eigen::RowMajor>> J(
            jacobians[idx_r]);
        J.setZero();
        J.block<3, 3>(3, 0) = sqrt_alpha_R_dt_ * J_omega.d_val_d_knot[i];
      }
      const size_t idx_p = knot_num + J_acc.start_idx + i;
      if (jacobians[idx_p]) {
        Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>> J(
            jacobians[idx_p]);
        J.setZero();
        J.block<3, 3>(0, 0) =
            sqrt_alpha_p_dt_ * J_acc.d_val_d_knot[i] * Eigen::Matrix3d::Identity();
      }
    }

    return true;
  }

 private:
  int64_t t_ns_;
  double sqrt_alpha_p_dt_;
  double sqrt_alpha_R_dt_;
  SplineSegmentMeta<SplineOrder> spline_meta_;
};

}  // namespace analytic_derivative
}  // namespace clic_calib
