/*
 * AR(1) Prais–Winsten RTK whitening: r̃₁ = √(1−ρ²)·r₁/σ, r̃ₜ = (rₜ−ρ·rₜ₋₁)/σ.
 * Couples consecutive RTK innovations (decorr constant drift).
 */

#pragma once

#include <clic_calib/factor/rd_spline_view.h>
#include <clic_calib/factor/so3_spline_view.h>
#include <clic_calib/spline/spline_segment.h>
#include <clic_calib/utils/sophus_utils.hpp>

#include <ceres/ceres.h>
#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace clic_calib {
namespace analytic_derivative {

/** First RTK sample: √(1−ρ²) scaling on whitened position residual. */
class RTKPositionPraisWinstenFirstFactor : public ceres::CostFunction,
                                           public So3SplineView,
                                           public RdSplineView {
 public:
  using Vec3 = Eigen::Vector3d;
  using Mat3 = Eigen::Matrix3d;

  RTKPositionPraisWinstenFirstFactor(int64_t t_ns, const Vec3& p_A_W_observed,
                                     const Eigen::Matrix3d& covariance,
                                     const Vec3& L_B_to_A, double rho,
                                     const SplineSegmentMeta<SplineOrder>& meta)
      : t_ns_(t_ns),
        p_obs_(p_A_W_observed),
        L_B_to_A_(L_B_to_A),
        rho_scale_(std::sqrt(std::max(0.0, 1.0 - rho * rho))),
        spline_meta_(meta) {
    set_num_residuals(3);
    for (int i = 0; i < SplineOrder; ++i) {
      mutable_parameter_block_sizes()->push_back(4);
    }
    for (int i = 0; i < SplineOrder; ++i) {
      mutable_parameter_block_sizes()->push_back(3);
    }
    sqrt_info_ = ComputeSqrtInfo(covariance);
  }

  bool Evaluate(double const* const* parameters, double* residuals,
                double** jacobians) const override {
    typename So3SplineView::JacobianStruct J_R;
    typename RdSplineView::JacobianStruct J_p;

    static constexpr size_t knot_num = SplineOrder;
    const size_t p_offset = knot_num;

    SO3d R_WB;
    Vec3 p_WB;
    if (jacobians) {
      R_WB = So3SplineView::EvaluateRp(t_ns_, spline_meta_, parameters, &J_R);
      p_WB = RdSplineView::evaluate(t_ns_, spline_meta_, parameters + p_offset,
                                    &J_p);
    } else {
      R_WB = So3SplineView::EvaluateRp(t_ns_, spline_meta_, parameters);
      p_WB = RdSplineView::evaluate(t_ns_, spline_meta_, parameters + p_offset);
    }

    const Vec3 r = p_obs_ - (p_WB + R_WB * L_B_to_A_);
    Eigen::Map<Vec3> residual(residuals);
    residual = rho_scale_ * sqrt_info_ * r;

    if (!jacobians) {
      return true;
    }

    for (size_t i = 0; i < knot_num; ++i) {
      if (jacobians[i]) {
        Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>>(jacobians[i])
            .setZero();
      }
      if (jacobians[knot_num + i]) {
        Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
            jacobians[knot_num + i])
            .setZero();
      }
    }

    const Mat3 R_hat_L = R_WB.matrix() * SO3d::hat(L_B_to_A_);
    const Mat3 scale = rho_scale_ * sqrt_info_;

    for (int i = 0; i < SplineOrder; ++i) {
      const size_t idx_r = J_R.start_idx + i;
      if (jacobians[idx_r]) {
        Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> J_knot(
            jacobians[idx_r]);
        J_knot.block<3, 3>(0, 0) = scale * (R_hat_L * J_R.d_val_d_knot[i]);
      }
      const size_t idx_p = knot_num + J_p.start_idx + i;
      if (jacobians[idx_p]) {
        Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> J_knot(
            jacobians[idx_p]);
        J_knot.block<3, 3>(0, 0) = scale * (-J_p.d_val_d_knot[i]);
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
  double rho_scale_ = 1.0;
  SplineSegmentMeta<SplineOrder> spline_meta_;
  WhiteningMatrix sqrt_info_;
};

/** Deduplicated knot union for Ceres (no duplicate parameter blocks). */
struct RtkSplineKnotUnion {
  std::vector<double*> rot_blocks;
  std::vector<double*> pos_blocks;
  std::array<int, SplineOrder> prev_rot_idx{};
  std::array<int, SplineOrder> prev_pos_idx{};
  std::array<int, SplineOrder> curr_rot_idx{};
  std::array<int, SplineOrder> curr_pos_idx{};
};

inline RtkSplineKnotUnion BuildRtkSplineKnotUnion(
    const std::array<double*, SplineOrder>& rot_prev,
    const std::array<double*, SplineOrder>& pos_prev,
    const std::array<double*, SplineOrder>& rot_curr,
    const std::array<double*, SplineOrder>& pos_curr) {
  RtkSplineKnotUnion u;
  auto append = [](double* p, std::vector<double*>* blocks) -> int {
    const auto it = std::find(blocks->begin(), blocks->end(), p);
    if (it == blocks->end()) {
      blocks->push_back(p);
      return static_cast<int>(blocks->size() - 1);
    }
    return static_cast<int>(it - blocks->begin());
  };
  for (int i = 0; i < SplineOrder; ++i) {
    u.prev_rot_idx[i] = append(rot_prev[i], &u.rot_blocks);
  }
  for (int i = 0; i < SplineOrder; ++i) {
    u.prev_pos_idx[i] = append(pos_prev[i], &u.pos_blocks);
  }
  for (int i = 0; i < SplineOrder; ++i) {
    u.curr_rot_idx[i] = append(rot_curr[i], &u.rot_blocks);
  }
  for (int i = 0; i < SplineOrder; ++i) {
    u.curr_pos_idx[i] = append(pos_curr[i], &u.pos_blocks);
  }
  return u;
}

/** Innovation r̃ₜ = (rₜ − ρ·rₜ₋₁)/σ with deduplicated spline knots. */
class RTKPositionPraisWinstenInnovationFactor : public ceres::CostFunction,
                                                public So3SplineView,
                                                public RdSplineView {
 public:
  using Vec3 = Eigen::Vector3d;
  using Mat3 = Eigen::Matrix3d;

  RTKPositionPraisWinstenInnovationFactor(
      int64_t t_prev_ns, int64_t t_curr_ns, const Vec3& p_obs_prev,
      const Vec3& p_obs_curr, const Eigen::Matrix3d& covariance,
      const Vec3& L_B_to_A, double rho,
      const SplineSegmentMeta<SplineOrder>& meta,
      const RtkSplineKnotUnion& knot_union)
      : t_prev_ns_(t_prev_ns),
        t_curr_ns_(t_curr_ns),
        p_obs_prev_(p_obs_prev),
        p_obs_curr_(p_obs_curr),
        L_B_to_A_(L_B_to_A),
        rho_(rho),
        inv_rho_scale_(1.0 / std::sqrt(std::max(1e-12, 1.0 - rho * rho))),
        spline_meta_(meta),
        union_(knot_union) {
    set_num_residuals(3);
    for (size_t i = 0; i < union_.rot_blocks.size(); ++i) {
      (void)i;
      mutable_parameter_block_sizes()->push_back(4);
    }
    for (size_t i = 0; i < union_.pos_blocks.size(); ++i) {
      (void)i;
      mutable_parameter_block_sizes()->push_back(3);
    }
    sqrt_info_ = ComputeSqrtInfo(covariance);
  }

  bool Evaluate(double const* const* parameters, double* residuals,
                double** jacobians) const override {
    const size_t n_rot = union_.rot_blocks.size();
    const size_t n_pos = union_.pos_blocks.size();

    double const* prev_rot[SplineOrder];
    double const* prev_pos[SplineOrder];
    double const* curr_rot[SplineOrder];
    double const* curr_pos[SplineOrder];
    for (int i = 0; i < SplineOrder; ++i) {
      prev_rot[i] = parameters[union_.prev_rot_idx[i]];
      prev_pos[i] = parameters[n_rot + union_.prev_pos_idx[i]];
      curr_rot[i] = parameters[union_.curr_rot_idx[i]];
      curr_pos[i] = parameters[n_rot + union_.curr_pos_idx[i]];
    }

    typename So3SplineView::JacobianStruct J_R_prev, J_R_curr;
    typename RdSplineView::JacobianStruct J_p_prev, J_p_curr;

    SO3d R_prev, R_curr;
    Vec3 p_prev, p_curr;

    if (jacobians) {
      R_prev = So3SplineView::EvaluateRp(t_prev_ns_, spline_meta_, prev_rot,
                                         &J_R_prev);
      p_prev = RdSplineView::evaluate(t_prev_ns_, spline_meta_, prev_pos,
                                      &J_p_prev);
      R_curr = So3SplineView::EvaluateRp(t_curr_ns_, spline_meta_, curr_rot,
                                         &J_R_curr);
      p_curr = RdSplineView::evaluate(t_curr_ns_, spline_meta_, curr_pos,
                                      &J_p_curr);
    } else {
      R_prev = So3SplineView::EvaluateRp(t_prev_ns_, spline_meta_, prev_rot);
      p_prev = RdSplineView::evaluate(t_prev_ns_, spline_meta_, prev_pos);
      R_curr = So3SplineView::EvaluateRp(t_curr_ns_, spline_meta_, curr_rot);
      p_curr = RdSplineView::evaluate(t_curr_ns_, spline_meta_, curr_pos);
    }

    const Vec3 r_prev = p_obs_prev_ - (p_prev + R_prev * L_B_to_A_);
    const Vec3 r_curr = p_obs_curr_ - (p_curr + R_curr * L_B_to_A_);
    const Vec3 innov = r_curr - rho_ * r_prev;
    Eigen::Map<Vec3> residual(residuals);
    residual = inv_rho_scale_ * sqrt_info_ * innov;

    if (!jacobians) {
      return true;
    }

    for (size_t b = 0; b < n_rot + n_pos; ++b) {
      if (!jacobians[b]) {
        continue;
      }
      if (b < n_rot) {
        Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>>(jacobians[b])
            .setZero();
      } else {
        Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(jacobians[b])
            .setZero();
      }
    }

    const Mat3 R_hat_prev = R_prev.matrix() * SO3d::hat(L_B_to_A_);
    const Mat3 R_hat_curr = R_curr.matrix() * SO3d::hat(L_B_to_A_);
    const Mat3 scale_prev = -rho_ * inv_rho_scale_ * sqrt_info_;
    const Mat3 scale_curr = inv_rho_scale_ * sqrt_info_;

    for (int i = 0; i < SplineOrder; ++i) {
      if (jacobians[union_.prev_rot_idx[i]]) {
        Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> J_knot(
            jacobians[union_.prev_rot_idx[i]]);
        J_knot.block<3, 3>(0, 0) +=
            scale_prev * (R_hat_prev * J_R_prev.d_val_d_knot[i]);
      }
      if (jacobians[n_rot + union_.prev_pos_idx[i]]) {
        Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> J_knot(
            jacobians[n_rot + union_.prev_pos_idx[i]]);
        J_knot.block<3, 3>(0, 0) += scale_prev * (-J_p_prev.d_val_d_knot[i]);
      }
      if (jacobians[union_.curr_rot_idx[i]]) {
        Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> J_knot(
            jacobians[union_.curr_rot_idx[i]]);
        J_knot.block<3, 3>(0, 0) +=
            scale_curr * (R_hat_curr * J_R_curr.d_val_d_knot[i]);
      }
      if (jacobians[n_rot + union_.curr_pos_idx[i]]) {
        Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> J_knot(
            jacobians[n_rot + union_.curr_pos_idx[i]]);
        J_knot.block<3, 3>(0, 0) += scale_curr * (-J_p_curr.d_val_d_knot[i]);
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

  int64_t t_prev_ns_;
  int64_t t_curr_ns_;
  Vec3 p_obs_prev_;
  Vec3 p_obs_curr_;
  Vec3 L_B_to_A_;
  double rho_ = 0.0;
  double inv_rho_scale_ = 1.0;
  SplineSegmentMeta<SplineOrder> spline_meta_;
  RtkSplineKnotUnion union_;
  WhiteningMatrix sqrt_info_;
};

}  // namespace analytic_derivative
}  // namespace clic_calib
