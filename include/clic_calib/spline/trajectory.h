/*
 * clic_calib — UAV body trajectory T_WB(t) as split SE(3) B-spline.
 * Preserved from APRIL-ZJU/clic spline stack (Basalt-style SO(3)+R^3).
 * See doc/DERIVATIONS.md and RECON_REPORT.md.
 */

#pragma once

#include <clic_calib/spline/se3_spline.h>
#include <clic_calib/utils/sophus_utils.hpp>

#include <memory>

namespace clic_calib {

/**
 * @brief Continuous-time pose of UAV body frame B in world frame W.
 *
 * Naming: T_WB maps points from B to W (p_W = T_WB * p_B).
 * Knot spacing is uniform with interval @p knot_interval_s (seconds).
 */
class BodyTrajectory : public Se3Spline<SplineOrder, double> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<BodyTrajectory>;

  static constexpr double kNsToS = 1e-9;
  static constexpr double kSToNs = 1e9;

  BodyTrajectory(double knot_interval_s, double start_time_s = 0.0);

  /** @brief SE(3) pose T_WB at time t (seconds, trajectory time base). */
  SE3d pose_wb(double t_s) const;

  /** @brief Translation p_WB(t) — body origin in W. */
  Eigen::Vector3d position_wb(double t_s) const;

  /** @brief Rotation R_WB(t). */
  SO3d rotation_wb(double t_s) const;

  /** @brief §4.1: p_A^W(t) = p_WB(t) + R_WB(t) * L_{B→A}. */
  Eigen::Vector3d antenna_position_w(double t_s,
                                     const Eigen::Vector3d& lever_b_to_a) const;

  /** @brief §4.1: p_G^W(t) = p_WB(t) + R_WB(t) * L_{B→G}. */
  Eigen::Vector3d sphere_center_w(double t_s,
                                  const Eigen::Vector3d& lever_b_to_g) const;

  /** @brief Board-free: p_body^W(t) = p_WB(t) + R_WB(t) * L_{B→body}. */
  Eigen::Vector3d body_centroid_w(double t_s,
                                  const Eigen::Vector3d& lever_b_to_body) const;

  /** @brief §4.1: p_{M_j}^W(t) = p_G^W(t) + R_WB(t) * L_{G→M_j}. */
  Eigen::Vector3d marker_center_w(double t_s,
                                  const Eigen::Vector3d& lever_b_to_g,
                                  const Eigen::Vector3d& lever_g_to_mj) const;
};

/** @brief Alias used by CalibrationEstimator (UAV anchor T_WB). */
using Trajectory = BodyTrajectory;

}  // namespace clic_calib
