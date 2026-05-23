#pragma once

#include <clic_calib/utils/sophus_utils.hpp>

#include <Eigen/Core>

namespace clic_calib {

/**
 * @brief Fused body attitude sample (e.g. DJI PSDK flight-controller quaternion).
 *
 * Timestamp shares the RTK/world clock (no relative offset). Covariance is
 * anisotropic in the body tangent: Σ_att = diag(σ_roll², σ_pitch², σ_yaw²).
 */
struct AttitudeObservation {
  double t_world_ = 0.0;
  SO3d R_WB_observed_ = SO3d(Eigen::Quaterniond::Identity());
  /** 3×3 tangent-space covariance [rad²], body frame, roll/pitch/yaw diagonal. */
  Eigen::Matrix3d covariance_ = Eigen::Matrix3d::Identity();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace clic_calib
