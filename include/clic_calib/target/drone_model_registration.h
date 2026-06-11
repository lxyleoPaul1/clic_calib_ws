#pragma once

#include <clic_calib/config/body_model_config.h>
#include <clic_calib/sensor_data/body_cluster_observation.h>
#include <clic_calib/spline/trajectory.h>
#include <clic_calib/utils/sophus_utils.hpp>

#include <Eigen/Core>

#include <vector>

namespace clic_calib {

struct DroneModelRegistrationResult {
  Eigen::Vector3d centroid_L = Eigen::Vector3d::Zero();
  Eigen::Vector3d fitted_center_B = Eigen::Vector3d::Zero();
  double surface_rmse_m = 0.0;
  bool ok = false;
};

/**
 * @brief Register visible cluster points to a fixed axis-aligned box in B.
 *
 * Uses RTK-frozen T_WB(t) and nominal T_LW to transform points into body frame,
 * then optimizes box center (3 DOF) with fixed half-extents. Yaw from T_WB
 * fixes box orientation (axis-aligned in B, no 180° flip).
 */
class DroneModelRegistration {
 public:
  static DroneModelRegistrationResult Register(
      const std::vector<Eigen::Vector3d>& points_L, const SE3d& T_LW,
      const SE3d& T_WB, const BodyModelConfig& model);

  /**
   * @brief Register using only model surface points on faces visible from LiDAR.
   * Avoids fitting against hidden/back faces (Phase 1.5 audit).
   */
  static DroneModelRegistrationResult RegisterVisibilityAware(
      const std::vector<Eigen::Vector3d>& points_L, const SE3d& T_LW,
      const SE3d& T_WB, const BodyModelConfig& model);

  /** Replace centroid_L_ with model-registered geometric center per frame. */
  static std::vector<BodyClusterObservation> ApplyVisibilityAwareToObservations(
      const std::vector<BodyClusterObservation>& observations,
      const BodyTrajectory& traj, const SE3d& T_LW, double t_d_L_s,
      const BodyModelConfig& model);

  /** Replace centroid_L_ with model-registered geometric center per frame. */
  static std::vector<BodyClusterObservation> ApplyToObservations(
      const std::vector<BodyClusterObservation>& observations,
      const BodyTrajectory& traj, const SE3d& T_LW, double t_d_L_s,
      const BodyModelConfig& model);
};

}  // namespace clic_calib
