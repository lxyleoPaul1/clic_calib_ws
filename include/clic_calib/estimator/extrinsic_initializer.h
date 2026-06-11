#pragma once

#include <clic_calib/estimator/two_stage_types.h>
#include <clic_calib/sensor_data/apriltag_observation.h>
#include <clic_calib/sensor_data/body_cluster_observation.h>
#include <clic_calib/sensor_data/lidar_target_observation.h>
#include <clic_calib/spline/trajectory.h>
#include <clic_calib/utils/camera_projection.h>
#include <clic_calib/utils/lever_arm.h>

#include <vector>

namespace clic_calib {

struct ExtrinsicInitializerConfig {
  LidarTargetMode lidar_target_mode = LidarTargetMode::kSphere;
  double sphere_radius_m = 0.10;
  /** Nominal yaml t_d used for world-time pairing at init (not refined t_d). */
  double nominal_t_d_L_s = 0.0;
  double nominal_t_d_C_s = 0.0;
  PinholeIntrinsics camera_K{600.0, 600.0, 320.0, 240.0};
  RadtanDistortion camera_dist;
  int marker_id = 0;
};

/**
 * Closed-form Stage-2 init: Umeyama (LiDAR) + tag-local IPPE PnP (camera).
 *
 * Inputs use the same observation schema as preprocess / synthetic generators:
 *   LiDARTargetObservation — from SphereExtractor (real) or simulator (synthetic)
 *   AprilTagObservation    — from AprilTagDetectorWrapper or simulator
 * Output init sets t_d^L = t_d^C = 0 (refined in Stage-2).
 */
class ExtrinsicInitializer {
 public:
  static GeometricInitReport FromGeometric(
      const BodyTrajectory& traj,
      const std::vector<LiDARTargetObservation>& lidar,
      const std::vector<BodyClusterObservation>& body_cluster,
      const std::vector<AprilTagObservation>& tags,
      const LeverArmConfig& levers, const ExtrinsicInitializerConfig& cfg,
      const std::vector<Eigen::Vector3d>* L_B_per_obs = nullptr);

  /** Board-free LiDAR init from body centroids only; camera pose held at @p T_CW_seed. */
  static GeometricInitReport FromBodyCentroidsOnly(
      const BodyTrajectory& traj,
      const std::vector<BodyClusterObservation>& body_cluster,
      const LeverArmConfig& levers, const ExtrinsicInitializerConfig& cfg,
      const SE3d& T_CW_seed);
};

}  // namespace clic_calib
