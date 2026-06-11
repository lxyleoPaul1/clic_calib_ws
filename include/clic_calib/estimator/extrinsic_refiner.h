#pragma once

#include <clic_calib/estimator/two_stage_types.h>
#include <clic_calib/sensor_data/apriltag_observation.h>
#include <clic_calib/sensor_data/body_cluster_observation.h>
#include <clic_calib/sensor_data/lidar_target_observation.h>
#include <clic_calib/spline/trajectory.h>
#include <clic_calib/utils/camera_projection.h>
#include <clic_calib/utils/lever_arm.h>
#include <clic_calib/utils/noise_model.h>

#include <ceres/ceres.h>

#include <vector>

namespace clic_calib {

struct ExtrinsicRefinerConfig {
  LidarTargetMode lidar_target_mode = LidarTargetMode::kSphere;
  BodyLeverArmMode body_lever_arm_mode = BodyLeverArmMode::kNominalYaml;
  double sphere_radius_m = 0.10;
  double t_d_max_abs_s = 0.1;
  double lidar_cauchy_scale = 1.0;
  double camera_huber_delta_px = 2.0;
  PinholeIntrinsics camera_K{600.0, 600.0, 320.0, 240.0};
  RadtanDistortion camera_dist;
  int marker_id = 0;
  int max_iterations = 500;
};

struct ExtrinsicRefinerResult {
  ExtrinsicOptimizeState lidar;
  ExtrinsicOptimizeState camera;
  Eigen::Vector3d optimized_L_B_to_body = Eigen::Vector3d::Zero();
  bool has_optimized_L_B = false;
  ceres::Solver::Summary summary;
  bool converged = false;
};

/** Stage-2 problem wiring audit (joint p_B debugging). */
struct ExtrinsicRefinerProblemAudit {
  int body_centroid_fixed_blocks = 0;
  int body_centroid_joint_blocks = 0;
  int sphere_blocks = 0;
  int apriltag_blocks = 0;
  int total_residual_blocks = 0;
  bool joint_path_active = false;
  bool L_B_block_present = false;
  bool L_B_block_constant = false;
  double cost_at_init = 0.0;
  double L_B_gradient_norm_at_init = 0.0;
  double L_B_numeric_cost_drop_1mm_x = 0.0;
};

/**
 * Stage-2 prior-free extrinsic refinement on a fixed trajectory.
 * Port of probe SolveStage2Extrinsics().
 */
class ExtrinsicRefiner {
 public:
  static ExtrinsicRefinerResult Refine(
      const BodyTrajectory& fixed_traj,
      const std::vector<LiDARTargetObservation>& lidar,
      const std::vector<BodyClusterObservation>& body_cluster,
      const std::vector<AprilTagObservation>& tags,
      const LeverArmConfig& levers, const NoiseModel& noise,
      const CoarseExtrinsicInit& init, const ExtrinsicRefinerConfig& cfg,
      const std::vector<Eigen::Vector3d>* L_B_per_obs = nullptr);

  /**
   * Pass-1 joint p_B: optimize L_B with T_LW/t_d fixed at @p init (body factors
   * only). Residual is linear in L_B so this is well-conditioned.
   */
  static Eigen::Vector3d OptimizeJointBodyLeverAtFixedExtrinsic(
      const BodyTrajectory& fixed_traj,
      const std::vector<BodyClusterObservation>& body_cluster,
      const CoarseExtrinsicInit& init, const LeverArmConfig& levers,
      const NoiseModel& noise, const ExtrinsicRefinerConfig& cfg);

  /** Build the same Ceres problem as Refine (pre-solve) and report wiring. */
  static ExtrinsicRefinerProblemAudit AuditProblem(
      const BodyTrajectory& fixed_traj,
      const std::vector<LiDARTargetObservation>& lidar,
      const std::vector<BodyClusterObservation>& body_cluster,
      const std::vector<AprilTagObservation>& tags,
      const LeverArmConfig& levers, const NoiseModel& noise,
      const CoarseExtrinsicInit& init, const ExtrinsicRefinerConfig& cfg);
};

}  // namespace clic_calib
