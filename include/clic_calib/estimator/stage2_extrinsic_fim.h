#pragma once

#include <clic_calib/estimator/two_stage_types.h>
#include <clic_calib/sensor_data/apriltag_observation.h>
#include <clic_calib/sensor_data/lidar_target_observation.h>
#include <clic_calib/spline/trajectory.h>
#include <clic_calib/utils/camera_projection.h>
#include <clic_calib/utils/lever_arm.h>
#include <clic_calib/utils/noise_model.h>

#include <ceres/ceres.h>
#include <Eigen/Core>

namespace clic_calib {

/** Symmetric information matrix eigen summary (Stage-2 extrinsic block). */
struct SymmetricInformationReport {
  double lambda_min = 0.0;
  double lambda_max = 0.0;
  double cond = std::numeric_limits<double>::infinity();
  int rank = 0;
  int dim = 0;
  bool is_pd = false;
};

/** F = JᵀJ from Ceres CRS Jacobian (loss disabled). */
Eigen::MatrixXd InformationFromCeresJacobian(const ceres::CRSMatrix& crs);

SymmetricInformationReport AnalyzeInformationMatrix(const Eigen::MatrixXd& F);

/**
 * Prior-free Stage-2 extrinsic information @ linearization point.
 * Column layout (14): t_d_L, LW_rot×3, LW_t×3, t_d_C, CW_rot×3, CW_t×3.
 */
Eigen::MatrixXd BuildStage2ExtrinsicInformation(
    const BodyTrajectory& fixed_traj,
    const std::vector<LiDARTargetObservation>& lidar,
    const std::vector<AprilTagObservation>& tags,
    const ExtrinsicOptimizeState& lidar_state,
    const ExtrinsicOptimizeState& camera_state,
    const LeverArmConfig& levers, const NoiseModel& noise,
    double sphere_radius_m, const PinholeIntrinsics& K,
    const RadtanDistortion& dist, int marker_id = 0);

}  // namespace clic_calib
