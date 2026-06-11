#pragma once

#include <clic_calib/estimator/body_gated_calibration.h>
#include <clic_calib/estimator/observed_mean_gate.h>
#include <clic_calib/utils/lever_arm.h>
#include <clic_calib/utils/noise_model.h>
#include <clic_calib/utils/sophus_utils.hpp>

#include <Eigen/Core>

#include <map>
#include <string>
#include <vector>

namespace clic_calib {

/** Per-LiDAR body-cluster stream in a shared Stage-1 mission. */
struct LidarBodyCalibrationSpec {
  int sensor_id = 0;
  std::string sensor_key;
  std::vector<BodyClusterObservation> body_obs;
  Eigen::Vector3d lidar_post_W = Eigen::Vector3d::Zero();
  const SE3d* gt_T_LW = nullptr;
};

struct PerSensorBodyCalibResult {
  GatedBodyCalibrationResult calib;
  double u_B_azimuth_std_deg = -1.0;
};

struct MultiLidarBodyCalibrationResult {
  std::map<int, PerSensorBodyCalibResult> per_sensor;
  SE3d T_rel_observed_mean = SE3d();
  SE3d T_rel_centroid_only = SE3d();
  double rel_rot_deg = 0.0;
  double rel_trans_mm = 0.0;
  const SE3d* gt_T_rel = nullptr;
};

/**
 * Stage-2 dual LiDAR = independent gated body calibrations per sensor;
 * relative extrinsic composed post hoc (no shared Ceres blocks).
 */
MultiLidarBodyCalibrationResult CalibrateMultiLidarBodyGated(
    const BodyGatedCalibrationInput& shared_mission,
    const std::vector<LidarBodyCalibrationSpec>& sensors,
    const LeverArmConfig& levers, const NoiseModel& noise,
    const TwoStagePipelineConfig& base_cfg, int observed_mean_iters = 3,
    const ObservedMeanAspectGate& gate = {},
    const std::vector<double>* temporal_sqrt_info_scales = nullptr);

}  // namespace clic_calib
