#pragma once

#include <clic_calib/estimator/extrinsic_initializer.h>
#include <clic_calib/estimator/extrinsic_refiner.h>
#include <clic_calib/estimator/observed_mean_gate.h>
#include <clic_calib/estimator/stage1_trajectory_fitter.h>
#include <clic_calib/estimator/two_stage_pipeline.h>
#include <clic_calib/utils/sophus_utils.hpp>
#include <clic_calib/sensor_data/apriltag_observation.h>
#include <clic_calib/sensor_data/attitude_observation.h>
#include <clic_calib/sensor_data/body_cluster_observation.h>
#include <clic_calib/sensor_data/rtk_measurement.h>
#include <clic_calib/utils/lever_arm.h>
#include <clic_calib/utils/noise_model.h>

#include <limits>
#include <vector>

namespace clic_calib {

/** Shared mission streams for per-LiDAR board-free Stage-2 calibration. */
struct BodyGatedCalibrationInput {
  std::vector<RTKMeasurement> rtk;
  std::vector<AttitudeObservation> attitude;
  std::vector<AprilTagObservation> tags;
  std::vector<BodyClusterObservation> body_obs;
  double nominal_t_d_L_s = 0.0;
  /** Optional: simulation guard compares |trans| vs GT (field: leave null). */
  const SE3d* gt_T_LW = nullptr;
};

struct BodyExtrinsicMetrics {
  double rot_deg = 0.0;
  double trans_mm = 0.0;
  bool stage2_ok = false;
};

struct GatedBodyCalibrationResult {
  BodyExtrinsicMetrics centroid_only;
  BodyExtrinsicMetrics observed_mean;
  SE3d T_LW_centroid;
  SE3d T_LW_observed_mean;
  TrajectoryAttitudeSpread aspect;
  bool observed_mean_applied = false;
  double centroid_refine_cost = 0.0;
  double observed_refine_cost = 0.0;
};

struct IterativeObservedMeanResult {
  BodyExtrinsicMetrics final_metrics;
  SE3d final_T_LW;
  double final_refine_cost = std::numeric_limits<double>::infinity();
  std::vector<double> trans_mm_per_iter;
};

IterativeObservedMeanResult RunBodyPathIterativeObservedMean(
    const BodyGatedCalibrationInput& in, const LeverArmConfig& levers,
    const NoiseModel& noise, const TwoStagePipelineConfig& base_cfg,
    int num_iters,
    const std::vector<double>* temporal_sqrt_info_scales = nullptr);

GatedBodyCalibrationResult CalibrateBodyGatedObservedMean(
    const BodyGatedCalibrationInput& in, const LeverArmConfig& levers,
    const NoiseModel& noise, const TwoStagePipelineConfig& base_cfg,
    int observed_mean_iters = 3, const ObservedMeanAspectGate& gate = {},
    double u_B_azimuth_std_deg = -1.0,
    const std::vector<double>* temporal_sqrt_info_scales = nullptr);

}  // namespace clic_calib
