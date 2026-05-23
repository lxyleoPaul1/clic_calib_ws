#pragma once

#include <clic_calib/estimator/attitude_stream_config.h>
#include <clic_calib/estimator/calibration_estimator.h>
#include <clic_calib/utils/noise_model.h>

#include <iosfwd>
#include <string>
#include <vector>

namespace clic_calib {

/**
 * File paths for a real-data calibration session (no rosbag parsing here).
 *
 * Pipeline:
 *   rosbag → preprocess_rosbag → observations.clicob (LiDAR sphere + AprilTag)
 *   RTK CSV → RTKReader
 *   PSDK attitude CSV → AttitudeReader (50 Hz, RTK clock, Σ_att from config)
 */
struct RealDataPaths {
  std::string config_dir;
  std::string rtk_csv;
  /** PSDK fused attitude log; empty → RTK-only fallback (not for field calib). */
  std::string attitude_csv;
  /** Preprocessed target archive from preprocess_rosbag. */
  std::string observations_clicob;
};

struct RealDataReadinessReport {
  bool config_dir_ok = false;
  bool lever_arms_loaded = false;
  bool noise_model_loaded = false;
  bool attitude_sigma_configured = false;
  bool attitude_stream_config_loaded = false;
  bool target_geometry_loaded = false;
  bool sensor_rig_loaded = false;

  AttitudeStreamConfig attitude_stream;
  NoiseModel noise_model;

  /** Human-readable interface checklist for DECISION GATE 5. */
  std::vector<std::string> stream_interfaces;
  std::vector<std::string> warnings;
};

/**
 * Wires real-data streams into @ref CalibrationEstimator (interfaces only).
 * Does not run rosbag extraction or field QA.
 */
class RealDataSession {
 public:
  static RealDataReadinessReport CheckReadiness(const std::string& config_dir);

  static void PrintReadinessReport(std::ostream& os,
                                   const RealDataReadinessReport& report);

  /** Load paths and call estimator add_* methods. Attitude optional. */
  static void WireInto(CalibrationEstimator* estimator,
                       const RealDataPaths& paths);
};

}  // namespace clic_calib
