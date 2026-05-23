#include <clic_calib/estimator/real_data_session.h>

#include <clic_calib/io/attitude_reader.h>
#include <clic_calib/io/observation_archive.h>
#include <clic_calib/io/rtk_reader.h>
#include <clic_calib/utils/lever_arm.h>

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <iostream>

namespace clic_calib {
namespace {

bool FileExists(const std::string& path) {
  std::ifstream f(path);
  return f.good();
}

}  // namespace

RealDataReadinessReport RealDataSession::CheckReadiness(
    const std::string& config_dir) {
  RealDataReadinessReport report;
  std::string prefix = config_dir;
  if (!prefix.empty() && prefix.back() == '/') {
    prefix.pop_back();
  }

  report.config_dir_ok = FileExists(prefix + "/lever_arms.yaml") &&
                         FileExists(prefix + "/noise_model.yaml") &&
                         FileExists(prefix + "/spline.yaml") &&
                         FileExists(prefix + "/target_geometry.yaml") &&
                         FileExists(prefix + "/sensor_rig.yaml");

  try {
    const LeverArmConfig levers =
        LeverArmConfig::from_yaml(prefix + "/lever_arms.yaml");
    report.lever_arms_loaded = !levers.L_B_to_A.isZero(0) && !levers.L_B_to_G.isZero(0);
  } catch (...) {
    report.warnings.push_back("lever_arms.yaml failed to parse");
  }

  try {
    report.noise_model = NoiseModel::FromConfigDir(prefix);
    report.noise_model_loaded = true;
    report.attitude_sigma_configured =
        report.noise_model.attitude_sigma_roll_deg > 0.0 &&
        report.noise_model.attitude_sigma_pitch_deg > 0.0 &&
        report.noise_model.attitude_sigma_yaw_deg > 0.0;
  } catch (...) {
    report.warnings.push_back("noise_model.yaml failed to parse");
  }

  try {
    report.attitude_stream = AttitudeStreamConfig::FromConfigDir(prefix);
    report.attitude_stream_config_loaded = report.attitude_stream.stride >= 1;
  } catch (...) {
    report.warnings.push_back("spline.yaml attitude block failed to parse");
  }

  try {
    const YAML::Node tg = YAML::LoadFile(prefix + "/target_geometry.yaml");
    report.target_geometry_loaded =
        tg["sphere_radius_m"] && tg["apriltag_corners_in_marker_frame"];
  } catch (...) {
    report.warnings.push_back("target_geometry.yaml failed to parse");
  }

  try {
    const YAML::Node rig = YAML::LoadFile(prefix + "/sensor_rig.yaml");
    report.sensor_rig_loaded = rig["lidars"] || rig["cameras"];
  } catch (...) {
    report.warnings.push_back("sensor_rig.yaml failed to parse");
  }

  report.stream_interfaces = {
      "RTK position → RTKMeasurement via CSVReader / DJIDatLogReader",
      "PSDK 50 Hz attitude → AttitudeObservation via AttitudeReader "
      "(Σ_att from noise_model.yaml)",
      "LiDAR scans → LiDARTargetObservation via preprocess_rosbag + "
      "SphereExtractor",
      "Camera frames → AprilTagObservation via preprocess_rosbag + "
      "AprilTagDetectorWrapper",
      "Lever arms L_B_to_A, L_B_to_G, L_G_to_M → lever_arms.yaml",
      "Sphere radius + tag corners → target_geometry.yaml",
      "Stage-1 attitude stride + transport delay → spline.yaml attitude:",
  };

  if (report.noise_model.attitude_sigma_roll_deg == 0.2 &&
      report.noise_model.attitude_sigma_pitch_deg == 0.2 &&
      report.noise_model.attitude_sigma_yaw_deg == 1.5) {
    report.warnings.push_back(
        "Σ_att uses synthetic defaults (0.2°/0.2°/1.5°) — replace with "
        "hover-measured STD before field calibration");
  }

  return report;
}

void RealDataSession::PrintReadinessReport(
    std::ostream& os, const RealDataReadinessReport& report) {
  os << "\n=== Real-data interface readiness ===\n";
  os << "  config_dir:           " << (report.config_dir_ok ? "OK" : "MISSING")
     << "\n";
  os << "  lever_arms:           "
     << (report.lever_arms_loaded ? "OK" : "FAIL") << "\n";
  os << "  noise_model (Σ_att):  "
     << (report.noise_model_loaded ? "OK" : "FAIL") << "\n";
  os << "  attitude_stream cfg:  "
     << (report.attitude_stream_config_loaded ? "OK" : "FAIL") << "\n";
  os << "  target_geometry:      "
     << (report.target_geometry_loaded ? "OK" : "FAIL") << "\n";
  os << "  sensor_rig:           "
     << (report.sensor_rig_loaded ? "OK" : "FAIL") << "\n";

  if (report.noise_model_loaded) {
    os << "  Σ_att (deg): roll=" << report.noise_model.attitude_sigma_roll_deg
       << " pitch=" << report.noise_model.attitude_sigma_pitch_deg
       << " yaw=" << report.noise_model.attitude_sigma_yaw_deg << "\n";
  }
  report.attitude_stream.Log(os);

  os << "\n  Stream interfaces:\n";
  for (const std::string& line : report.stream_interfaces) {
    os << "    • " << line << "\n";
  }
  if (!report.warnings.empty()) {
    os << "\n  Warnings:\n";
    for (const std::string& w : report.warnings) {
      os << "    ⚠ " << w << "\n";
    }
  }
}

void RealDataSession::WireInto(CalibrationEstimator* estimator,
                               const RealDataPaths& paths) {
  if (!estimator) {
    throw std::runtime_error("RealDataSession::WireInto: null estimator");
  }

  CSVReader rtk_reader;
  const std::vector<RTKMeasurement> rtk = rtk_reader.read(paths.rtk_csv);
  if (rtk.empty()) {
    throw std::runtime_error("RealDataSession: no RTK measurements in " +
                             paths.rtk_csv);
  }
  estimator->add_rtk_measurements(rtk);

  if (!paths.attitude_csv.empty()) {
    const RealDataReadinessReport readiness =
        CheckReadiness(paths.config_dir);
    AttitudeReader::Options att_opts;
    att_opts.transport_delay_s = readiness.attitude_stream.transport_delay_s;
    att_opts.noise_model = readiness.noise_model;
    const AttitudeReader att_reader(att_opts);
    const std::vector<AttitudeObservation> attitude =
        att_reader.read(paths.attitude_csv);
    if (attitude.empty()) {
      throw std::runtime_error("RealDataSession: empty attitude stream in " +
                               paths.attitude_csv);
    }
    estimator->add_attitude_observations(attitude);
  }

  ObservationArchive::LidarBySensor lidar;
  ObservationArchive::AprilTagBySensor apriltag;
  if (!ObservationArchive::Read(paths.observations_clicob, &lidar, &apriltag)) {
    throw std::runtime_error("RealDataSession: failed to read " +
                             paths.observations_clicob);
  }
  for (const auto& kv : lidar) {
    estimator->add_lidar_target_observations(kv.first, kv.second);
  }
  for (const auto& kv : apriltag) {
    estimator->add_apriltag_observations(kv.first, kv.second);
  }
}

}  // namespace clic_calib
