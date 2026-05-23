#pragma once

#include "experiments/synthetic_flight_geometry.hpp"

#include <clic_calib/estimator/calibration_estimator.h>
#include <clic_calib/estimator/observability_analyzer.h>
#include <clic_calib/sensor_data/attitude_observation.h>
#include <clic_calib/utils/camera_projection.h>
#include <clic_calib/utils/lever_arm.h>
#include <clic_calib/utils/noise_model.h>
#include <clic_calib/utils/sophus_utils.hpp>

#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace clic_calib {
namespace experiments {

/** Loaded from config/noise_model.yaml (shared with CalibrationEstimator). */
using RealisticNoiseSpec = NoiseModel;

struct ExtrinsicGroundTruth {
  SE3d T_LW;
  SE3d T_CW;
  double t_d_L_s = 0.030;
  double t_d_C_s = -0.015;
};

struct SyntheticScenarioBundle {
  BodyTrajectory gt_traj{0.05, 0.0};
  std::vector<RTKMeasurement> rtk;
  std::vector<AttitudeObservation> attitude_obs;
  std::vector<LiDARTargetObservation> lidar_obs;
  std::vector<AprilTagObservation> tag_obs;
  ExtrinsicGroundTruth gt;
};

struct PosteriorMarginals {
  /** Posterior σ from Cov(F_ext^{-1}) diagonal [rad or m]. */
  double pitch_LW_rad = 0.0;
  double roll_LW_rad = 0.0;
  double yaw_LW_rad = 0.0;
  double tx_LW_m = 0.0;
  double ty_LW_m = 0.0;
  double tz_LW_m = 0.0;
};

struct CalibrationRunMetrics {
  double rot_LW_deg = 0.0;
  double rot_CW_deg = 0.0;
  double pitch_LW_deg = 0.0;
  double pitch_LW_err_deg = 0.0;
  Eigen::Vector3d trans_LW_err_m = Eigen::Vector3d::Zero();
  Eigen::Vector3d trans_CW_err_m = Eigen::Vector3d::Zero();
  double trans_LW_norm_mm = 0.0;
  double trans_CW_norm_mm = 0.0;
  double t_d_L_ms = 0.0;
  double t_d_C_ms = 0.0;
  double t_d_L_err_ms = 0.0;
  double t_d_C_err_ms = 0.0;
  double traj_rms_mm = 0.0;
  PosteriorMarginals post;
};

inline std::string ConfigDirFromExperiments() {
  const std::filesystem::path from_source =
      std::filesystem::path(__FILE__).parent_path().parent_path().parent_path() /
      "config";
  if (std::filesystem::exists(from_source / "lever_arms.yaml")) {
    return from_source.string();
  }
  return "config";
}

inline double RotationErrorDeg(const SE3d& T_est, const SE3d& T_gt) {
  const SO3d R_err = T_gt.so3().inverse() * T_est.so3();
  return R_err.log().norm() * 180.0 / M_PI;
}

inline double PitchFromSO3Rad(const SO3d& R) {
  const Eigen::Matrix3d M = R.matrix();
  return std::atan2(-M(2, 0), std::sqrt(M(2, 1) * M(2, 1) + M(2, 2) * M(2, 2)));
}

inline BodyTrajectory MakeLocalGroundTruthTrajectory() {
  BodyTrajectory traj(0.05, 0.0);
  const int num_knots = 24;
  const SE3d k0(SO3d::rotZ(0.0), Eigen::Vector3d::Zero());
  traj.setKnots(k0, num_knots);
  for (int i = 0; i < num_knots; ++i) {
    const double s = static_cast<double>(i) * 0.05;
    const SO3d R = SO3d::rotZ(0.05 * s);
    const Eigen::Vector3d p(0.5 * s, 0.3 * std::sin(s), 2.0 + 0.1 * s);
    traj.setKnot(clic_calib::SE3d(R, p), i);
  }
  const int64_t t_end_ns = static_cast<int64_t>(5.0 * S_TO_NS);
  traj.extendKnotsTo(t_end_ns, traj.getKnotSO3(num_knots - 1),
                     traj.getKnotPos(num_knots - 1));
  return traj;
}

inline BodyTrajectory MakeMultiLayerLocalTrajectory() {
  BodyTrajectory traj(0.05, 0.0);
  const int num_knots = 24;
  const SE3d k0(SO3d::rotZ(0.0), Eigen::Vector3d::Zero());
  traj.setKnots(k0, num_knots);
  for (int i = 0; i < num_knots; ++i) {
    const double s = static_cast<double>(i) * 0.05;
    const SO3d R = SO3d::rotZ(0.05 * s) * SO3d::rotY(0.12 * std::sin(s));
    const Eigen::Vector3d p(0.5 * s, 0.3 * std::sin(s), 2.0 + 0.4 * std::sin(s));
    traj.setKnot(SE3d(R, p), i);
  }
  const int64_t t_end_ns = static_cast<int64_t>(5.0 * S_TO_NS);
  traj.extendKnotsTo(t_end_ns, traj.getKnotSO3(num_knots - 1),
                     traj.getKnotPos(num_knots - 1));
  return traj;
}

inline SyntheticScenarioBundle BuildNoisyScenarioFromGeometry(
    uint32_t seed_traj, uint32_t seed_obs, const RealisticNoiseSpec& noise,
    const SyntheticFlightGeometry& geom) {
  const auto levers =
      LeverArmConfig::from_yaml(ConfigDirFromExperiments() + "/lever_arms.yaml");

  SyntheticScenarioBundle out;
  SensorExtrinsicsFromGeometry(geom, &out.gt.T_LW, &out.gt.T_CW);
  out.gt.t_d_L_s = 0.030;
  out.gt.t_d_C_s = -0.015;
  out.gt_traj = BuildGtTrajectoryFromGeometry(geom);

  const int n_layers =
      std::max(1, static_cast<int>(geom.flight_layers_m.size()));
  const double t_end = geom.use_legacy_local_pose || geom.use_legacy_200m_pose
                           ? 5.0
                           : n_layers * geom.layer_duration_s;

  const double R_ball = 0.10;
  PinholeIntrinsics K{600.0, 600.0, 320.0, 240.0};
  RadtanDistortion dist;
  std::mt19937 rng_traj(seed_traj);
  std::mt19937 rng_obs(seed_obs);

  for (double t = geom.rtk_dt_s; t <= t_end - 1e-9; t += geom.rtk_dt_s) {
    RTKMeasurement m;
    m.t_world_ = t;
    m.fix_status_ = RTKMeasurement::FixStatus::FIXED;
    m.p_A_W_observed_ =
        out.gt_traj.antenna_position_w(t, levers.L_B_to_A) +
        noise.SampleRtkNoise(rng_traj);
    m.covariance_ = noise.RtkPositionCovariance();
    out.rtk.push_back(m);
  }

  for (double t = geom.lidar_dt_s; t <= t_end - 1e-9; t += geom.lidar_dt_s) {
    LiDARTargetObservation scan;
    scan.t_sensor_ = t + out.gt.t_d_L_s;
    scan.sensor_id_ = 0;
    for (int k = 0; k < 24; ++k) {
      const double phi = 2.0 * M_PI * k / 24.0;
      const Eigen::Vector3d p_G_W = out.gt_traj.sphere_center_w(t, levers.L_B_to_G);
      const Eigen::Vector3d p_G_L = out.gt.T_LW * p_G_W;
      Eigen::Vector3d dir(std::cos(phi), std::sin(phi), 0.0);
      if (!geom.multilayer && geom.use_legacy_200m_pose) {
        dir.z() = 0.0;
      }
      const Eigen::Vector3d p_surface = p_G_L + R_ball * dir.normalized();
      const Eigen::Vector3d radial = (p_surface - p_G_L).normalized();
      scan.points_L_.push_back(p_surface + radial * noise.SampleLidarRangeNoise(rng_obs));
    }
    out.lidar_obs.push_back(scan);
  }

  const Eigen::Vector3d corners[4] = {
      Eigen::Vector3d(-0.025, -0.025, 0.0), Eigen::Vector3d(0.025, -0.025, 0.0),
      Eigen::Vector3d(0.025, 0.025, 0.0), Eigen::Vector3d(-0.025, 0.025, 0.0)};
  for (double t = geom.camera_dt_s; t <= t_end - 1e-9; t += geom.camera_dt_s) {
    AprilTagObservation det;
    det.t_sensor_ = t + out.gt.t_d_C_s;
    det.tag_id_ = 0;
    det.sensor_id_ = 0;
    det.detection_confidence_ = 1.0;
    const Eigen::Vector3d L_G_to_M = levers.L_G_to_M.at(0);
    for (int c = 0; c < 4; ++c) {
      const Eigen::Vector3d L_corner = levers.L_B_to_G + L_G_to_M + corners[c];
      const SE3d T_WB = out.gt_traj.pose_wb(t);
      const Eigen::Vector3d p_M_C = out.gt.T_CW * (T_WB * L_corner);
      const Eigen::Vector2d uv = ProjectRadtan(p_M_C, K, dist, nullptr);
      det.corners_pixel_[c] = uv + noise.SamplePixelNoise(rng_obs);
    }
    out.tag_obs.push_back(det);
  }
  return out;
}

inline SyntheticScenarioBundle BuildNoisyScenarioFromGeometry(
    uint32_t seed, const RealisticNoiseSpec& noise,
    const SyntheticFlightGeometry& geom) {
  return BuildNoisyScenarioFromGeometry(seed, seed, noise, geom);
}

inline SyntheticScenarioBundle BuildMultiLayerNoisyScenario(
    uint32_t seed, const RealisticNoiseSpec& noise) {
  return BuildNoisyScenarioFromGeometry(seed, noise,
                                        LegacyLocalMultiLayerGeometry());
}

/** Near-field FIM / corrected-pipeline scenario (15 s/layer, 2 Hz). */
inline SyntheticFlightGeometry NearFieldFimScenarioGeometry() {
  SyntheticFlightGeometry g = NearFieldMultiLayerGeometry();
  g.layer_duration_s = 15.0;
  g.rtk_dt_s = 0.5;
  g.lidar_dt_s = 0.5;
  g.camera_dt_s = 0.5;
  return g;
}

inline SyntheticScenarioBundle BuildNearFieldFimNoisyScenario(
    uint32_t seed, const RealisticNoiseSpec& noise) {
  return BuildNoisyScenarioFromGeometry(seed, noise,
                                        NearFieldFimScenarioGeometry());
}

inline SyntheticScenarioBundle BuildNearFieldCoplanarFimNoisyScenario(
    uint32_t seed, const RealisticNoiseSpec& noise) {
  SyntheticFlightGeometry g = NearFieldCoplanarGeometry();
  g.layer_duration_s = 15.0;
  g.rtk_dt_s = 0.5;
  g.lidar_dt_s = 0.5;
  g.camera_dt_s = 0.5;
  return BuildNoisyScenarioFromGeometry(seed, noise, g);
}

inline double NearFieldFlightDurationS(const SyntheticFlightGeometry& geom) {
  const int n_layers =
      std::max(1, static_cast<int>(geom.flight_layers_m.size()));
  return n_layers * geom.layer_duration_s;
}

inline SyntheticScenarioBundle BuildLocalNoisyScenario(uint32_t seed,
                                                       const RealisticNoiseSpec& noise) {
  const auto levers =
      LeverArmConfig::from_yaml(ConfigDirFromExperiments() + "/lever_arms.yaml");

  SyntheticScenarioBundle out;
  out.gt.T_LW = SE3d(SO3d::rotY(-0.15), Eigen::Vector3d(3.0, -1.0, 0.5));
  out.gt.T_CW = SE3d(SO3d::rotX(0.1), Eigen::Vector3d(2.0, 1.5, 0.2));
  out.gt_traj = MakeLocalGroundTruthTrajectory();

  const double R_ball = 0.10;
  PinholeIntrinsics K{600.0, 600.0, 320.0, 240.0};
  RadtanDistortion dist;

  std::mt19937 rng(seed);

  for (double t = 0.2; t <= 4.8; t += 0.1) {
    RTKMeasurement m;
    m.t_world_ = t;
    m.fix_status_ = RTKMeasurement::FixStatus::FIXED;
    m.p_A_W_observed_ =
        out.gt_traj.antenna_position_w(t, levers.L_B_to_A) +
        noise.SampleRtkNoise(rng);
    m.covariance_ = noise.RtkPositionCovariance();
    out.rtk.push_back(m);
  }

  for (double t = 0.5; t <= 4.5; t += 0.4) {
    LiDARTargetObservation scan;
    scan.t_sensor_ = t + out.gt.t_d_L_s;
    scan.sensor_id_ = 0;
    for (int k = 0; k < 24; ++k) {
      const double phi = 2.0 * M_PI * k / 24.0;
      const Eigen::Vector3d p_G_W = out.gt_traj.sphere_center_w(t, levers.L_B_to_G);
      const Eigen::Vector3d p_G_L = out.gt.T_LW * p_G_W;
      const Eigen::Vector3d dir(std::cos(phi), std::sin(phi), 0.0);
      const Eigen::Vector3d p_surface = p_G_L + R_ball * dir.normalized();
      const Eigen::Vector3d radial = (p_surface - p_G_L).normalized();
      scan.points_L_.push_back(p_surface + radial * noise.SampleLidarRangeNoise(rng));
    }
    out.lidar_obs.push_back(scan);
  }

  const Eigen::Vector3d corners[4] = {
      Eigen::Vector3d(-0.025, -0.025, 0.0), Eigen::Vector3d(0.025, -0.025, 0.0),
      Eigen::Vector3d(0.025, 0.025, 0.0), Eigen::Vector3d(-0.025, 0.025, 0.0)};
  for (double t = 0.6; t <= 4.4; t += 0.35) {
    AprilTagObservation det;
    det.t_sensor_ = t + out.gt.t_d_C_s;
    det.tag_id_ = 0;
    det.sensor_id_ = 0;
    det.detection_confidence_ = 1.0;
    const Eigen::Vector3d L_G_to_M = levers.L_G_to_M.at(0);
    for (int c = 0; c < 4; ++c) {
      const Eigen::Vector3d L_corner = levers.L_B_to_G + L_G_to_M + corners[c];
      const SE3d T_WB = out.gt_traj.pose_wb(t);
      const Eigen::Vector3d p_M_C = out.gt.T_CW * (T_WB * L_corner);
      const Eigen::Vector2d uv = ProjectRadtan(p_M_C, K, dist, nullptr);
      det.corners_pixel_[c] = uv + noise.SamplePixelNoise(rng);
    }
    out.tag_obs.push_back(det);
  }
  return out;
}

inline PosteriorMarginals PosteriorFromReport(
    const ObservabilityReport& report, const AnalysisParameterLayout& layout) {
  PosteriorMarginals out;
  if (report.information_matrix.rows() == 0) {
    return out;
  }
  const Eigen::MatrixXd cov = report.information_matrix.inverse();
  auto std_at = [&](const std::vector<int>& idx, int k) -> double {
    if (k < 0 || static_cast<size_t>(k) >= idx.size()) {
      return 0.0;
    }
    const int i = idx[static_cast<size_t>(k)];
    if (i < 0 || i >= cov.rows()) {
      return 0.0;
    }
    return std::sqrt(std::max(0.0, cov(i, i)));
  };
  out.roll_LW_rad = std_at(layout.lidar_rot_fext_indices, 0);
  out.pitch_LW_rad = std_at(layout.lidar_rot_fext_indices, 1);
  out.yaw_LW_rad = std_at(layout.lidar_rot_fext_indices, 2);
  out.tx_LW_m = std_at(layout.lidar_trans_fext_indices, 0);
  out.ty_LW_m = std_at(layout.lidar_trans_fext_indices, 1);
  out.tz_LW_m = std_at(layout.lidar_trans_fext_indices, 2);
  return out;
}

inline CalibrationRunMetrics RunLocalCalibration(const SyntheticScenarioBundle& scenario,
                                                 int max_iters = 1000) {
  CalibrationEstimator estimator(ConfigDirFromExperiments());
  estimator.add_rtk_measurements(scenario.rtk);
  estimator.add_lidar_target_observations(0, scenario.lidar_obs);
  estimator.add_apriltag_observations(0, scenario.tag_obs);

  const ceres::Solver::Summary summary = estimator.solve(max_iters);
  if (!summary.IsSolutionUsable()) {
    throw std::runtime_error("solve failed: " + summary.BriefReport());
  }

  CalibrationRunMetrics m;
  const SE3d T_LW_est = estimator.get_T_LW(0);
  const SE3d T_CW_est = estimator.get_T_CW(0);

  m.rot_LW_deg = RotationErrorDeg(T_LW_est, scenario.gt.T_LW);
  m.rot_CW_deg = RotationErrorDeg(T_CW_est, scenario.gt.T_CW);
  m.pitch_LW_deg = PitchFromSO3Rad(T_LW_est.so3()) * 180.0 / M_PI;
  const double pitch_gt_deg = PitchFromSO3Rad(scenario.gt.T_LW.so3()) * 180.0 / M_PI;
  m.pitch_LW_err_deg = m.pitch_LW_deg - pitch_gt_deg;
  m.trans_LW_err_m = T_LW_est.translation() - scenario.gt.T_LW.translation();
  m.trans_CW_err_m = T_CW_est.translation() - scenario.gt.T_CW.translation();
  m.trans_LW_norm_mm = m.trans_LW_err_m.norm() * 1e3;
  m.trans_CW_norm_mm = m.trans_CW_err_m.norm() * 1e3;

  m.t_d_L_ms = estimator.get_t_d_lidar(0) * 1e3;
  m.t_d_C_ms = estimator.get_t_d_camera(0) * 1e3;
  m.t_d_L_err_ms = (estimator.get_t_d_lidar(0) - scenario.gt.t_d_L_s) * 1e3;
  m.t_d_C_err_ms = (estimator.get_t_d_camera(0) - scenario.gt.t_d_C_s) * 1e3;

  double sq_sum = 0.0;
  int count = 0;
  auto est_traj = estimator.get_trajectory();
  for (double t = 0.3; t <= 4.7; t += 0.1) {
    sq_sum += (est_traj->position_wb(t) - scenario.gt_traj.position_wb(t)).squaredNorm();
    ++count;
  }
  m.traj_rms_mm = std::sqrt(sq_sum / std::max(count, 1)) * 1e3;

  estimator.build_problem_for_analysis();
  ObservabilityAnalyzer analyzer;
  const ObservabilityReport report = analyzer.analyze(estimator);
  const AnalysisParameterLayout layout = estimator.analysis_parameter_layout();
  m.post = PosteriorFromReport(report, layout);
  return m;
}

struct RunningStats {
  int n = 0;
  double mean = 0.0;
  double m2 = 0.0;
  double vmin = 0.0;
  double vmax = 0.0;
  bool initialized = false;

  void Push(double x) {
    if (!initialized) {
      vmin = vmax = x;
      initialized = true;
    } else {
      vmin = std::min(vmin, x);
      vmax = std::max(vmax, x);
    }
    ++n;
    const double d = x - mean;
    mean += d / static_cast<double>(n);
    const double d2 = x - mean;
    m2 += d * d2;
  }

  double Std() const {
    if (n < 2) {
      return 0.0;
    }
    return std::sqrt(m2 / static_cast<double>(n - 1));
  }

  double Max() const { return vmax; }
  double Min() const { return vmin; }
};

inline std::string FormatMeanStd(const RunningStats& s, int precision = 4) {
  std::ostringstream oss;
  oss << std::setprecision(precision) << std::fixed << s.mean << " ± " << s.Std();
  return oss.str();
}

inline std::string FormatMeanStdMm(const RunningStats& s) {
  return FormatMeanStd(s, 4) + " mm";
}

inline std::string FormatMeanStdDeg(const RunningStats& s) {
  return FormatMeanStd(s, 4) + " deg";
}

inline std::string FormatMeanStdMs(const RunningStats& s) {
  return FormatMeanStd(s, 4) + " ms";
}

inline std::string FormatMeanStdMrad(const RunningStats& s) {
  std::ostringstream oss;
  oss << std::setprecision(4) << std::fixed << (s.mean * 1000.0) << " ± "
      << (s.Std() * 1000.0) << " mrad";
  return oss.str();
}

inline std::string FormatMeanStdMax(const RunningStats& s, const char* unit,
                                    int precision = 4) {
  std::ostringstream oss;
  oss << std::setprecision(precision) << std::fixed << s.mean << " ± " << s.Std()
      << " (max " << s.Max() << ") " << unit;
  return oss.str();
}

}  // namespace experiments
}  // namespace clic_calib
