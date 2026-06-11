#pragma once

/**
 * Dual-diagonal LiDAR **simulation** + Phase-1.5 **calibration reuse**.
 * Stage-2 dual LiDAR = two independent single-LiDAR calibrations; relative
 * extrinsic composed post hoc.
 */

#include "diagnostic/attitude_scenario_common.hpp"
#include "diagnostic/closed_form_init_common.hpp"

#include <clic_calib/utils/camera_projection.h>
#include "experiments/body_sampling_common.hpp"
#include "experiments/dual_lidar_diagonal_geometry.hpp"
#include "experiments/phase15_ablation_common.hpp"

#include <map>
#include <string>
#include <vector>

namespace clic_calib {
namespace experiments {

struct DualLidarGtExtrinsics {
  std::map<std::string, SE3d> T_LW;
  double t_d_L_s = 0.030;
  SE3d T_CW;
  double t_d_C_s = -0.015;
};

struct DualLidarPhase3Scenario {
  SyntheticScenarioBundle sc;
  DualDiagonalFlightGeometry geom;
  DualLidarGtExtrinsics gt_lidars;
  std::map<std::string, std::vector<BodyClusterObservation>> body_by_sensor;
};

/** GT-template AprilTag stream (same schema as BuildNoisyScenarioFromGeometry). */
inline void AppendTemplateTagObservations(
    SyntheticScenarioBundle* sc, const LeverArmConfig& levers,
    const SE3d& T_CW, double t_end, double camera_dt_s, double t_d_C_s,
    const NoiseModel& noise, uint32_t seed_obs) {
  const PinholeIntrinsics K{600.0, 600.0, 320.0, 240.0};
  const RadtanDistortion dist;
  const Eigen::Vector3d corners[4] = {
      Eigen::Vector3d(-0.025, -0.025, 0.0), Eigen::Vector3d(0.025, -0.025, 0.0),
      Eigen::Vector3d(0.025, 0.025, 0.0), Eigen::Vector3d(-0.025, 0.025, 0.0)};
  std::mt19937 rng_obs(seed_obs);
  sc->tag_obs.clear();
  for (double t = camera_dt_s; t <= t_end - 1e-9; t += camera_dt_s) {
    AprilTagObservation det;
    det.t_sensor_ = t + t_d_C_s;
    det.tag_id_ = 0;
    det.sensor_id_ = 0;
    det.detection_confidence_ = 1.0;
    const Eigen::Vector3d L_G_to_M = levers.L_G_to_M.at(0);
    for (int c = 0; c < 4; ++c) {
      const Eigen::Vector3d L_corner = levers.L_B_to_G + L_G_to_M + corners[c];
      const SE3d T_WB = sc->gt_traj.pose_wb(t);
      const Eigen::Vector3d p_M_C = T_CW * (T_WB * L_corner);
      det.corners_pixel_[c] = ProjectRadtan(p_M_C, K, dist, nullptr) +
                              noise.SamplePixelNoise(rng_obs);
    }
    sc->tag_obs.push_back(det);
  }
}

inline BodyClusterObservation MakeBodyClusterObservationForSensor(
    double t_sensor, const BodySampleResult& sample, const NoiseModel& noise,
    const std::string& sensor_key, int sensor_id, std::mt19937* rng,
    bool add_residual_noise = true) {
  BodyClusterObservation obs =
      MakeBodyClusterObservation(t_sensor, sample, noise, rng, add_residual_noise);
  obs.sensor_key_ = sensor_key;
  obs.sensor_id_ = sensor_id;
  return obs;
}

inline std::vector<BodyClusterObservation> BuildBodyClusterForDiagonalSensor(
    const BodyTrajectory& traj, const SE3d& T_LW, double t_d_L_s,
    const LeverArmConfig& levers, const NoiseModel& noise,
    const DualDiagonalFlightGeometry& geom, const std::string& sensor_key,
    int sensor_id, uint32_t seed_obs, bool add_residual_noise = true) {
  const Eigen::Vector3d post_W = DiagonalLidarPostW(geom.dual_preset, sensor_key);
  const double t_end = 2.0 * geom.sector_duration_s;

  std::vector<BodyClusterObservation> out;
  std::mt19937 rng(seed_obs);
  for (double t = geom.lidar_dt_s; t <= t_end - 1e-9; t += geom.lidar_dt_s) {
    if (!IsTimeInSensorSector(t, sensor_key, geom)) {
      continue;
    }
    const SE3d T_WB = traj.pose_wb(t);
    if (!IsDroneVisibleFromPost(T_WB.translation(), post_W)) {
      continue;
    }
    const BodySampleResult sample = SampleBodyPointsLidar(
        T_WB.so3(), T_WB.translation(), T_LW, levers.L_B_to_body_centroid, &rng);
    if (sample.points_L.size() < 10) {
      continue;
    }
    out.push_back(MakeBodyClusterObservationForSensor(
        t + t_d_L_s, sample, noise, sensor_key, sensor_id, &rng,
        add_residual_noise));
  }
  return out;
}

inline DualLidarPhase3Scenario BuildDualDiagonalScenario(
    uint32_t seed, const NoiseModel& noise,
    const DualDiagonalFlightGeometry& geom,
    const two_stage_probe::SplineConfig& spline_cfg,
    const two_stage_probe::CoarseExtrinsicInit& t_d_nominal) {
  const auto levers =
      LeverArmConfig::from_yaml(ConfigDirFromExperiments() + "/lever_arms.yaml");

  DualLidarPhase3Scenario out;
  out.geom = geom;
  out.sc.gt_traj = BuildGtTrajectoryDualDiagonal(geom);
  out.sc.gt.t_d_L_s = 0.030;
  out.sc.gt.t_d_C_s = -0.015;

  out.gt_lidars.t_d_L_s = out.sc.gt.t_d_L_s;
  out.gt_lidars.t_d_C_s = out.sc.gt.t_d_C_s;
  out.gt_lidars.T_LW["lidar_NE"] = T_LWFromRoadsidePost(
      DiagonalLidarPostW(geom.dual_preset, "lidar_NE"),
      geom.dual_preset.look_target_W);
  out.gt_lidars.T_LW["lidar_SW"] = T_LWFromRoadsidePost(
      DiagonalLidarPostW(geom.dual_preset, "lidar_SW"),
      geom.dual_preset.look_target_W);
  const SE3d T_WC_ne =
      out.gt_lidars.T_LW.at("lidar_NE").inverse() *
      SE3d(SO3d::rotX(0.08), Eigen::Vector3d(0.2, 0.0, 0.0));
  out.gt_lidars.T_CW = T_WC_ne.inverse();
  out.sc.gt.T_LW = out.gt_lidars.T_LW.at("lidar_NE");
  out.sc.gt.T_CW = out.gt_lidars.T_CW;

  const double t_end = 2.0 * geom.sector_duration_s;
  std::mt19937 rng_traj(seed);

  for (double t = geom.rtk_dt_s; t <= t_end - 1e-9; t += geom.rtk_dt_s) {
    RTKMeasurement m;
    m.t_world_ = t;
    m.fix_status_ = RTKMeasurement::FixStatus::FIXED;
    m.p_A_W_observed_ =
        out.sc.gt_traj.antenna_position_w(t, levers.L_B_to_A) +
        noise.SampleRtkNoise(rng_traj);
    m.covariance_ = noise.RtkPositionCovariance();
    out.sc.rtk.push_back(m);
  }

  std::mt19937 rng_att(seed + 7919u);
  AppendAttitudeObservationsForDuration(&out.sc, t_end, AttitudeNoiseSpec(),
                                        &rng_att);

  out.body_by_sensor["lidar_NE"] = BuildBodyClusterForDiagonalSensor(
      out.sc.gt_traj, out.gt_lidars.T_LW.at("lidar_NE"), out.sc.gt.t_d_L_s,
      levers, noise, geom, "lidar_NE", 0, seed + 17u, true);
  out.body_by_sensor["lidar_SW"] = BuildBodyClusterForDiagonalSensor(
      out.sc.gt_traj, out.gt_lidars.T_LW.at("lidar_SW"), out.sc.gt.t_d_L_s,
      levers, noise, geom, "lidar_SW", 1, seed + 29u, true);

  AppendTemplateTagObservations(&out.sc, levers, out.gt_lidars.T_CW, t_end,
                                geom.camera_dt_s, out.sc.gt.t_d_C_s, noise,
                                seed + 1u);
  Phase15Scenario tag_ps;
  tag_ps.sc = out.sc;
  tag_ps.geom = geom;
  if (PreparePhase15ScenarioTags(&tag_ps, levers, noise, t_d_nominal,
                                 spline_cfg, seed)) {
    out.sc.tag_obs = tag_ps.sc.tag_obs;
  }
  return out;
}

/** Slice shared dual streams into a per-sensor Phase-1.5 scenario. */
inline Phase15Scenario ToPhase15SensorSlice(const DualLidarPhase3Scenario& ds,
                                            const std::string& sensor_key) {
  Phase15Scenario ps;
  ps.sc = ds.sc;
  ps.sc.gt.T_LW = ds.gt_lidars.T_LW.at(sensor_key);
  ps.geom = ds.geom;
  ps.body_cluster = ds.body_by_sensor.at(sensor_key);
  return ps;
}

struct RelativeExtrinsicMetrics {
  double rot_deg = 0.0;
  double trans_mm = 0.0;
};

struct DualSensorCalibResult {
  std::string sensor_key;
  GatedObservedMeanCalibResult calib;
  int body_frames = 0;
};

struct DualFlightCalibReport {
  DualSensorCalibResult ne;
  DualSensorCalibResult sw;
  RelativeExtrinsicMetrics rel_centroid;
  RelativeExtrinsicMetrics rel_observed;
  double b_angle_body_deg = 0.0;
};

inline RelativeExtrinsicMetrics RelativeExtrinsicError(
    const SE3d& T_LW_a_est, const SE3d& T_LW_b_est, const SE3d& T_LW_a_gt,
    const SE3d& T_LW_b_gt) {
  const SE3d T_ab_est = T_LW_a_est * T_LW_b_est.inverse();
  const SE3d T_ab_gt = T_LW_a_gt * T_LW_b_gt.inverse();
  RelativeExtrinsicMetrics m;
  m.rot_deg = RotationErrorDeg(T_ab_est, T_ab_gt);
  m.trans_mm =
      (T_ab_est.translation() - T_ab_gt.translation()).norm() * 1e3;
  return m;
}

inline double AngleDegBetweenVectors(const Eigen::Vector3d& a,
                                     const Eigen::Vector3d& b) {
  const double na = a.norm();
  const double nb = b.norm();
  if (na < 1e-12 || nb < 1e-12) {
    return 0.0;
  }
  const double c = std::clamp(a.dot(b) / (na * nb), -1.0, 1.0);
  return std::acos(c) * 180.0 / M_PI;
}

inline DualSensorCalibResult CalibrateDualSensorViaPhase15(
    const Phase15Scenario& ps, const LeverArmConfig& levers,
    const NoiseModel& noise, const TwoStagePipelineConfig& cfg,
    const std::string& sensor_key, int observed_mean_iters = 3,
    double u_B_azimuth_std_deg = -1.0) {
  DualSensorCalibResult out;
  out.sensor_key = sensor_key;
  out.body_frames = static_cast<int>(ps.body_cluster.size());
  out.calib = CalibrateBodyGatedObservedMean(
      ps, levers, noise, cfg, ps.body_cluster, observed_mean_iters, {},
      u_B_azimuth_std_deg);
  return out;
}

inline DualFlightCalibReport CalibrateDualDiagonalFlight(
    const DualLidarPhase3Scenario& ds, const LeverArmConfig& levers,
    const NoiseModel& noise, const TwoStagePipelineConfig& cfg,
    int observed_mean_iters = 3) {
  DualFlightCalibReport rep;
  const Phase15Scenario ps_ne = ToPhase15SensorSlice(ds, "lidar_NE");
  const Phase15Scenario ps_sw = ToPhase15SensorSlice(ds, "lidar_SW");
  const Eigen::Vector3d post_ne =
      DiagonalLidarPostW(ds.geom.dual_preset, "lidar_NE");
  const Eigen::Vector3d post_sw =
      DiagonalLidarPostW(ds.geom.dual_preset, "lidar_SW");
  const double u_B_std_ne = ComputeUBAzimuthStdDeg(
      ds.sc.gt_traj, ds.sc.gt.t_d_L_s, post_ne, ps_ne.body_cluster);
  const double u_B_std_sw = ComputeUBAzimuthStdDeg(
      ds.sc.gt_traj, ds.sc.gt.t_d_L_s, post_sw, ps_sw.body_cluster);
  rep.ne = CalibrateDualSensorViaPhase15(ps_ne, levers, noise, cfg, "lidar_NE",
                                         observed_mean_iters, u_B_std_ne);
  rep.sw = CalibrateDualSensorViaPhase15(ps_sw, levers, noise, cfg, "lidar_SW",
                                         observed_mean_iters, u_B_std_sw);
  rep.rel_centroid = RelativeExtrinsicError(
      rep.ne.calib.T_LW_centroid, rep.sw.calib.T_LW_centroid,
      ds.gt_lidars.T_LW.at("lidar_NE"), ds.gt_lidars.T_LW.at("lidar_SW"));
  rep.rel_observed = RelativeExtrinsicError(
      rep.ne.calib.T_LW_observed_mean, rep.sw.calib.T_LW_observed_mean,
      ds.gt_lidars.T_LW.at("lidar_NE"), ds.gt_lidars.T_LW.at("lidar_SW"));
  const Eigen::Vector3d b_ne = ComputeBConstFromCentroidBackproject(
      ds.sc.gt_traj, ds.gt_lidars.T_LW.at("lidar_NE"), ds.sc.gt.t_d_L_s,
      levers.L_B_to_body_centroid, ds.body_by_sensor.at("lidar_NE"));
  const Eigen::Vector3d b_sw = ComputeBConstFromCentroidBackproject(
      ds.sc.gt_traj, ds.gt_lidars.T_LW.at("lidar_SW"), ds.sc.gt.t_d_L_s,
      levers.L_B_to_body_centroid, ds.body_by_sensor.at("lidar_SW"));
  rep.b_angle_body_deg = AngleDegBetweenVectors(b_ne, b_sw);
  return rep;
}

/** Intersection center consistency via per-sensor estimated extrinsics [mm]. */
inline double CenterRegistrationErrorMm(
    const SE3d& T_LW_ne_est, const SE3d& T_LW_sw_est,
    const SE3d& T_LW_ne_gt, const SE3d& T_LW_sw_gt,
    const Eigen::Vector3d& p_center_W) {
  const Eigen::Vector3d p_C_L_ne = T_LW_ne_gt * p_center_W;
  const Eigen::Vector3d p_C_L_sw = T_LW_sw_gt * p_center_W;
  const Eigen::Vector3d p_W_via_ne = T_LW_ne_est.inverse() * p_C_L_ne;
  const Eigen::Vector3d p_W_via_sw = T_LW_sw_est.inverse() * p_C_L_sw;
  return (p_W_via_ne - p_W_via_sw).norm() * 1e3;
}

struct DualFlightEntryMetrics {
  DualFlightCalibReport calib;
  double center_reg_cent_mm = 0.0;
  double center_reg_obs_mm = 0.0;
};

inline DualFlightEntryMetrics CalibrateDualDiagonalFlightEntry(
    const DualLidarPhase3Scenario& ds, const LeverArmConfig& levers,
    const NoiseModel& noise, const TwoStagePipelineConfig& cfg,
    int observed_mean_iters = 3) {
  DualFlightEntryMetrics out;
  out.calib = CalibrateDualDiagonalFlight(ds, levers, noise, cfg,
                                          observed_mean_iters);
  const Eigen::Vector3d p_C = ds.geom.dual_preset.look_target_W;
  out.center_reg_cent_mm = CenterRegistrationErrorMm(
      out.calib.ne.calib.T_LW_centroid, out.calib.sw.calib.T_LW_centroid,
      ds.gt_lidars.T_LW.at("lidar_NE"), ds.gt_lidars.T_LW.at("lidar_SW"), p_C);
  out.center_reg_obs_mm = CenterRegistrationErrorMm(
      out.calib.ne.calib.T_LW_observed_mean,
      out.calib.sw.calib.T_LW_observed_mean,
      ds.gt_lidars.T_LW.at("lidar_NE"), ds.gt_lidars.T_LW.at("lidar_SW"), p_C);
  return out;
}

}  // namespace experiments
}  // namespace clic_calib
