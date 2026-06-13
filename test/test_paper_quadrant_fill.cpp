#include <clic_calib/target/body_centroid_analysis.h>
#include <clic_calib/utils/lever_arm.h>
#include <clic_calib/utils/noise_model.h>
#include "gtest_ceres_guard.hpp"

#include <gtest/gtest.h>

#include "diagnostic/closed_form_init_common.hpp"
#include "diagnostic/two_stage_probe_common.hpp"
#include "experiments/dual_lidar_aspect_bias_diagnostic.hpp"
#include "experiments/dual_lidar_diagonal_geometry.hpp"
#include "experiments/dual_lidar_scenario_common.hpp"
#include "experiments/noise_regime_common.hpp"
#include "experiments/phase15_ablation_common.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

namespace {

constexpr uint32_t kRepSeed = 13025;
constexpr double kSphereRadiusM = 0.10;
constexpr int kObservedMeanIters = 3;

clic_calib::TwoStagePipelineConfig MakeBaseConfig(
    const clic_calib::two_stage_probe::SplineConfig& spline_cfg,
    const clic_calib::two_stage_probe::CoarseExtrinsicInit& t_d_nominal) {
  clic_calib::TwoStagePipelineConfig cfg;
  cfg.stage1.knot_interval_s = spline_cfg.knot_interval_s;
  cfg.stage1.alpha_p = 0.01;
  cfg.stage1.alpha_R = 0.01;
  cfg.stage1.attitude_stride = 25;
  cfg.stage1.trim_to_observation_support = true;
  cfg.init.sphere_radius_m = kSphereRadiusM;
  cfg.init.nominal_t_d_L_s = t_d_nominal.t_d_L_s;
  cfg.init.nominal_t_d_C_s = t_d_nominal.t_d_C_s;
  cfg.init.camera_K = {600.0, 600.0, 320.0, 240.0};
  cfg.refine.sphere_radius_m = kSphereRadiusM;
  cfg.refine.t_d_max_abs_s = spline_cfg.t_d_max_abs_s;
  cfg.refine.lidar_cauchy_scale = 1.0;
  cfg.refine.camera_huber_delta_px = 2.0;
  cfg.refine.camera_K = cfg.init.camera_K;
  cfg.refine.max_iterations = 500;
  return cfg;
}

/** u_B sweep via wide orbit + fixed route heading (R_WB near-locked). */
clic_calib::experiments::DualDiagonalFlightGeometry QuadrantFillGeometry() {
  auto g = clic_calib::experiments::DiagonalFlightA_Geometry();
  g.label = "paper_quadrant_uB_sweep_locked_Rwb";
  g.azimuth_span_deg = 165.0;
  g.flight_dist_min_m = 18.0;
  g.flight_dist_max_m = 38.0;
  g.high_attitude_variation = false;
  return g;
}

std::filesystem::path DataDir() {
  return std::filesystem::path(__FILE__).parent_path().parent_path() /
         "paper/figures/data";
}

void ExportDingScatterCsv(
    const clic_calib::experiments::DualLidarPhase3Scenario& ds,
    const clic_calib::LeverArmConfig& levers) {
  const auto path = DataDir() / "ding_bias_scatter.csv";
  std::filesystem::create_directories(path.parent_path());
  std::ofstream out(path);
  out << "sensor,u_B_azimuth_deg,bias_x_mm,bias_y_mm,bias_z_mm\n";
  out << std::fixed << std::setprecision(4);
  for (const char* key : {"lidar_NE", "lidar_SW"}) {
    const Eigen::Vector3d post =
        clic_calib::experiments::DiagonalLidarPostW(ds.geom.dual_preset, key);
    const auto report = clic_calib::BuildAspectBiasScatterReport(
        ds.sc.gt_traj, ds.gt_lidars.T_LW.at(key), ds.sc.gt.t_d_L_s,
        levers.L_B_to_body_centroid, post, ds.body_by_sensor.at(key));
    for (const auto& s : report.samples) {
      out << key << "," << s.u_B_azimuth_deg << ","
          << s.bias_B.x() * 1e3 << "," << s.bias_B.y() * 1e3 << ","
          << s.bias_B.z() * 1e3 << "\n";
    }
  }
  std::cout << "[csv] wrote " << path << "\n";
}

}  // namespace

TEST(PaperQuadrantFill, UbSweepWithLockedAttitudeAndDingScatter) {
  const std::string config_dir =
      clic_calib::experiments::ConfigDirFromExperiments();
  const auto levers =
      clic_calib::LeverArmConfig::from_yaml(config_dir + "/lever_arms.yaml");
  const auto spline_cfg =
      clic_calib::two_stage_probe::LoadSplineConfig(config_dir + "/spline.yaml");
  const clic_calib::two_stage_probe::CoarseExtrinsicInit t_d_nominal =
      clic_calib::two_stage_probe::LoadCoarseExtrinsicsFromYaml(config_dir);
  const clic_calib::NoiseModel noise =
      clic_calib::NoiseModel::FromConfigDir(config_dir);
  const clic_calib::TwoStagePipelineConfig base_cfg =
      MakeBaseConfig(spline_cfg, t_d_nominal);

  // --- 2×2 empty cell: u_B sweeps × R_WB near-locked ---
  const auto geom_q = QuadrantFillGeometry();
  const auto ds_q = clic_calib::experiments::BuildDualDiagonalScenario(
      kRepSeed, noise, geom_q, spline_cfg, t_d_nominal);
  const auto rep_q =
      clic_calib::experiments::CalibrateDualDiagonalFlightEntry(
          ds_q, levers, noise, base_cfg, kObservedMeanIters);

  const auto spread_ne = clic_calib::ComputeAttitudeSpreadAtObservations(
      ds_q.sc.gt_traj, ds_q.sc.gt.t_d_L_s,
      ds_q.body_by_sensor.at("lidar_NE"));
  const auto spread_sw = clic_calib::ComputeAttitudeSpreadAtObservations(
      ds_q.sc.gt_traj, ds_q.sc.gt.t_d_L_s,
      ds_q.body_by_sensor.at("lidar_SW"));
  const Eigen::Vector3d post_ne =
      clic_calib::experiments::DiagonalLidarPostW(geom_q.dual_preset, "lidar_NE");
  const Eigen::Vector3d post_sw =
      clic_calib::experiments::DiagonalLidarPostW(geom_q.dual_preset, "lidar_SW");
  const double u_B_ne = clic_calib::experiments::ComputeUBAzimuthStdDeg(
      ds_q.sc.gt_traj, ds_q.sc.gt.t_d_L_s, post_ne,
      ds_q.body_by_sensor.at("lidar_NE"));
  const double u_B_sw = clic_calib::experiments::ComputeUBAzimuthStdDeg(
      ds_q.sc.gt_traj, ds_q.sc.gt.t_d_L_s, post_sw,
      ds_q.body_by_sensor.at("lidar_SW"));

  std::cout << "\n=== P1.2 quadrant fill: u_B sweep × R_WB locked @ seed "
            << kRepSeed << " ===\n";
  std::cout << std::fixed << std::setprecision(3);
  std::cout << "  NE yaw_cv=" << spread_ne.yaw_circular_variance
            << " pitch_std=" << spread_ne.pitch_std_deg
            << " u_B_std=" << u_B_ne << " deg\n";
  std::cout << "  SW yaw_cv=" << spread_sw.yaw_circular_variance
            << " pitch_std=" << spread_sw.pitch_std_deg
            << " u_B_std=" << u_B_sw << " deg\n";
  std::cout << "  NE cent=" << rep_q.calib.ne.calib.centroid_only.trans_mm
            << " obs=" << rep_q.calib.ne.calib.observed_mean.trans_mm
            << " applied="
            << (rep_q.calib.ne.calib.observed_mean_applied ? 1 : 0) << "\n";
  std::cout << "  SW cent=" << rep_q.calib.sw.calib.centroid_only.trans_mm
            << " obs=" << rep_q.calib.sw.calib.observed_mean.trans_mm
            << " applied="
            << (rep_q.calib.sw.calib.observed_mean_applied ? 1 : 0) << "\n";

  const auto path = DataDir() / "quadrant_fill.csv";
  std::filesystem::create_directories(path.parent_path());
  std::ofstream out(path);
  out << "regime,ne_cent_mm,ne_obs_mm,ne_obs_applied,sw_cent_mm,sw_obs_mm,"
         "sw_obs_applied,ne_yaw_cv,sw_yaw_cv,ne_u_B_std_deg,sw_u_B_std_deg\n";
  out << std::fixed << std::setprecision(4);
  out << "uB_sweep_Rwb_locked,"
      << rep_q.calib.ne.calib.centroid_only.trans_mm << ","
      << rep_q.calib.ne.calib.observed_mean.trans_mm << ","
      << (rep_q.calib.ne.calib.observed_mean_applied ? 1 : 0) << ","
      << rep_q.calib.sw.calib.centroid_only.trans_mm << ","
      << rep_q.calib.sw.calib.observed_mean.trans_mm << ","
      << (rep_q.calib.sw.calib.observed_mean_applied ? 1 : 0) << ","
      << spread_ne.yaw_circular_variance << "," << spread_sw.yaw_circular_variance
      << "," << u_B_ne << "," << u_B_sw << "\n";
  std::cout << "[csv] wrote " << path << "\n";

  // --- Fig. 2(a) data: Flight-D aspect scatter ---
  const auto geom_d = clic_calib::experiments::DiagonalFlightD_Geometry();
  const auto ds_d = clic_calib::experiments::BuildDualDiagonalScenario(
      kRepSeed, noise, geom_d, spline_cfg, t_d_nominal);
  ExportDingScatterCsv(ds_d, levers);

  EXPECT_GT(u_B_ne, 5.0);
  EXPECT_LT(spread_ne.pitch_std_deg, 5.0);
  EXPECT_FALSE(rep_q.calib.ne.calib.observed_mean_applied);
  EXPECT_FALSE(rep_q.calib.sw.calib.observed_mean_applied);
}

int main(int argc, char** argv) { return ClicGTestRunAll(argc, argv); }
