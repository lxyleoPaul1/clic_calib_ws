#include <clic_calib/spline/trajectory.h>
#include <clic_calib/utils/lever_arm.h>
#include <clic_calib/utils/noise_model.h>
#include "gtest_ceres_guard.hpp"

#include <gtest/gtest.h>

#include "diagnostic/closed_form_init_common.hpp"
#include "diagnostic/two_stage_probe_common.hpp"
#include "experiments/dual_lidar_diagonal_geometry.hpp"
#include "experiments/dual_lidar_scenario_common.hpp"
#include "experiments/noise_regime_common.hpp"
#include "experiments/phase15_ablation_common.hpp"

#include <cmath>
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

/** POI sector lock with sector 0↔1 lidar targets swapped (test-only mirror). */
clic_calib::SE3d PoseWbMirrorPoiWu(
    double t, const clic_calib::experiments::DualDiagonalFlightGeometry& geom) {
  using clic_calib::experiments::DiagonalLidarPostW;
  using clic_calib::experiments::SectorIndexFromTime;
  using clic_calib::experiments::SectorOrbitCenterW;
  using clic_calib::SO3d;
  using clic_calib::SE3d;

  const int sector = SectorIndexFromTime(t, geom);
  const double t_local =
      (sector == 0) ? t : (t - geom.sector_duration_s);
  const int n_layers =
      std::max(1, static_cast<int>(geom.flight_layers_m.size()));
  const double phase = t_local / geom.layer_duration_s;
  const int layer =
      std::min(n_layers - 1, static_cast<int>(phase));
  const double t_in_layer = t_local - layer * geom.layer_duration_s;
  const double u = t_in_layer / geom.layer_duration_s;

  const double az_span_rad = geom.azimuth_span_deg * M_PI / 180.0;
  const double az = az_span_rad * std::sin(2.0 * M_PI * u);
  const double dist_center =
      0.5 * (geom.flight_dist_min_m + geom.flight_dist_max_m);
  const double dist_amp =
      0.5 * (geom.flight_dist_max_m - geom.flight_dist_min_m);
  const double dist = dist_center + dist_amp * std::cos(2.0 * M_PI * u);

  const Eigen::Vector3d center = SectorOrbitCenterW(sector, geom);
  const double orbit_yaw = (sector == 0) ? (M_PI / 4.0) : (-3.0 * M_PI / 4.0);
  const double cx = dist * std::cos(az + orbit_yaw);
  const double cy = dist * std::sin(az + orbit_yaw);
  Eigen::Vector3d p_wb = center + Eigen::Vector3d(cx, cy, 0.0);
  if (geom.multilayer) {
    p_wb.z() = geom.flight_layers_m[static_cast<size_t>(layer)];
  } else {
    p_wb.z() = geom.coplanar_z_m;
  }

  const std::string active_lidar =
      (sector == 0) ? "lidar_SW" : "lidar_NE";
  const Eigen::Vector3d post =
      DiagonalLidarPostW(geom.dual_preset, active_lidar);
  Eigen::Vector3d to_lidar = post - p_wb;
  const double yaw_poi = std::atan2(to_lidar.y(), to_lidar.x());
  SO3d R = SO3d::rotZ(yaw_poi);
  if (geom.high_attitude_variation) {
    double roll_scale = 1.0;
    const bool tighten_roll =
        geom.poi_roll_scale_all_sectors || (sector == 0);
    if (tighten_roll && geom.poi_sector0_attitude_scale >= 0.0) {
      roll_scale = geom.poi_sector0_attitude_scale;
    }
    R = R * SO3d::rotY(geom.pitch_amp_rad * std::sin(2.0 * M_PI * u)) *
            SO3d::rotX(roll_scale * geom.roll_amp_rad *
                       std::cos(2.0 * M_PI * u));
  }
  return SE3d(R, p_wb);
}

clic_calib::BodyTrajectory BuildMirrorPoiTrajectory(
    const clic_calib::experiments::DualDiagonalFlightGeometry& geom) {
  const double t_end = 2.0 * geom.sector_duration_s;
  clic_calib::BodyTrajectory traj(0.05, 0.0);
  const int num_knots =
      std::max(24, static_cast<int>(std::ceil(t_end / 0.05)) + 4);
  traj.setKnots(PoseWbMirrorPoiWu(0.0, geom), num_knots);
  for (int i = 0; i < num_knots; ++i) {
    const double t = std::min(t_end, static_cast<double>(i) * 0.05);
    traj.setKnot(PoseWbMirrorPoiWu(t, geom), i);
  }
  const int64_t t_end_ns = static_cast<int64_t>(t_end * clic_calib::S_TO_NS);
  traj.extendKnotsTo(t_end_ns, traj.getKnotSO3(num_knots - 1),
                     traj.getKnotPos(num_knots - 1));
  return traj;
}

clic_calib::experiments::DualLidarPhase3Scenario BuildMirrorWuScenario(
    uint32_t seed, const clic_calib::NoiseModel& noise,
    const clic_calib::experiments::DualDiagonalFlightGeometry& geom,
    const clic_calib::two_stage_probe::SplineConfig& spline_cfg,
    const clic_calib::two_stage_probe::CoarseExtrinsicInit& t_d_nominal) {
  auto ds = clic_calib::experiments::BuildDualDiagonalScenario(
      seed, noise, geom, spline_cfg, t_d_nominal);
  ds.geom.label = "flight_W_mirror_poi_swap";
  ds.sc.gt_traj = BuildMirrorPoiTrajectory(geom);
  const auto levers = clic_calib::LeverArmConfig::from_yaml(
      clic_calib::experiments::ConfigDirFromExperiments() + "/lever_arms.yaml");
  ds.body_by_sensor["lidar_NE"] =
      clic_calib::experiments::BuildBodyClusterForDiagonalSensor(
          ds.sc.gt_traj, ds.gt_lidars.T_LW.at("lidar_NE"), ds.sc.gt.t_d_L_s,
          levers, noise, geom, "lidar_NE", 0, seed + 17u, true);
  ds.body_by_sensor["lidar_SW"] =
      clic_calib::experiments::BuildBodyClusterForDiagonalSensor(
          ds.sc.gt_traj, ds.gt_lidars.T_LW.at("lidar_SW"), ds.sc.gt.t_d_L_s,
          levers, noise, geom, "lidar_SW", 1, seed + 29u, true);
  return ds;
}

std::filesystem::path DataDir() {
  return std::filesystem::path(__FILE__).parent_path().parent_path() /
         "paper/figures/data";
}

}  // namespace

TEST(PaperMirrorControl, WuAsymmetryFlipsUnderPoiSectorMirror) {
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
  const auto geom = clic_calib::experiments::DiagonalFlightE_Geometry();

  const auto ds_base = clic_calib::experiments::BuildDualDiagonalScenario(
      kRepSeed, noise, geom, spline_cfg, t_d_nominal);
  const auto ds_mirror = BuildMirrorWuScenario(kRepSeed, noise, geom, spline_cfg,
                                               t_d_nominal);

  const auto rep_base =
      clic_calib::experiments::CalibrateDualDiagonalFlightEntry(
          ds_base, levers, noise, base_cfg, kObservedMeanIters);
  const auto rep_mirror =
      clic_calib::experiments::CalibrateDualDiagonalFlightEntry(
          ds_mirror, levers, noise, base_cfg, kObservedMeanIters);

  const double base_ne = rep_base.calib.ne.calib.observed_mean.trans_mm;
  const double base_sw = rep_base.calib.sw.calib.observed_mean.trans_mm;
  const double mir_ne = rep_mirror.calib.ne.calib.observed_mean.trans_mm;
  const double mir_sw = rep_mirror.calib.sw.calib.observed_mean.trans_mm;
  const double base_gap = base_sw - base_ne;
  const double mir_gap = mir_sw - mir_ne;

  std::cout << "\n=== P1.4 mirror control (POI sector swap) @ seed "
            << kRepSeed << " ===\n";
  std::cout << std::fixed << std::setprecision(2);
  std::cout << "  baseline  NE=" << base_ne << " SW=" << base_sw
            << "  gap(SW-NE)=" << base_gap << " mm\n";
  std::cout << "  mirrored  NE=" << mir_ne << " SW=" << mir_sw
            << "  gap(SW-NE)=" << mir_gap << " mm\n";
  std::cout << "  cross NE_base≈SW_mirror: |" << base_ne - mir_sw
            << "|  SW_base≈NE_mirror: |" << base_sw - mir_ne << "|\n";

  const auto path = DataDir() / "mirror_control.csv";
  std::filesystem::create_directories(path.parent_path());
  std::ofstream out(path);
  out << "regime,ne_obs_mm,sw_obs_mm,sw_minus_ne_mm\n";
  out << std::fixed << std::setprecision(4);
  out << "baseline," << base_ne << "," << base_sw << "," << base_gap << "\n";
  out << "mirror_poi_swap," << mir_ne << "," << mir_sw << "," << mir_gap << "\n";
  std::cout << "[csv] wrote " << path << "\n";

  EXPECT_GT(base_gap, 15.0);
  EXPECT_LT(std::abs(mir_gap), std::abs(base_gap));
  EXPECT_GT(mir_ne, base_ne);
}

int main(int argc, char** argv) { return ClicGTestRunAll(argc, argv); }
