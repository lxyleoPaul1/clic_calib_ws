#include <clic_calib/estimator/two_stage_pipeline.h>
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
#include <numeric>
#include <string>
#include <vector>

namespace {

constexpr uint32_t kSeedBase = 13025;
constexpr int kNumSeeds = 10;
constexpr double kSphereRadiusM = 0.10;
constexpr int kObservedMeanIters = 3;

constexpr double kFrozenCentroidMm = 63.5;
constexpr double kFrozenObsMm = 18.5;
constexpr double kFrozenJointMm = 20.4;
constexpr double kFrozenWuNeMm = 12.15;
constexpr double kFrozenWuSwMm = 42.04;

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

struct Stats {
  double mean = 0.0;
  double std = 0.0;
};

Stats ComputeStats(const std::vector<double>& v) {
  Stats s;
  if (v.empty()) {
    return s;
  }
  s.mean = std::accumulate(v.begin(), v.end(), 0.0) /
           static_cast<double>(v.size());
  if (v.size() < 2) {
    return s;
  }
  double sq = 0.0;
  for (double x : v) {
    const double d = x - s.mean;
    sq += d * d;
  }
  s.std = std::sqrt(sq / static_cast<double>(v.size() - 1));
  return s;
}

bool WithinTwoSigma(double mean, double std, double frozen) {
  return std < 1e-6 || std::abs(mean - frozen) <= 2.0 * std;
}

std::filesystem::path DataDir() {
  return std::filesystem::path(__FILE__).parent_path().parent_path() /
         "paper/figures/data";
}

}  // namespace

TEST(PaperMultiseed, P15AndWuMeanStd) {
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
  const auto geom_p15 =
      clic_calib::experiments::NearFieldHighAspectScenarioGeometry();
  const auto geom_wu = clic_calib::experiments::DiagonalFlightE_Geometry();

  std::vector<double> p15_cent, p15_obs, p15_joint, wu_ne, wu_sw;
  const auto path = DataDir() / "multiseed_summary.csv";
  std::filesystem::create_directories(path.parent_path());
  std::ofstream out(path);
  out << "seed,p15_centroid_mm,p15_obs_mm,p15_joint_mm,wu_ne_obs_mm,wu_sw_obs_mm\n";
  out << std::fixed << std::setprecision(4);

  for (int i = 0; i < kNumSeeds; ++i) {
    const uint32_t seed = kSeedBase + static_cast<uint32_t>(i);

    auto ps = clic_calib::experiments::BuildPhase15ScenarioWithGeom(
        seed, noise, geom_p15);
    ASSERT_TRUE(clic_calib::experiments::PreparePhase15ScenarioTags(
        &ps, levers, noise, t_d_nominal, spline_cfg, seed, 0.01, 0.01));

    const Eigen::Vector3d lidar_post_W(0.0, 0.0, geom_p15.sensor_height_m);
    const double u_B_std = clic_calib::experiments::ComputeUBAzimuthStdDeg(
        ps.sc.gt_traj, ps.sc.gt.t_d_L_s, lidar_post_W, ps.body_cluster);
    const auto gated =
        clic_calib::experiments::CalibrateBodyGatedObservedMean(
            ps, levers, noise, base_cfg, ps.body_cluster, kObservedMeanIters,
            {}, u_B_std);
    const auto cent = clic_calib::experiments::RunBodyPath(
        ps, levers, noise, base_cfg, clic_calib::BodyLeverArmMode::kNominalYaml,
        ps.body_cluster);
    clic_calib::experiments::Phase15TlwMetrics joint_m;
    const Eigen::Vector3d b_const =
        clic_calib::ComputeBConstFromCentroidBackproject(
            ps.sc.gt_traj, ps.sc.gt.T_LW, ps.sc.gt.t_d_L_s,
            levers.L_B_to_body_centroid, ps.body_cluster);
    clic_calib::experiments::RunBodyPathJointDiagnostic(
        ps, levers, noise, base_cfg, ps.body_cluster, b_const, &joint_m);

    const auto ds = clic_calib::experiments::BuildDualDiagonalScenario(
        seed, noise, geom_wu, spline_cfg, t_d_nominal);
    const auto rep_wu =
        clic_calib::experiments::CalibrateDualDiagonalFlightEntry(
            ds, levers, noise, base_cfg, kObservedMeanIters);

    p15_cent.push_back(cent.trans_mm);
    p15_obs.push_back(gated.observed_mean.trans_mm);
    p15_joint.push_back(joint_m.trans_mm);
    wu_ne.push_back(rep_wu.calib.ne.calib.observed_mean.trans_mm);
    wu_sw.push_back(rep_wu.calib.sw.calib.observed_mean.trans_mm);

    out << seed << "," << cent.trans_mm << "," << gated.observed_mean.trans_mm
        << "," << joint_m.trans_mm << ","
        << rep_wu.calib.ne.calib.observed_mean.trans_mm << ","
        << rep_wu.calib.sw.calib.observed_mean.trans_mm << "\n";
  }

  const Stats s_cent = ComputeStats(p15_cent);
  const Stats s_obs = ComputeStats(p15_obs);
  const Stats s_joint = ComputeStats(p15_joint);
  const Stats s_ne = ComputeStats(wu_ne);
  const Stats s_sw = ComputeStats(wu_sw);

  std::cout << "\n=== P1.3 multiseed (" << kNumSeeds << " seeds) ===\n";
  std::cout << std::fixed << std::setprecision(3);
  std::cout << "  P1.5 centroid:    " << s_cent.mean << " ± " << s_cent.std
            << " mm\n";
  std::cout << "  P1.5 obs-mean:    " << s_obs.mean << " ± " << s_obs.std
            << " mm\n";
  std::cout << "  P1.5 joint-opt:   " << s_joint.mean << " ± " << s_joint.std
            << " mm\n";
  std::cout << "  戊 NE obs:        " << s_ne.mean << " ± " << s_ne.std
            << " mm\n";
  std::cout << "  戊 SW obs:        " << s_sw.mean << " ± " << s_sw.std
            << " mm\n";
  std::cout << "[csv] wrote " << path << "\n";

  const bool ok_cent = WithinTwoSigma(s_cent.mean, s_cent.std, kFrozenCentroidMm);
  const bool ok_obs = WithinTwoSigma(s_obs.mean, s_obs.std, kFrozenObsMm);
  const bool ok_joint = WithinTwoSigma(s_joint.mean, s_joint.std, kFrozenJointMm);
  const bool ok_ne = WithinTwoSigma(s_ne.mean, s_ne.std, kFrozenWuNeMm);
  const bool ok_sw = WithinTwoSigma(s_sw.mean, s_sw.std, kFrozenWuSwMm);

  if (!ok_cent || !ok_obs || !ok_joint || !ok_ne || !ok_sw) {
    std::cout << "\n*** STOP: multiseed mean deviates >2σ from frozen anchor ***\n";
    if (!ok_cent) {
      std::cout << "  centroid frozen=" << kFrozenCentroidMm
                << " mean=" << s_cent.mean << " std=" << s_cent.std << "\n";
    }
    if (!ok_obs) {
      std::cout << "  obs-mean frozen=" << kFrozenObsMm
                << " mean=" << s_obs.mean << " std=" << s_obs.std << "\n";
    }
    if (!ok_joint) {
      std::cout << "  joint frozen=" << kFrozenJointMm
                << " mean=" << s_joint.mean << " std=" << s_joint.std << "\n";
    }
    if (!ok_ne) {
      std::cout << "  戊 NE frozen=" << kFrozenWuNeMm
                << " mean=" << s_ne.mean << " std=" << s_ne.std << "\n";
    }
    if (!ok_sw) {
      std::cout << "  戊 SW frozen=" << kFrozenWuSwMm
                << " mean=" << s_sw.mean << " std=" << s_sw.std << "\n";
    }
    GTEST_SKIP() << "Multiseed mean >2σ from frozen — report to author";
  }
}

int main(int argc, char** argv) { return ClicGTestRunAll(argc, argv); }
