#include <clic_calib/estimator/stage1_trajectory_fitter.h>
#include <clic_calib/utils/lever_arm.h>
#include <clic_calib/utils/noise_model.h>
#include "gtest_ceres_guard.hpp"

#include <gtest/gtest.h>

#include "diagnostic/closed_form_init_common.hpp"
#include "diagnostic/two_stage_probe_common.hpp"
#include "experiments/dual_lidar_diagonal_geometry.hpp"
#include "experiments/dual_lidar_scenario_common.hpp"
#include "experiments/phase15_ablation_common.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <vector>

namespace {

constexpr uint32_t kSeedBase = 13025;
constexpr int kNumOuterSeeds = 10;
constexpr uint32_t kMcSeedBase = 13000;
constexpr int kDefaultMcDraws = 20;
constexpr int kObservedMeanIters = 3;
constexpr double kStage1AlphaP = 0.01;
constexpr double kStage1AlphaR = 0.01;
constexpr double kFrozenCoherentRatio = 0.166;
constexpr double kFrozenWhiteRatio = 3.101;

int NumMcDraws() {
  if (const char* env = std::getenv("CLIC_PAPER_RELEXT_MC_N")) {
    return std::max(10, std::atoi(env));
  }
  return kDefaultMcDraws;
}

clic_calib::TwoStagePipelineConfig MakeBaseConfig(
    const clic_calib::two_stage_probe::SplineConfig& spline_cfg,
    const clic_calib::two_stage_probe::CoarseExtrinsicInit& t_d_nominal) {
  clic_calib::TwoStagePipelineConfig cfg;
  cfg.stage1.knot_interval_s = spline_cfg.knot_interval_s;
  cfg.stage1.alpha_p = kStage1AlphaP;
  cfg.stage1.alpha_R = kStage1AlphaR;
  cfg.stage1.attitude_stride = 25;
  cfg.stage1.trim_to_observation_support = false;
  cfg.init.sphere_radius_m = 0.10;
  cfg.init.nominal_t_d_L_s = t_d_nominal.t_d_L_s;
  cfg.init.nominal_t_d_C_s = t_d_nominal.t_d_C_s;
  cfg.init.camera_K = {600.0, 600.0, 320.0, 240.0};
  cfg.refine.sphere_radius_m = 0.10;
  cfg.refine.t_d_max_abs_s = spline_cfg.t_d_max_abs_s;
  cfg.refine.lidar_cauchy_scale = 0.0;
  cfg.refine.camera_huber_delta_px = 0.0;
  cfg.refine.camera_K = cfg.init.camera_K;
  cfg.refine.max_iterations = 500;
  cfg.refine.use_centroid_cov_whitening = true;
  return cfg;
}

Eigen::Vector3d TranslationErrorMm(const clic_calib::SE3d& est,
                                   const clic_calib::SE3d& gt) {
  return (est.translation() - gt.translation()) * 1e3;
}

Eigen::Matrix3d SampleCovariance(const std::vector<Eigen::Vector3d>& samples) {
  Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
  if (samples.size() < 2) {
    return cov;
  }
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (const auto& s : samples) {
    mean += s;
  }
  mean /= static_cast<double>(samples.size());
  for (const auto& s : samples) {
    const Eigen::Vector3d d = s - mean;
    cov += d * d.transpose();
  }
  cov /= static_cast<double>(samples.size() - 1);
  return cov;
}

double Pearson1d(const std::vector<double>& a, const std::vector<double>& b) {
  if (a.size() != b.size() || a.size() < 2) {
    return 0.0;
  }
  double ma = 0.0;
  double mb = 0.0;
  for (size_t i = 0; i < a.size(); ++i) {
    ma += a[i];
    mb += b[i];
  }
  ma /= static_cast<double>(a.size());
  mb /= static_cast<double>(b.size());
  double num = 0.0;
  double da = 0.0;
  double db = 0.0;
  for (size_t i = 0; i < a.size(); ++i) {
    const double xa = a[i] - ma;
    const double xb = b[i] - mb;
    num += xa * xb;
    da += xa * xa;
    db += xb * xb;
  }
  const double den = std::sqrt(da * db);
  return den > 1e-12 ? num / den : 0.0;
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

struct DualMcArm {
  std::vector<Eigen::Vector3d> err_ne_mm;
  std::vector<Eigen::Vector3d> err_sw_mm;
  std::vector<Eigen::Vector3d> err_rel_mm;
  std::vector<double> ne_seg_inj_norm_m;
  std::vector<double> sw_seg_inj_norm_m;
  int n_ok = 0;
};

struct DualMcSample {
  Eigen::Vector3d err_ne_mm = Eigen::Vector3d::Zero();
  Eigen::Vector3d err_sw_mm = Eigen::Vector3d::Zero();
  Eigen::Vector3d err_rel_mm = Eigen::Vector3d::Zero();
  double ne_seg_inj_norm_m = 0.0;
  double sw_seg_inj_norm_m = 0.0;
};

bool RunDualMcSample(
    uint32_t traj_seed, uint32_t obs_seed,
    const clic_calib::experiments::DualDiagonalFlightGeometry& geom,
    const clic_calib::experiments::RtkMcInjectionConfig& rtk_inj,
    const clic_calib::LeverArmConfig& levers,
    const clic_calib::NoiseModel& noise,
    const clic_calib::TwoStagePipelineConfig& cfg,
    const clic_calib::two_stage_probe::SplineConfig& spline_cfg,
    const clic_calib::two_stage_probe::CoarseExtrinsicInit& t_d_nominal,
    DualMcSample* out) {
  if (!out) {
    return false;
  }
  auto ds = clic_calib::experiments::BuildDualDiagonalScenario(
      traj_seed, noise, geom, spline_cfg, t_d_nominal, rtk_inj);
  if (obs_seed != traj_seed) {
    ds.body_by_sensor["lidar_NE"] =
        clic_calib::experiments::BuildBodyClusterForDiagonalSensor(
            ds.sc.gt_traj, ds.gt_lidars.T_LW.at("lidar_NE"), ds.sc.gt.t_d_L_s,
            levers, noise, geom, "lidar_NE", 0, obs_seed + 17u, true);
    ds.body_by_sensor["lidar_SW"] =
        clic_calib::experiments::BuildBodyClusterForDiagonalSensor(
            ds.sc.gt_traj, ds.gt_lidars.T_LW.at("lidar_SW"), ds.sc.gt.t_d_L_s,
            levers, noise, geom, "lidar_SW", 1, obs_seed + 29u, true);
  }

  const auto seg_inj =
      clic_calib::experiments::MeanInjectedRtkPerturbationBySector(ds, levers);
  out->ne_seg_inj_norm_m = seg_inj.ne.norm();
  out->sw_seg_inj_norm_m = seg_inj.sw.norm();

  const auto m = clic_calib::experiments::CalibrateDualDiagonalFlightEntry(
      ds, levers, noise, cfg, kObservedMeanIters);
  if (!m.calib.ne.calib.observed_mean.stage2_ok ||
      !m.calib.sw.calib.observed_mean.stage2_ok) {
    return false;
  }

  const clic_calib::SE3d& gt_ne = ds.gt_lidars.T_LW.at("lidar_NE");
  const clic_calib::SE3d& gt_sw = ds.gt_lidars.T_LW.at("lidar_SW");
  const clic_calib::SE3d T_ne = m.calib.ne.calib.T_LW_observed_mean;
  const clic_calib::SE3d T_sw = m.calib.sw.calib.T_LW_observed_mean;

  out->err_ne_mm = TranslationErrorMm(T_ne, gt_ne);
  out->err_sw_mm = TranslationErrorMm(T_sw, gt_sw);
  const clic_calib::SE3d T_rel_est = T_ne * T_sw.inverse();
  const clic_calib::SE3d T_rel_gt = gt_ne * gt_sw.inverse();
  out->err_rel_mm = TranslationErrorMm(T_rel_est, T_rel_gt);
  return true;
}

DualMcArm CollectDualMc(
    int n, uint32_t fixed_obs_seed,
    const clic_calib::experiments::DualDiagonalFlightGeometry& geom,
    const clic_calib::experiments::RtkMcInjectionConfig& rtk_inj,
    const clic_calib::LeverArmConfig& levers,
    const clic_calib::NoiseModel& noise,
    const clic_calib::TwoStagePipelineConfig& cfg,
    const clic_calib::two_stage_probe::SplineConfig& spline_cfg,
    const clic_calib::two_stage_probe::CoarseExtrinsicInit& t_d_nominal) {
  DualMcArm arm;
  for (int i = 0; i < n; ++i) {
    const uint32_t traj_seed = kMcSeedBase + static_cast<uint32_t>(i);
    DualMcSample s;
    if (RunDualMcSample(traj_seed, fixed_obs_seed, geom, rtk_inj, levers,
                        noise, cfg, spline_cfg, t_d_nominal, &s)) {
      arm.err_ne_mm.push_back(s.err_ne_mm);
      arm.err_sw_mm.push_back(s.err_sw_mm);
      arm.err_rel_mm.push_back(s.err_rel_mm);
      arm.ne_seg_inj_norm_m.push_back(s.ne_seg_inj_norm_m);
      arm.sw_seg_inj_norm_m.push_back(s.sw_seg_inj_norm_m);
      ++arm.n_ok;
    }
  }
  return arm;
}

struct CovSummary {
  double rel_over_abs = 0.0;
  double tr_rel = 0.0;
  double tr_abs_mean = 0.0;
};

CovSummary SummarizeCov(const DualMcArm& arm) {
  CovSummary s;
  const Eigen::Matrix3d cov_ne = SampleCovariance(arm.err_ne_mm);
  const Eigen::Matrix3d cov_sw = SampleCovariance(arm.err_sw_mm);
  const Eigen::Matrix3d cov_rel = SampleCovariance(arm.err_rel_mm);
  s.tr_rel = cov_rel.trace();
  s.tr_abs_mean = 0.5 * (cov_ne.trace() + cov_sw.trace());
  s.rel_over_abs = s.tr_rel / std::max(s.tr_abs_mean, 1e-9);
  return s;
}

struct SeedResult {
  uint32_t seed = 0;
  int white_ok = 0;
  int coherent_ok = 0;
  double white_ratio = 0.0;
  double coherent_ratio = 0.0;
  double white_corr = 0.0;
  double coherent_corr = 0.0;
};

std::filesystem::path DataDir() {
  return std::filesystem::path(__FILE__).parent_path().parent_path() /
         "paper/figures/data";
}

}  // namespace

TEST(PaperRelativeExtrinsic, MultiseedRtkPerturbationDichotomy) {
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
  const clic_calib::TwoStagePipelineConfig cfg =
      MakeBaseConfig(spline_cfg, t_d_nominal);
  const auto geom =
      clic_calib::experiments::DiagonalFlightE_Geometry();

  clic_calib::experiments::RtkMcInjectionConfig white_inj;
  white_inj.mode =
      clic_calib::experiments::RtkMcInjectionMode::kWhiteNoisePerSample;

  clic_calib::experiments::RtkMcInjectionConfig coherent_inj;
  coherent_inj.mode =
      clic_calib::experiments::RtkMcInjectionMode::kCoherentSystemBiasOnly;
  coherent_inj.system_bias_sigma_h_m = 5.0 * noise.rtk_sigma_horizontal_m;
  coherent_inj.system_bias_sigma_v_m = 5.0 * noise.rtk_sigma_vertical_m;

  const int n_mc = NumMcDraws();
  const auto csv_path = DataDir() / "relext_multiseed.csv";
  std::filesystem::create_directories(csv_path.parent_path());
  std::ofstream csv(csv_path);
  csv << "seed,mc_draws,white_ok,coherent_ok,white_rel_over_abs,"
         "coherent_rel_over_abs,white_corr,coherent_corr\n";
  csv << std::fixed << std::setprecision(6);

  std::vector<SeedResult> rows;
  std::vector<double> white_ratios;
  std::vector<double> coherent_ratios;
  std::vector<double> white_corrs;
  std::vector<double> coherent_corrs;

  for (int i = 0; i < kNumOuterSeeds; ++i) {
    const uint32_t seed = kSeedBase + static_cast<uint32_t>(i);
    const DualMcArm white_mc = CollectDualMc(
        n_mc, seed, geom, white_inj, levers, noise, cfg, spline_cfg,
        t_d_nominal);
    const DualMcArm coherent_mc = CollectDualMc(
        n_mc, seed, geom, coherent_inj, levers, noise, cfg, spline_cfg,
        t_d_nominal);

    const CovSummary white_cov = SummarizeCov(white_mc);
    const CovSummary coherent_cov = SummarizeCov(coherent_mc);
    SeedResult row;
    row.seed = seed;
    row.white_ok = white_mc.n_ok;
    row.coherent_ok = coherent_mc.n_ok;
    row.white_ratio = white_cov.rel_over_abs;
    row.coherent_ratio = coherent_cov.rel_over_abs;
    row.white_corr =
        Pearson1d(white_mc.ne_seg_inj_norm_m, white_mc.sw_seg_inj_norm_m);
    row.coherent_corr =
        Pearson1d(coherent_mc.ne_seg_inj_norm_m,
                  coherent_mc.sw_seg_inj_norm_m);
    rows.push_back(row);
    white_ratios.push_back(row.white_ratio);
    coherent_ratios.push_back(row.coherent_ratio);
    white_corrs.push_back(row.white_corr);
    coherent_corrs.push_back(row.coherent_corr);

    csv << row.seed << "," << n_mc << "," << row.white_ok << ","
        << row.coherent_ok << "," << row.white_ratio << ","
        << row.coherent_ratio << "," << row.white_corr << ","
        << row.coherent_corr << "\n";

    std::cout << std::fixed << std::setprecision(3)
              << "seed=" << row.seed << " coherent=" << row.coherent_ratio
              << " white=" << row.white_ratio
              << " corr(coh)=" << row.coherent_corr
              << " corr(white)=" << row.white_corr << "\n";
  }

  const Stats white_ratio_stats = ComputeStats(white_ratios);
  const Stats coherent_ratio_stats = ComputeStats(coherent_ratios);
  const Stats white_corr_stats = ComputeStats(white_corrs);
  const Stats coherent_corr_stats = ComputeStats(coherent_corrs);

  std::cout << "\n=== Paper §7.2 relext multiseed (N=" << rows.size()
            << " outer seeds; MC=" << n_mc << ") ===\n";
  std::cout << std::fixed << std::setprecision(4)
            << "  coherent rel/mean(abs): " << coherent_ratio_stats.mean
            << " ± " << coherent_ratio_stats.std << "\n"
            << "  white rel/mean(abs):    " << white_ratio_stats.mean << " ± "
            << white_ratio_stats.std << "\n"
            << "  coherent corr:          " << coherent_corr_stats.mean
            << " ± " << coherent_corr_stats.std << "\n"
            << "  white corr:             " << white_corr_stats.mean << " ± "
            << white_corr_stats.std << "\n"
            << "  csv: " << csv_path << "\n";

  ASSERT_EQ(rows.size(), static_cast<size_t>(kNumOuterSeeds));
  for (const auto& row : rows) {
    EXPECT_GE(row.white_ok, n_mc / 2);
    EXPECT_GE(row.coherent_ok, n_mc / 2);
  }

  const SeedResult& anchor = rows.front();
  EXPECT_EQ(anchor.seed, kSeedBase);
  EXPECT_NEAR(anchor.coherent_ratio, kFrozenCoherentRatio, 0.002)
      << "anchor seed must reproduce frozen §7.2 coherent ratio";
  EXPECT_NEAR(anchor.white_ratio, kFrozenWhiteRatio, 0.002)
      << "anchor seed must reproduce frozen §7.2 white ratio";
}

int main(int argc, char** argv) { return ClicGTestRunAll(argc, argv); }
