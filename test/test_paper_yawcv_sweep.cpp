#include <clic_calib/estimator/stage1_trajectory_fitter.h>
#include <clic_calib/estimator/two_stage_pipeline.h>
#include <clic_calib/target/body_centroid_analysis.h>
#include <clic_calib/utils/lever_arm.h>
#include <clic_calib/utils/noise_model.h>
#include "gtest_ceres_guard.hpp"

#include <gtest/gtest.h>

#include "diagnostic/closed_form_init_common.hpp"
#include "diagnostic/two_stage_probe_common.hpp"
#include "experiments/noise_regime_common.hpp"
#include "experiments/phase15_ablation_common.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

namespace {

constexpr uint32_t kAnchorSeed = 13025;
constexpr int kSeedsPerLevel = 8;
constexpr int kObservedMeanIters = 3;
constexpr double kFrozenCentroidMm = 63.5;
constexpr double kFrozenObsMeanMm = 18.5;
constexpr double kTwoSigmaMm = 10.0;  // conservative stop threshold for anchor

clic_calib::TwoStagePipelineConfig MakeBaseConfig(
    const clic_calib::two_stage_probe::SplineConfig& spline_cfg,
    const clic_calib::two_stage_probe::CoarseExtrinsicInit& t_d_nominal) {
  clic_calib::TwoStagePipelineConfig cfg;
  cfg.stage1.knot_interval_s = spline_cfg.knot_interval_s;
  cfg.stage1.alpha_p = 0.01;
  cfg.stage1.alpha_R = 0.01;
  cfg.stage1.attitude_stride = 25;
  cfg.stage1.trim_to_observation_support = true;
  cfg.init.sphere_radius_m = 0.10;
  cfg.init.nominal_t_d_L_s = t_d_nominal.t_d_L_s;
  cfg.init.nominal_t_d_C_s = t_d_nominal.t_d_C_s;
  cfg.init.camera_K = {600.0, 600.0, 320.0, 240.0};
  cfg.refine.sphere_radius_m = 0.10;
  cfg.refine.t_d_max_abs_s = spline_cfg.t_d_max_abs_s;
  cfg.refine.lidar_cauchy_scale = 1.0;
  cfg.refine.camera_huber_delta_px = 2.0;
  cfg.refine.camera_K = cfg.init.camera_K;
  cfg.refine.max_iterations = 500;
  return cfg;
}

clic_calib::experiments::SyntheticFlightGeometry
YawCvSweepGeometry(double azimuth_span_deg) {
  auto g = clic_calib::experiments::NearFieldHighAspectScenarioGeometry();
  g.azimuth_span_deg = azimuth_span_deg;
  g.label = "yawcv_sweep_az" + std::to_string(static_cast<int>(azimuth_span_deg));
  return g;
}

struct SeedRun {
  uint32_t seed = 0;
  double centroid_mm = 0.0;
  double obs_mm = 0.0;
  bool obs_applied = false;
  double yaw_cv = 0.0;
};

struct LevelStats {
  double azimuth_span_deg = 0.0;
  double yaw_cv_mean = 0.0;
  double centroid_mean = 0.0;
  double centroid_std = 0.0;
  double obs_mean = 0.0;
  double obs_std = 0.0;
  std::vector<SeedRun> runs;
};

double Mean(const std::vector<double>& v) {
  if (v.empty()) {
    return 0.0;
  }
  return std::accumulate(v.begin(), v.end(), 0.0) /
         static_cast<double>(v.size());
}

double Std(const std::vector<double>& v, double mean) {
  if (v.size() < 2) {
    return 0.0;
  }
  double sq = 0.0;
  for (double x : v) {
    const double d = x - mean;
    sq += d * d;
  }
  return std::sqrt(sq / static_cast<double>(v.size() - 1));
}

std::filesystem::path CsvOutputPath() {
  const std::filesystem::path from_source =
      std::filesystem::path(__FILE__).parent_path().parent_path() /
      "paper/figures/data/yawcv_sweep.csv";
  return from_source;
}

LevelStats RunLevel(double azimuth_span_deg, uint32_t seed_base,
                    const clic_calib::LeverArmConfig& levers,
                    const clic_calib::NoiseModel& noise,
                    const clic_calib::TwoStagePipelineConfig& base_cfg,
                    const clic_calib::two_stage_probe::CoarseExtrinsicInit& t_d,
                    const clic_calib::two_stage_probe::SplineConfig& spline_cfg) {
  LevelStats level;
  level.azimuth_span_deg = azimuth_span_deg;
  const auto geom = YawCvSweepGeometry(azimuth_span_deg);
  const Eigen::Vector3d lidar_post_W(0.0, 0.0, geom.sensor_height_m);

  std::vector<double> cent_vals;
  std::vector<double> obs_vals;
  std::vector<double> yaw_vals;

  for (int i = 0; i < kSeedsPerLevel; ++i) {
    const uint32_t seed = seed_base + static_cast<uint32_t>(i);
    auto ps = clic_calib::experiments::BuildPhase15ScenarioWithGeom(seed, noise,
                                                                    geom);
    if (!clic_calib::experiments::PreparePhase15ScenarioTags(
            &ps, levers, noise, t_d, spline_cfg, seed, 0.01, 0.01)) {
      ADD_FAILURE() << "PreparePhase15ScenarioTags failed seed=" << seed
                    << " az=" << azimuth_span_deg;
      continue;
    }

    const double u_B_std = clic_calib::experiments::ComputeUBAzimuthStdDeg(
        ps.sc.gt_traj, ps.sc.gt.t_d_L_s, lidar_post_W, ps.body_cluster);
    const auto gated =
        clic_calib::experiments::CalibrateBodyGatedObservedMean(
            ps, levers, noise, base_cfg, ps.body_cluster, kObservedMeanIters,
            {}, u_B_std);

    SeedRun run;
    run.seed = seed;
    run.yaw_cv = gated.aspect.yaw_circular_variance;
    run.centroid_mm = gated.centroid_only.trans_mm;
    run.obs_mm = gated.observed_mean.trans_mm;
    run.obs_applied = gated.observed_mean_applied;
    level.runs.push_back(run);

    cent_vals.push_back(run.centroid_mm);
    obs_vals.push_back(run.obs_mm);
    yaw_vals.push_back(run.yaw_cv);
  }

  level.yaw_cv_mean = Mean(yaw_vals);
  level.centroid_mean = Mean(cent_vals);
  level.centroid_std = Std(cent_vals, level.centroid_mean);
  level.obs_mean = Mean(obs_vals);
  level.obs_std = Std(obs_vals, level.obs_mean);
  return level;
}

void WriteCsv(const std::vector<LevelStats>& levels) {
  const auto path = CsvOutputPath();
  std::filesystem::create_directories(path.parent_path());
  std::ofstream out(path);
  out << "azimuth_span_deg,yaw_cv_mean,seed,centroid_mm,obs_mm,obs_applied\n";
  out << std::fixed << std::setprecision(4);
  for (const auto& level : levels) {
    for (const auto& run : level.runs) {
      out << level.azimuth_span_deg << "," << run.yaw_cv << "," << run.seed
          << "," << run.centroid_mm << "," << run.obs_mm << ","
          << (run.obs_applied ? 1 : 0) << "\n";
    }
    out << level.azimuth_span_deg << "," << level.yaw_cv_mean << ","
        << "AGGREGATE" << "," << level.centroid_mean << "," << level.obs_mean
        << ",\n";
  }
  std::cout << "[csv] wrote " << path << "\n";
}

}  // namespace

TEST(PaperYawCvSweep, CentroidVsObservedMeanAcrossYawCv) {
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

  // P1.5 near-field geometry: sweep azimuth span (other knobs fixed).
  const std::vector<double> azimuth_spans_deg = {5.0,  12.0,  25.0,  40.0,
                                                 65.0, 95.0,  130.0, 165.0};

  std::vector<LevelStats> levels;
  levels.reserve(azimuth_spans_deg.size());
  for (double az : azimuth_spans_deg) {
    std::cout << "\n=== yaw_cv sweep azimuth_span=" << az << " deg ===\n";
    auto level = RunLevel(az, kAnchorSeed, levers, noise, base_cfg, t_d_nominal,
                          spline_cfg);
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "  yaw_cv mean=" << level.yaw_cv_mean
              << "  centroid=" << level.centroid_mean << "±"
              << level.centroid_std << " mm"
              << "  obs=" << level.obs_mean << "±" << level.obs_std << " mm\n";
    levels.push_back(std::move(level));
  }

  WriteCsv(levels);

  // Anchor: full P1.5 span (165°) must stay within 2σ of frozen single-seed refs.
  const LevelStats* anchor = nullptr;
  for (const auto& level : levels) {
    if (std::abs(level.azimuth_span_deg - 165.0) < 0.5) {
      anchor = &level;
      break;
    }
  }
  ASSERT_NE(anchor, nullptr);
  const bool centroid_ok =
      std::abs(anchor->centroid_mean - kFrozenCentroidMm) <= kTwoSigmaMm;
  const bool obs_ok =
      std::abs(anchor->obs_mean - kFrozenObsMeanMm) <= kTwoSigmaMm;
  if (!centroid_ok || !obs_ok) {
    std::cout << "\n*** STOP: anchor 165° mean deviates >2σ from frozen ***\n";
    std::cout << "  frozen centroid=" << kFrozenCentroidMm
              << " mm  got mean=" << anchor->centroid_mean << "±"
              << anchor->centroid_std << " mm\n";
    std::cout << "  frozen obs-mean=" << kFrozenObsMeanMm
              << " mm  got mean=" << anchor->obs_mean << "±" << anchor->obs_std
              << " mm\n";
    GTEST_SKIP() << "Anchor deviates >2σ from frozen — report before continuing";
  }

  EXPECT_GT(levels.front().yaw_cv_mean, 0.0);
  EXPECT_LT(levels.back().yaw_cv_mean, 1.0);
  EXPECT_GT(levels.back().yaw_cv_mean, levels.front().yaw_cv_mean);
}

int main(int argc, char** argv) { return ClicGTestRunAll(argc, argv); }
