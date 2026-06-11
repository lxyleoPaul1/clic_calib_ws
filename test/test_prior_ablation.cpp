/**
 * STEP 4 — Prior ablation: geometric observability vs extrinsic prior dependence.
 *
 * (A) Default prior 5 deg / 0.5 m
 * (B) Weak prior 90 deg / 100 m
 * (C) No ExtrinsicPriorFactor (RTK-only gauge)
 */

#include "experiments/noise_regime_common.hpp"
#include "gtest_ceres_guard.hpp"


#include <clic_calib/estimator/observability_analyzer.h>

#include <gtest/gtest.h>

#include <cmath>
#include <iostream>
#include <map>
#include <string>

namespace {

constexpr int kNumSeeds = 20;
constexpr uint32_t kSeedBase = 3000;

using clic_calib::experiments::BuildMultiLayerNoisyScenario;
using clic_calib::experiments::ConfigDirFromExperiments;
using clic_calib::experiments::FormatMeanStdMax;
using clic_calib::experiments::PitchFromSO3Rad;
using clic_calib::experiments::RealisticNoiseSpec;
using clic_calib::experiments::RotationErrorDeg;
using clic_calib::experiments::RunningStats;
using clic_calib::experiments::SyntheticScenarioBundle;

enum class PriorMode { kDefault, kWeak, kNone };

const char* PriorModeName(PriorMode mode) {
  switch (mode) {
    case PriorMode::kDefault:
      return "A_default_5deg_0p5m";
    case PriorMode::kWeak:
      return "B_weak_90deg_100m";
    case PriorMode::kNone:
      return "C_no_prior_rtk_gauge";
  }
  return "unknown";
}

void ApplyPriorMode(clic_calib::CalibrationEstimator* estimator, PriorMode mode) {
  switch (mode) {
    case PriorMode::kDefault:
      break;
    case PriorMode::kWeak:
      estimator->set_extrinsic_prior_std(90.0, 100.0);
      break;
    case PriorMode::kNone:
      estimator->set_extrinsic_prior_enabled(false);
      break;
  }
}

struct SeedRunResult {
  bool converged = false;
  double rot_LW_deg = 0.0;
  double rot_CW_deg = 0.0;
  double pitch_err_deg = 0.0;
  double trans_LW_mm = 0.0;
  double trans_CW_mm = 0.0;
  double t_d_L_err_ms = 0.0;
  double t_d_C_err_ms = 0.0;
  double lambda_min = 0.0;
  double worst_pitch_dom = 0.0;
  double worst_tz_dom = 0.0;
};

SeedRunResult RunSeed(const SyntheticScenarioBundle& scenario, PriorMode mode) {
  clic_calib::CalibrationEstimator estimator(ConfigDirFromExperiments());
  ApplyPriorMode(&estimator, mode);
  estimator.add_rtk_measurements(scenario.rtk);
  estimator.add_lidar_target_observations(0, scenario.lidar_obs);
  estimator.add_apriltag_observations(0, scenario.tag_obs);

  SeedRunResult out;
  try {
    const ceres::Solver::Summary summary = estimator.solve(1000);
    out.converged = summary.IsSolutionUsable();
    if (!out.converged) {
      return out;
    }

    const clic_calib::SE3d T_LW = estimator.get_T_LW(0);
    const clic_calib::SE3d T_CW = estimator.get_T_CW(0);
    out.rot_LW_deg = RotationErrorDeg(T_LW, scenario.gt.T_LW);
    out.rot_CW_deg = RotationErrorDeg(T_CW, scenario.gt.T_CW);
    const double pitch_est = PitchFromSO3Rad(T_LW.so3()) * 180.0 / M_PI;
    const double pitch_gt = PitchFromSO3Rad(scenario.gt.T_LW.so3()) * 180.0 / M_PI;
    out.pitch_err_deg = pitch_est - pitch_gt;
    out.trans_LW_mm = (T_LW.translation() - scenario.gt.T_LW.translation()).norm() * 1e3;
    out.trans_CW_mm = (T_CW.translation() - scenario.gt.T_CW.translation()).norm() * 1e3;
    out.t_d_L_err_ms = (estimator.get_t_d_lidar(0) - scenario.gt.t_d_L_s) * 1e3;
    out.t_d_C_err_ms = (estimator.get_t_d_camera(0) - scenario.gt.t_d_C_s) * 1e3;

    estimator.build_problem_for_analysis();
    clic_calib::ObservabilityAnalyzer analyzer;
    const clic_calib::ObservabilityReport report = analyzer.analyze(estimator);
    const clic_calib::AnalysisParameterLayout layout =
        estimator.analysis_parameter_layout();
    out.lambda_min = report.lambda_min;
    if (layout.lidar_rot_fext_indices.size() == 3) {
      out.worst_pitch_dom =
          std::abs(report.worst_eigenvector(layout.lidar_rot_fext_indices[1]));
    }
    if (layout.lidar_trans_fext_indices.size() == 3) {
      out.worst_tz_dom =
          std::abs(report.worst_eigenvector(layout.lidar_trans_fext_indices[2]));
    }
  } catch (const std::exception& e) {
    std::cerr << "[prior_ablation] exception: " << e.what() << "\n";
    out.converged = false;
  }
  return out;
}

struct ConfigAggregate {
  int success_count = 0;
  RunningStats rot_LW_deg;
  RunningStats rot_CW_deg;
  RunningStats pitch_err_deg;
  RunningStats trans_LW_mm;
  RunningStats trans_CW_mm;
  RunningStats t_d_L_err_ms;
  RunningStats t_d_C_err_ms;
  RunningStats lambda_min;
  RunningStats worst_pitch_dom;
  RunningStats worst_tz_dom;
};

ConfigAggregate RunConfig(PriorMode mode, const RealisticNoiseSpec& noise) {
  ConfigAggregate agg;
  for (int i = 0; i < kNumSeeds; ++i) {
    const uint32_t seed = kSeedBase + static_cast<uint32_t>(i);
    const SyntheticScenarioBundle scenario = BuildMultiLayerNoisyScenario(seed, noise);
    const SeedRunResult r = RunSeed(scenario, mode);
    if (r.converged) {
      ++agg.success_count;
      agg.rot_LW_deg.Push(r.rot_LW_deg);
      agg.rot_CW_deg.Push(r.rot_CW_deg);
      agg.pitch_err_deg.Push(r.pitch_err_deg);
      agg.trans_LW_mm.Push(r.trans_LW_mm);
      agg.trans_CW_mm.Push(r.trans_CW_mm);
      agg.t_d_L_err_ms.Push(r.t_d_L_err_ms);
      agg.t_d_C_err_ms.Push(r.t_d_C_err_ms);
      agg.lambda_min.Push(r.lambda_min);
      agg.worst_pitch_dom.Push(r.worst_pitch_dom);
      agg.worst_tz_dom.Push(r.worst_tz_dom);
    }
    std::cout << "[" << PriorModeName(mode) << " seed " << seed << "] conv="
              << r.converged << " rot_LW=" << r.rot_LW_deg
              << " pitch_err=" << r.pitch_err_deg << " |T_LW|=" << r.trans_LW_mm
              << " mm t_d_L=" << r.t_d_L_err_ms << " ms λ_min=" << r.lambda_min
              << " worst(pitch,z)=(" << r.worst_pitch_dom << ","
              << r.worst_tz_dom << ")\n";
  }
  return agg;
}

void PrintConfigSummary(PriorMode mode, const ConfigAggregate& agg) {
  std::cout << "\n=== " << PriorModeName(mode) << " (N=" << kNumSeeds
            << ", success=" << agg.success_count << "/" << kNumSeeds << ") ===\n";
  if (agg.success_count == 0) {
    std::cout << "  (no converged runs)\n";
    return;
  }
  std::cout << "  rot_LW:      " << FormatMeanStdMax(agg.rot_LW_deg, "deg") << "\n";
  std::cout << "  rot_CW:      " << FormatMeanStdMax(agg.rot_CW_deg, "deg") << "\n";
  std::cout << "  pitch_err:   " << FormatMeanStdMax(agg.pitch_err_deg, "deg") << "\n";
  std::cout << "  |T_LW|:      " << FormatMeanStdMax(agg.trans_LW_mm, "mm") << "\n";
  std::cout << "  |T_CW|:      " << FormatMeanStdMax(agg.trans_CW_mm, "mm") << "\n";
  std::cout << "  t_d_L_err:   " << FormatMeanStdMax(agg.t_d_L_err_ms, "ms") << "\n";
  std::cout << "  t_d_C_err:   " << FormatMeanStdMax(agg.t_d_C_err_ms, "ms") << "\n";
  std::cout << "  λ_min(F_ext):" << FormatMeanStdMax(agg.lambda_min, "") << "\n";
  std::cout << "  worst pitch: " << FormatMeanStdMax(agg.worst_pitch_dom, "") << "\n";
  std::cout << "  worst tz:    " << FormatMeanStdMax(agg.worst_tz_dom, "") << "\n";
}

}  // namespace

TEST(PriorAblation, MultiLayerGeometricObservabilityVsPrior) {
  const RealisticNoiseSpec noise =
      clic_calib::NoiseModel::FromConfigDir(ConfigDirFromExperiments());

  const ConfigAggregate cfg_a = RunConfig(PriorMode::kDefault, noise);
  const ConfigAggregate cfg_b = RunConfig(PriorMode::kWeak, noise);
  const ConfigAggregate cfg_c = RunConfig(PriorMode::kNone, noise);

  PrintConfigSummary(PriorMode::kDefault, cfg_a);
  PrintConfigSummary(PriorMode::kWeak, cfg_b);
  PrintConfigSummary(PriorMode::kNone, cfg_c);

  // Diagnostic only — GATE 4 is interpreted from printed tables, not hard pass/fail.
  SUCCEED();
}

int main(int argc, char** argv) { return ClicGTestRunAll(argc, argv); }
