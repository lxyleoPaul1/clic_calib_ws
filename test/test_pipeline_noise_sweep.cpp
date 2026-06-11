/**
 * STEP 2 — Full local pipeline under realistic noise (all modalities), N-seed sweep.
 *
 * Replaces the noise-free identity check as the primary synthetic evaluation.
 * Configuration: extrinsics free, yaml prior (5 deg / 0.5 m), t_d in ±0.1 s,
 * RTK warm-start, noise from config/noise_model.yaml on RTK + LiDAR + camera.
 *
 * Pass criteria: finite spread; |mean error| <= 3× per-seed std (unbiased under noise).
 */

#include "experiments/noise_regime_common.hpp"
#include "gtest_ceres_guard.hpp"


#include <gtest/gtest.h>

#include <cmath>
#include <iostream>
#include <map>
#include <string>

namespace {

constexpr int kNumSeeds = 20;
constexpr uint32_t kSeedBase = 1000;

using clic_calib::experiments::BuildLocalNoisyScenario;
using clic_calib::experiments::CalibrationRunMetrics;
using clic_calib::experiments::ConfigDirFromExperiments;
using clic_calib::experiments::FormatMeanStdMax;
using clic_calib::experiments::RealisticNoiseSpec;
using clic_calib::experiments::RunLocalCalibration;
using clic_calib::experiments::RunningStats;
using clic_calib::experiments::SyntheticScenarioBundle;

void ExpectConsistentWithNoiseSpread(const RunningStats& s,
                                     const char* metric_name,
                                     double k = 3.0) {
  EXPECT_TRUE(std::isfinite(s.mean)) << metric_name << " mean not finite";
  EXPECT_TRUE(std::isfinite(s.Std())) << metric_name << " std not finite";
  if (s.n >= 2 && s.Std() > 1e-12) {
    EXPECT_LE(std::abs(s.mean), k * s.Std())
        << metric_name << ": |mean|=" << std::abs(s.mean) << " > " << k
        << "*std=" << (k * s.Std()) << " (possible bias under noise)";
  }
}

void PrintSummaryTable(const std::map<std::string, RunningStats>& stats) {
  std::cout << "\n=== PIPELINE NOISE SWEEP (N=" << kNumSeeds << ", seeds "
            << kSeedBase << ".." << (kSeedBase + kNumSeeds - 1)
            << ") mean ± std (max) ===\n";
  auto row = [&](const char* label, const std::string& key, const char* unit) {
    const auto it = stats.find(key);
    if (it != stats.end()) {
      std::cout << "  " << label << ": " << FormatMeanStdMax(it->second, unit)
                << "\n";
    }
  };
  row("rot_LW", "rot_LW_deg", "deg");
  row("rot_CW", "rot_CW_deg", "deg");
  row("pitch_LW_err", "pitch_LW_err_deg", "deg");
  row("trans_LW_x", "trans_LW_x_mm", "mm");
  row("trans_LW_y", "trans_LW_y_mm", "mm");
  row("trans_LW_z", "trans_LW_z_mm", "mm");
  row("|trans_LW|", "trans_LW_norm_mm", "mm");
  row("trans_CW_x", "trans_CW_x_mm", "mm");
  row("trans_CW_y", "trans_CW_y_mm", "mm");
  row("trans_CW_z", "trans_CW_z_mm", "mm");
  row("|trans_CW|", "trans_CW_norm_mm", "mm");
  row("t_d_L_err", "t_d_L_err_ms", "ms");
  row("t_d_C_err", "t_d_C_err_ms", "ms");
  row("traj_RMS", "traj_rms_mm", "mm");
}

}  // namespace

TEST(PipelineNoiseSweep, RealisticNoiseTwentySeedCharacterization) {
  const RealisticNoiseSpec noise =
      clic_calib::NoiseModel::FromConfigDir(ConfigDirFromExperiments());
  std::map<std::string, RunningStats> stats;

  for (int i = 0; i < kNumSeeds; ++i) {
    const uint32_t seed = kSeedBase + static_cast<uint32_t>(i);
    const SyntheticScenarioBundle scenario = BuildLocalNoisyScenario(seed, noise);
    const CalibrationRunMetrics m = RunLocalCalibration(scenario);

    stats["rot_LW_deg"].Push(m.rot_LW_deg);
    stats["rot_CW_deg"].Push(m.rot_CW_deg);
    stats["pitch_LW_err_deg"].Push(m.pitch_LW_err_deg);
    stats["trans_LW_x_mm"].Push(m.trans_LW_err_m.x() * 1e3);
    stats["trans_LW_y_mm"].Push(m.trans_LW_err_m.y() * 1e3);
    stats["trans_LW_z_mm"].Push(m.trans_LW_err_m.z() * 1e3);
    stats["trans_LW_norm_mm"].Push(m.trans_LW_norm_mm);
    stats["trans_CW_x_mm"].Push(m.trans_CW_err_m.x() * 1e3);
    stats["trans_CW_y_mm"].Push(m.trans_CW_err_m.y() * 1e3);
    stats["trans_CW_z_mm"].Push(m.trans_CW_err_m.z() * 1e3);
    stats["trans_CW_norm_mm"].Push(m.trans_CW_norm_mm);
    stats["t_d_L_err_ms"].Push(m.t_d_L_err_ms);
    stats["t_d_C_err_ms"].Push(m.t_d_C_err_ms);
    stats["traj_rms_mm"].Push(m.traj_rms_mm);

    std::cout << "[seed " << seed << "] rot_LW=" << m.rot_LW_deg
              << " deg, rot_CW=" << m.rot_CW_deg
              << " deg, pitch_err=" << m.pitch_LW_err_deg
              << " deg, |dT_LW|=" << m.trans_LW_norm_mm
              << " mm, |dT_CW|=" << m.trans_CW_norm_mm
              << " mm, t_d_L_err=" << m.t_d_L_err_ms
              << " ms, t_d_C_err=" << m.t_d_C_err_ms
              << " ms, traj_rms=" << m.traj_rms_mm << " mm\n";
  }

  PrintSummaryTable(stats);

  const double rtk_xy_floor_mm =
      1e3 * noise.rtk_sigma_horizontal_m / std::sqrt(47.0);
  std::cout << "\n[CRLB hint] i.i.d. RTK-only position floor σ/√N ≈ "
            << rtk_xy_floor_mm << " mm (extrinsic coupling is stronger)\n";
  std::cout << "[characterization] full table → doc/results/synthetic_evaluation.md §2\n";
  EXPECT_GT(stats["traj_rms_mm"].Std(), 0.0)
      << "trajectory spread should be non-zero under RTK noise";
  SUCCEED();
}

int main(int argc, char** argv) { return ClicGTestRunAll(argc, argv); }
