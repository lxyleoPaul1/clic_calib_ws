/**
 * STEP 1 — Local pipeline under REALISTIC noise (all modalities), 20-seed sweep.
 *
 * Paper-grade reporting only: no loosened thresholds. Strict GTest limits from
 * test_full_pipeline_synthetic are evaluated post-hoc for comparison.
 *
 * Noise: RTK σ_xy=1 cm, σ_z=2 cm; LiDAR σ_r=2 cm; camera σ_pix=1 px (matches factors).
 * Prior: default yaml (rot σ=5 deg, trans σ=0.5 m).
 */

#include "experiments/noise_regime_common.hpp"
#include "gtest_ceres_guard.hpp"


#include <gtest/gtest.h>

#include <iostream>
#include <map>
#include <string>

namespace {

constexpr int kNumSeeds = 20;
constexpr uint32_t kSeedBase = 1000;

struct StrictGateCounts {
  int rot_LW_pass = 0;
  int rot_CW_pass = 0;
  int trans_LW_pass = 0;
  int trans_CW_pass = 0;
  int td_L_pass = 0;
  int td_C_pass = 0;
  int rms_pass = 0;
};

void AccumulateStrictGate(const clic_calib::experiments::CalibrationRunMetrics& m,
                          StrictGateCounts* gate) {
  if (m.rot_LW_deg < 0.5) {
    ++gate->rot_LW_pass;
  }
  if (m.rot_CW_deg < 0.3) {
    ++gate->rot_CW_pass;
  }
  if (m.trans_LW_norm_mm < 50.0) {
    ++gate->trans_LW_pass;
  }
  if (m.trans_CW_norm_mm < 30.0) {
    ++gate->trans_CW_pass;
  }
  if (std::abs(m.t_d_L_err_ms) < 2.0) {
    ++gate->td_L_pass;
  }
  if (std::abs(m.t_d_C_err_ms) < 2.0) {
    ++gate->td_C_pass;
  }
  if (m.traj_rms_mm < 30.0) {
    ++gate->rms_pass;
  }
}

}  // namespace

TEST(NoiseRegimeLocal, RealisticNoiseSeedSweepWithFullMetrics) {
  using namespace clic_calib::experiments;

  const RealisticNoiseSpec noise =
      clic_calib::NoiseModel::FromConfigDir(ConfigDirFromExperiments());
  std::map<std::string, RunningStats> stats;
  StrictGateCounts strict_gate;

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
    stats["trans_CW_norm_mm"].Push(m.trans_CW_norm_mm);
    stats["t_d_L_err_ms"].Push(m.t_d_L_err_ms);
    stats["t_d_C_err_ms"].Push(m.t_d_C_err_ms);
    stats["traj_rms_mm"].Push(m.traj_rms_mm);
    stats["post_pitch_LW_rad"].Push(m.post.pitch_LW_rad);
    stats["post_tz_LW_mm"].Push(m.post.tz_LW_m * 1e3);

    AccumulateStrictGate(m, &strict_gate);

    std::cout << "[seed " << seed << "] rot_LW=" << m.rot_LW_deg
              << " deg, rot_CW=" << m.rot_CW_deg
              << " deg, pitch_err=" << m.pitch_LW_err_deg
              << " deg, |dT_LW|=" << m.trans_LW_norm_mm
              << " mm, t_d_L_err=" << m.t_d_L_err_ms
              << " ms, traj_rms=" << m.traj_rms_mm
              << " mm, post_σ_pitch=" << (m.post.pitch_LW_rad * 1000.0)
              << " mrad, post_σ_tz=" << (m.post.tz_LW_m * 1e3) << " mm\n";
  }

  std::cout << "\n=== LOCAL REALISTIC NOISE (N=" << kNumSeeds
            << ", seeds " << kSeedBase << ".." << (kSeedBase + kNumSeeds - 1)
            << ") mean ± std ===\n";
  std::cout << "  rot_LW:           " << FormatMeanStdDeg(stats["rot_LW_deg"]) << "\n";
  std::cout << "  rot_CW:           " << FormatMeanStdDeg(stats["rot_CW_deg"]) << "\n";
  std::cout << "  pitch_LW_err:     " << FormatMeanStdDeg(stats["pitch_LW_err_deg"]) << "\n";
  std::cout << "  trans_LW_x:       " << FormatMeanStdMm(stats["trans_LW_x_mm"]) << "\n";
  std::cout << "  trans_LW_y:       " << FormatMeanStdMm(stats["trans_LW_y_mm"]) << "\n";
  std::cout << "  trans_LW_z:       " << FormatMeanStdMm(stats["trans_LW_z_mm"]) << "\n";
  std::cout << "  |trans_LW|:       " << FormatMeanStdMm(stats["trans_LW_norm_mm"]) << "\n";
  std::cout << "  |trans_CW|:       " << FormatMeanStdMm(stats["trans_CW_norm_mm"]) << "\n";
  std::cout << "  t_d_L_err:        " << FormatMeanStdMs(stats["t_d_L_err_ms"]) << "\n";
  std::cout << "  t_d_C_err:        " << FormatMeanStdMs(stats["t_d_C_err_ms"]) << "\n";
  std::cout << "  traj_RMS:         " << FormatMeanStdMm(stats["traj_rms_mm"]) << "\n";
  std::cout << "  post σ_pitch_LW:  " << FormatMeanStdMrad(stats["post_pitch_LW_rad"]) << "\n";
  std::cout << "  post σ_tz_LW:     " << FormatMeanStdMm(stats["post_tz_LW_mm"]) << "\n";

  std::cout << "\n=== Strict gate pass rate (NOT asserted — diagnostic only) ===\n";
  std::cout << "  rot_LW < 0.5 deg:  " << strict_gate.rot_LW_pass << "/" << kNumSeeds << "\n";
  std::cout << "  rot_CW < 0.3 deg:  " << strict_gate.rot_CW_pass << "/" << kNumSeeds << "\n";
  std::cout << "  |T_LW| < 5 cm:     " << strict_gate.trans_LW_pass << "/" << kNumSeeds << "\n";
  std::cout << "  |T_CW| < 3 cm:     " << strict_gate.trans_CW_pass << "/" << kNumSeeds << "\n";
  std::cout << "  t_d_L ± 2 ms:      " << strict_gate.td_L_pass << "/" << kNumSeeds << "\n";
  std::cout << "  t_d_C ± 2 ms:      " << strict_gate.td_C_pass << "/" << kNumSeeds << "\n";
  std::cout << "  traj RMS < 3 cm:   " << strict_gate.rms_pass << "/" << kNumSeeds << "\n";

  SUCCEED();
}

int main(int argc, char** argv) { return ClicGTestRunAll(argc, argv); }
