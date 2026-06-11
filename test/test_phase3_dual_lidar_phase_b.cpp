/**
 * Phase 3 (B) — multi-LiDAR-only metrics:
 * (a) RTK perturbation source matters: coherent system bias vs white noise.
 * (b) Center registration as primary relative metric.
 * (2) Serial-sector perturbation correlation diagnostic.
 * (3) Temporal overlap variant when serial breaks common-mode visibility.
 */
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

#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <vector>

namespace {

constexpr uint32_t kRepSeed = 13025;
constexpr uint32_t kMcSeedBase = 13000;
constexpr double kStage1AlphaP = 0.01;
constexpr double kStage1AlphaR = 0.01;
constexpr int kObservedMeanIters = 3;
constexpr double kBaselineM = 70.7;

int NumMcSeeds() {
  if (const char* env = std::getenv("CLIC_PHASE_B_MC_N")) {
    return std::max(10, std::atoi(env));
  }
  return 20;
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

double TraceMm2(const Eigen::Matrix3d& cov) { return cov.trace(); }

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
    int n, uint32_t fixed_traj_seed, uint32_t fixed_obs_seed,
    const clic_calib::experiments::DualDiagonalFlightGeometry& geom,
    const clic_calib::experiments::RtkMcInjectionConfig& rtk_inj,
    const clic_calib::LeverArmConfig& levers,
    const clic_calib::NoiseModel& noise,
    const clic_calib::TwoStagePipelineConfig& cfg,
    const clic_calib::two_stage_probe::SplineConfig& spline_cfg,
    const clic_calib::two_stage_probe::CoarseExtrinsicInit& t_d_nominal) {
  DualMcArm arm;
  for (int i = 0; i < n; ++i) {
    const uint32_t varying = kMcSeedBase + static_cast<uint32_t>(i);
    const uint32_t traj_seed =
        fixed_traj_seed != 0 ? fixed_traj_seed : varying;
    const uint32_t obs_seed = fixed_obs_seed != 0 ? fixed_obs_seed : varying;
    DualMcSample s;
    if (RunDualMcSample(traj_seed, obs_seed, geom, rtk_inj, levers, noise, cfg,
                        spline_cfg, t_d_nominal, &s)) {
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

double Pearson1d(const std::vector<double>& a, const std::vector<double>& b) {
  if (a.size() != b.size() || a.size() < 2) {
    return 0.0;
  }
  const size_t n = a.size();
  double ma = 0.0, mb = 0.0;
  for (size_t i = 0; i < n; ++i) {
    ma += a[i];
    mb += b[i];
  }
  ma /= static_cast<double>(n);
  mb /= static_cast<double>(n);
  double num = 0.0, da = 0.0, db = 0.0;
  for (size_t i = 0; i < n; ++i) {
    const double xa = a[i] - ma;
    const double xb = b[i] - mb;
    num += xa * xb;
    da += xa * xa;
    db += xb * xb;
  }
  const double den = std::sqrt(da * db);
  return den > 1e-12 ? num / den : 0.0;
}

struct CovSummary {
  double tr_ne = 0.0;
  double tr_sw = 0.0;
  double tr_rel = 0.0;
  double tr_abs_mean = 0.0;
  double rel_over_abs = 0.0;
};

CovSummary SummarizeCov(const DualMcArm& arm) {
  CovSummary s;
  s.tr_ne = TraceMm2(SampleCovariance(arm.err_ne_mm));
  s.tr_sw = TraceMm2(SampleCovariance(arm.err_sw_mm));
  s.tr_rel = TraceMm2(SampleCovariance(arm.err_rel_mm));
  s.tr_abs_mean = 0.5 * (s.tr_ne + s.tr_sw);
  s.rel_over_abs = s.tr_rel / std::max(s.tr_abs_mean, 1e-9);
  return s;
}

void PrintCovBlock(const char* label, const DualMcArm& arm) {
  const CovSummary s = SummarizeCov(arm);
  std::cout << std::fixed << std::setprecision(3);
  std::cout << "  [" << label << "] N=" << arm.n_ok << "\n";
  std::cout << "    trace Cov(δt_NE)=" << s.tr_ne << " mm²  Cov(δt_SW)=" << s.tr_sw
            << " mm²  Cov(δt_rel)=" << s.tr_rel << " mm²\n";
  std::cout << "    trace ratio rel/mean(abs)=" << s.rel_over_abs << "\n";
}

}  // namespace

TEST(Phase3DualLidarPhaseB, RelativeExtrinsicMcAndCenterRegPrimary) {
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

  const auto ds_rep = clic_calib::experiments::BuildDualDiagonalScenario(
      kRepSeed, noise, geom, spline_cfg, t_d_nominal);
  const auto rep = clic_calib::experiments::CalibrateDualDiagonalFlightEntry(
      ds_rep, levers, noise, cfg, kObservedMeanIters);

  std::cout << "\n### Phase 3 (B): RTK source + serial-sector MC ###\n";

  // --- (b) Center registration primary @ rep ---
  const double rel_trans_mm = rep.calib.rel_observed.trans_mm;
  const double rel_rot_deg = rep.calib.rel_observed.rot_deg;
  const double center_reg_mm = rep.center_reg_obs_mm;

  std::cout << "\n=== (b) Relative metric @ flight 戊 0.5 Hz seed " << kRepSeed
            << " ===\n";
  std::cout << std::fixed << std::setprecision(2);
  std::cout << "  center-reg (obs):    " << center_reg_mm
            << " mm  ← primary\n";
  std::cout << "  rel-trans (obs):     " << rel_trans_mm
            << " mm  (rotation×" << kBaselineM << " m baseline)\n";
  std::cout << "  ratio center/rel:    "
            << center_reg_mm / std::max(rel_trans_mm, 1e-3) << "×\n";
  std::cout << "  → Intersection-scale relative extrinsic is rotation-dominated;\n";
  std::cout << "    center-reg does not inherit 70.7 m baseline leverage.\n";

  EXPECT_LT(center_reg_mm, rel_trans_mm + 5.0);
  EXPECT_LT(rel_rot_deg, 0.15);

  const int n_mc = NumMcSeeds();
  clic_calib::experiments::RtkMcInjectionConfig white_inj;
  white_inj.mode =
      clic_calib::experiments::RtkMcInjectionMode::kWhiteNoisePerSample;

  clic_calib::experiments::RtkMcInjectionConfig sys_inj;
  sys_inj.mode =
      clic_calib::experiments::RtkMcInjectionMode::kCoherentSystemBiasOnly;
  sys_inj.system_bias_sigma_h_m = 5.0 * noise.rtk_sigma_horizontal_m;
  sys_inj.system_bias_sigma_v_m = 5.0 * noise.rtk_sigma_vertical_m;

  // --- (a) + (2): perturbation source + serial-sector injection correlation ---
  std::cout << "\n=== (a)(2) MC N=" << n_mc
            << " obs fixed @rep; compare RTK injection ===\n";
  std::cout << "  coherent bias σ_h=" << sys_inj.system_bias_sigma_h_m * 1e3
            << " mm  σ_v=" << sys_inj.system_bias_sigma_v_m * 1e3 << " mm\n";

  const DualMcArm white_mc = CollectDualMc(
      n_mc, 0, kRepSeed, geom, white_inj, levers, noise, cfg, spline_cfg,
      t_d_nominal);
  const DualMcArm sys_mc = CollectDualMc(
      n_mc, 0, kRepSeed, geom, sys_inj, levers, noise, cfg, spline_cfg,
      t_d_nominal);

  PrintCovBlock("RTK white noise (per epoch)", white_mc);
  PrintCovBlock("RTK coherent system bias (global/epoch)", sys_mc);

  const double r_inj_white =
      Pearson1d(white_mc.ne_seg_inj_norm_m, white_mc.sw_seg_inj_norm_m);
  const double r_inj_sys =
      Pearson1d(sys_mc.ne_seg_inj_norm_m, sys_mc.sw_seg_inj_norm_m);
  const CovSummary cov_white = SummarizeCov(white_mc);
  const CovSummary cov_sys = SummarizeCov(sys_mc);

  std::cout << std::fixed << std::setprecision(3);
  std::cout << "  (2) r(||δp||_NE, ||δp||_SW) injected RTK:\n";
  std::cout << "      white noise MC:    " << r_inj_white << "\n";
  std::cout << "      coherent bias MC:  " << r_inj_sys << "\n";
  std::cout << "  → Serial 戊: white per-epoch noise is uncorrelated across\n";
  std::cout << "    0–45 s vs 45–90 s segments; coherent bias is identical.\n";

  EXPECT_GE(white_mc.n_ok, n_mc / 2);
  EXPECT_GE(sys_mc.n_ok, n_mc / 2);
  EXPECT_GT(r_inj_sys, 0.95)
      << "coherent RTK bias must correlate NE/SW segment means";
  EXPECT_LT(std::abs(r_inj_white), 0.5)
      << "white RTK noise should not correlate segment means";

  std::cout << "\n=== (a) §7.2 common-mode vs random RTK ===\n";
  if (cov_sys.rel_over_abs < 1.0) {
    std::cout << "  coherent bias: Cov(T_rel) trace < mean Cov(abs) — "
              << "common-mode cancellation YES (" << cov_sys.rel_over_abs
              << "×)\n";
    std::cout << "  → Relative extrinsic robust to RTK systematic bias;\n";
    std::cout << "    white RTK noise composes in relative frame ("
              << cov_white.rel_over_abs << "×).\n";
    EXPECT_LT(cov_sys.rel_over_abs, 1.0);
  } else {
    std::cout << "  coherent bias: Cov(T_rel) trace / mean Cov(abs) = "
              << cov_sys.rel_over_abs << " — no cancellation this round\n";
  }
  std::cout << "  white noise:   rel/mean(abs) = " << cov_white.rel_over_abs
            << " (random noise stacks in T_rel)\n";

  // --- (3) Overlap variant if serial white-noise path shows no common-mode ---
  if (cov_white.rel_over_abs > 1.0) {
    const auto geom_ov =
        clic_calib::experiments::DiagonalFlightE_OverlapGeometry(10.0);
    std::cout << "\n=== (3) Temporal overlap variant (10 s @ handoff) ===\n";
    std::cout << "  Same coherent system-bias MC; both LiDARs share body epochs.\n";

    const DualMcArm sys_ov = CollectDualMc(
        n_mc, 0, kRepSeed, geom_ov, sys_inj, levers, noise, cfg, spline_cfg,
        t_d_nominal);
    PrintCovBlock("overlap + coherent bias", sys_ov);
    const CovSummary cov_ov = SummarizeCov(sys_ov);
    const double r_inj_ov =
        Pearson1d(sys_ov.ne_seg_inj_norm_m, sys_ov.sw_seg_inj_norm_m);
    std::cout << "  r(||δp||_NE, ||δp||_SW) = " << r_inj_ov
              << "  rel/mean(abs)=" << cov_ov.rel_over_abs << "\n";
    std::cout << "  → Board-free does not require spatial FOV overlap; a short\n";
    std::cout << "    temporal overlap at handoff lets one trajectory segment\n";
    std::cout << "    coherently constrain both LiDARs and can tighten rel UQ.\n";
  }
}

int main(int argc, char** argv) { return ClicGTestRunAll(argc, argv); }
