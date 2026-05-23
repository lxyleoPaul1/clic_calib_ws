#include <clic_calib/estimator/extrinsic_initializer.h>
#include <clic_calib/estimator/extrinsic_refiner.h>
#include <clic_calib/estimator/stage1_trajectory_fitter.h>
#include <clic_calib/estimator/two_stage_pipeline.h>
#include <clic_calib/estimator/uq_decomposition.h>
#include <clic_calib/utils/lever_arm.h>
#include <clic_calib/utils/noise_model.h>

#include <gtest/gtest.h>

#include "diagnostic/attitude_scenario_common.hpp"
#include "diagnostic/closed_form_init_common.hpp"
#include "diagnostic/two_stage_probe_common.hpp"
#include "experiments/noise_regime_common.hpp"

#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>

namespace {

constexpr uint32_t kSeedBase = 13000;
constexpr uint32_t kRepSeed = 13025;
constexpr double kStage1AlphaP = 0.01;
constexpr double kStage1AlphaR = 0.01;
constexpr double kSphereRadiusM = 0.10;

int NumUqSeeds() {
  if (const char* env = std::getenv("CLIC_UQ_N")) {
    return std::max(100, std::atoi(env));
  }
  return 100;
}

clic_calib::TwoStagePipelineConfig PipelineConfig(
    const clic_calib::two_stage_probe::SplineConfig& spline_cfg,
    const clic_calib::two_stage_probe::CoarseExtrinsicInit& t_d_nominal) {
  clic_calib::TwoStagePipelineConfig cfg;
  cfg.stage1.knot_interval_s = spline_cfg.knot_interval_s;
  cfg.stage1.alpha_p = kStage1AlphaP;
  cfg.stage1.alpha_R = kStage1AlphaR;
  // §2 UQ frozen protocol: full 50 Hz attitude stream, no post-fit trim.
  cfg.stage1.attitude_stride = 1;
  cfg.stage1.trim_to_observation_support = false;

  cfg.init.sphere_radius_m = kSphereRadiusM;
  cfg.init.nominal_t_d_L_s = t_d_nominal.t_d_L_s;
  cfg.init.nominal_t_d_C_s = t_d_nominal.t_d_C_s;
  cfg.init.camera_K = {600.0, 600.0, 320.0, 240.0};

  cfg.refine.sphere_radius_m = kSphereRadiusM;
  cfg.refine.t_d_max_abs_s = spline_cfg.t_d_max_abs_s;
  // Prior-free Gaussian Stage-2 (probe SolveStage2Extrinsics; no robust loss).
  cfg.refine.lidar_cauchy_scale = 0.0;
  cfg.refine.camera_huber_delta_px = 0.0;
  cfg.refine.camera_K = cfg.init.camera_K;
  cfg.refine.max_iterations = 500;
  return cfg;
}

bool RefineStage2AtInit(
    const clic_calib::BodyTrajectory& traj,
    const clic_calib::experiments::SyntheticScenarioBundle& sc,
    const clic_calib::LeverArmConfig& levers,
    const clic_calib::NoiseModel& noise,
    const clic_calib::TwoStagePipelineConfig& cfg,
    const clic_calib::CoarseExtrinsicInit& init,
    clic_calib::ExtrinsicRefinerResult* s2_out) {
  if (!s2_out) {
    return false;
  }
  *s2_out = clic_calib::ExtrinsicRefiner::Refine(
      traj, sc.lidar_obs, sc.tag_obs, levers, noise, init, cfg.refine);
  return s2_out->converged;
}

bool SolveTotalOrTrajPropArm(
    uint32_t seed_traj, uint32_t seed_obs,
    const clic_calib::LeverArmConfig& levers,
    const clic_calib::NoiseModel& noise,
    const clic_calib::TwoStagePipelineConfig& cfg,
    const clic_calib::two_stage_probe::CoarseExtrinsicInit& t_d_nominal,
    clic_calib::experiments::SyntheticScenarioBundle* sc_out,
    clic_calib::ExtrinsicRefinerResult* s2_out) {
  if (!sc_out || !s2_out) {
    return false;
  }
  const clic_calib::PinholeIntrinsics K = cfg.init.camera_K;
  const clic_calib::RadtanDistortion dist;
  const Eigen::Vector3d L_G_to_M = levers.L_G_to_M.at(0);

  *sc_out = clic_calib::experiments::BuildNearFieldFimNoisyScenarioWithAttitude(
      seed_traj, seed_obs, noise);
  const clic_calib::Stage1TrajectoryInput s1_in =
      clic_calib::Stage1TrajectoryInput::FromRtkAttitudeStreams(
          sc_out->rtk, sc_out->attitude_obs);
  const clic_calib::Stage1TrajectoryResult s1 =
      clic_calib::Stage1TrajectoryFitter::Fit(s1_in, levers, cfg.stage1);
  if (!s1.summary.IsSolutionUsable() || !s1.trajectory) {
    return false;
  }

  sc_out->tag_obs = clic_calib::two_stage_probe::SynthesizeTagObsForTrajectory(
      sc_out->tag_obs, *s1.trajectory, sc_out->gt.T_CW, levers.L_B_to_G,
      L_G_to_M, t_d_nominal.t_d_C_s, K, dist, noise, seed_obs);

  const auto geo = clic_calib::two_stage_probe::MakeGeometricExtrinsicInit(
      *s1.trajectory, sc_out->lidar_obs, sc_out->tag_obs, levers, kSphereRadiusM,
      t_d_nominal);
  clic_calib::CoarseExtrinsicInit init;
  init.T_LW = geo.init.T_LW;
  init.T_CW = geo.init.T_CW;
  // Probe UQ protocol: yaml nominal t_d for world-time pairing at Stage-2 init.
  init.t_d_L_s = t_d_nominal.t_d_L_s;
  init.t_d_C_s = t_d_nominal.t_d_C_s;
  return RefineStage2AtInit(*s1.trajectory, *sc_out, levers, noise, cfg, init,
                            s2_out);
}

bool SolveFixedTrajArm(
    uint32_t obs_seed, const clic_calib::BodyTrajectory& fixed_traj,
    const clic_calib::LeverArmConfig& levers,
    const clic_calib::NoiseModel& noise,
    const clic_calib::TwoStagePipelineConfig& cfg,
    const clic_calib::two_stage_probe::CoarseExtrinsicInit& t_d_nominal,
    clic_calib::experiments::SyntheticScenarioBundle* sc_out,
    clic_calib::ExtrinsicRefinerResult* s2_out) {
  if (!sc_out || !s2_out) {
    return false;
  }
  const clic_calib::PinholeIntrinsics K = cfg.init.camera_K;
  const clic_calib::RadtanDistortion dist;
  const Eigen::Vector3d L_G_to_M = levers.L_G_to_M.at(0);

  *sc_out = clic_calib::experiments::BuildNearFieldFimNoisyScenarioWithAttitude(
      kRepSeed, obs_seed, noise);
  sc_out->tag_obs = clic_calib::two_stage_probe::SynthesizeTagObsForTrajectory(
      sc_out->tag_obs, fixed_traj, sc_out->gt.T_CW, levers.L_B_to_G, L_G_to_M,
      t_d_nominal.t_d_C_s, K, dist, noise, obs_seed);

  const auto geo = clic_calib::two_stage_probe::MakeGeometricExtrinsicInit(
      fixed_traj, sc_out->lidar_obs, sc_out->tag_obs, levers, kSphereRadiusM,
      t_d_nominal);
  clic_calib::CoarseExtrinsicInit init;
  init.T_LW = geo.init.T_LW;
  init.T_CW = geo.init.T_CW;
  init.t_d_L_s = t_d_nominal.t_d_L_s;
  init.t_d_C_s = t_d_nominal.t_d_C_s;
  return RefineStage2AtInit(fixed_traj, *sc_out, levers, noise, cfg, init,
                            s2_out);
}

void PushProductionSample(
    clic_calib::ExtrinsicMcArmStats* arm,
    const clic_calib::experiments::SyntheticScenarioBundle& sc,
    const clic_calib::ExtrinsicRefinerResult& s2) {
  clic_calib::PushExtrinsicMcSample(
      arm, s2.lidar.AsSE3(), s2.camera.AsSE3(), s2.lidar.t_d, s2.camera.t_d,
      sc.gt.T_LW, sc.gt.T_CW, sc.gt.t_d_L_s, sc.gt.t_d_C_s);
}

clic_calib::UqDecompositionMcResult CollectUqDecompositionMc(
    const clic_calib::LeverArmConfig& levers,
    const clic_calib::TwoStagePipelineConfig& cfg,
    const clic_calib::two_stage_probe::CoarseExtrinsicInit& t_d_nominal,
    const clic_calib::NoiseModel& noise, int num_seeds) {
  clic_calib::UqDecompositionMcResult out;

  clic_calib::experiments::SyntheticScenarioBundle rep_sc =
      clic_calib::experiments::BuildNearFieldFimNoisyScenarioWithAttitude(
          kRepSeed, kRepSeed, noise);
  const clic_calib::Stage1TrajectoryInput rep_in =
      clic_calib::Stage1TrajectoryInput::FromRtkAttitudeStreams(
          rep_sc.rtk, rep_sc.attitude_obs);
  const clic_calib::Stage1TrajectoryResult s1_rep =
      clic_calib::Stage1TrajectoryFitter::Fit(rep_in, levers, cfg.stage1);
  out.rep_traj_ok = s1_rep.summary.IsSolutionUsable() && s1_rep.trajectory;
  if (!out.rep_traj_ok) {
    return out;
  }

  for (int i = 0; i < num_seeds; ++i) {
    const uint32_t seed = kSeedBase + static_cast<uint32_t>(i);
    clic_calib::experiments::SyntheticScenarioBundle sc;
    clic_calib::ExtrinsicRefinerResult s2;

    if (SolveTotalOrTrajPropArm(seed, seed, levers, noise, cfg, t_d_nominal,
                                &sc, &s2)) {
      PushProductionSample(&out.total, sc, s2);
    }
    if (SolveFixedTrajArm(seed, *s1_rep.trajectory, levers, noise, cfg,
                          t_d_nominal, &sc, &s2)) {
      PushProductionSample(&out.fixed_traj, sc, s2);
    }
    if (SolveTotalOrTrajPropArm(seed, kRepSeed, levers, noise, cfg,
                                t_d_nominal, &sc, &s2)) {
      PushProductionSample(&out.traj_prop, sc, s2);
    }

    if ((i + 1) % 10 == 0 || i + 1 == num_seeds) {
      std::cout << "  UQ MC progress: " << (i + 1) << "/" << num_seeds
                << "  total=" << out.total.converged
                << " fixed=" << out.fixed_traj.converged
                << " traj=" << out.traj_prop.converged << "\n"
                << std::flush;
    }
  }
  return out;
}

int CountLwTranslationInBand(const clic_calib::UqDecompositionMcResult& uq,
                             double lo, double hi) {
  int n = 0;
  for (int d : {3, 4, 5}) {
    const double vf = clic_calib::SampleVar(
        *clic_calib::McArmDofConst(uq.fixed_traj, d));
    const double vp = clic_calib::SampleVar(
        *clic_calib::McArmDofConst(uq.traj_prop, d));
    const double vt = clic_calib::SampleVar(
        *clic_calib::McArmDofConst(uq.total, d));
    const double ratio = vt > 1e-24 ? (vf + vp) / vt : 0.0;
    if (std::isfinite(ratio) && ratio >= lo && ratio <= hi) {
      ++n;
    }
  }
  return n;
}

}  // namespace

TEST(UqDecomposition, RepSeedFixedTrajFimDiagnostic) {
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
      PipelineConfig(spline_cfg, t_d_nominal);

  clic_calib::experiments::SyntheticScenarioBundle sc;
  clic_calib::ExtrinsicRefinerResult s2;
  ASSERT_TRUE(SolveTotalOrTrajPropArm(kRepSeed, kRepSeed, levers, noise, cfg,
                                      t_d_nominal, &sc, &s2));

  const clic_calib::Stage1TrajectoryResult s1 =
      clic_calib::Stage1TrajectoryFitter::Fit(
          clic_calib::Stage1TrajectoryInput::FromRtkAttitudeStreams(
              sc.rtk, sc.attitude_obs),
          levers, cfg.stage1);
  ASSERT_TRUE(s1.summary.IsSolutionUsable());
  ASSERT_TRUE(s1.trajectory);

  const clic_calib::Stage2FimDiagnostic diag =
      clic_calib::ComputeStage2FimDiagnostic(
          *s1.trajectory, sc.lidar_obs, sc.tag_obs, s2.lidar, s2.camera,
          levers, noise, kSphereRadiusM, cfg.refine.camera_K,
          cfg.refine.camera_dist);

  clic_calib::PrintStage2FimDiagnostic(std::cout, diag, kRepSeed);
  EXPECT_EQ(diag.fim_report.rank, 11);
  EXPECT_GT(diag.fim_report.cond, 1e10);
}

TEST(UqDecomposition, Gate3ValidatedDecomposition) {
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
      PipelineConfig(spline_cfg, t_d_nominal);

  const int num_seeds = NumUqSeeds();
  const clic_calib::UqDecompositionBands bands;

  std::cout << "\n=== DECISION GATE 4 — Production UQ decomposition (N="
            << num_seeds << ") ===\n";
  std::cout << "  TOTAL:     seed_traj=seed_obs=i\n";
  std::cout << "  FIXED:     traj@rep=" << kRepSeed
            << ", vary Stage-2 obs noise only\n";
  std::cout << "  TRAJ-PROP: seed_traj=i, seed_obs=rep=" << kRepSeed << "\n";
  std::cout << "  Validation band: [" << bands.decomp_lo << ", "
            << bands.decomp_hi << "]\n\n";

  const clic_calib::UqDecompositionMcResult uq =
      CollectUqDecompositionMc(levers, cfg, t_d_nominal, noise, num_seeds);
  ASSERT_TRUE(uq.rep_traj_ok);
  EXPECT_GE(uq.total.converged, num_seeds - 5);
  EXPECT_GE(uq.fixed_traj.converged, num_seeds - 5);
  EXPECT_GE(uq.traj_prop.converged, num_seeds - 5);

  clic_calib::PrintMcBiasTableWithCaveat(std::cout, uq.total);
  clic_calib::DecompositionGateReport gate;
  clic_calib::PrintDecompositionValidationTable(std::cout, uq, bands.decomp_lo,
                                                bands.decomp_hi, &gate);
  clic_calib::PrintDecouplingCostTable(std::cout, uq);

  const clic_calib::McBiasCaveatReport caveat =
      clic_calib::EvaluateMcBiasCaveats(uq.total);
  const clic_calib::TrajPropDominanceReport dom =
      clic_calib::ComputeTrajPropDominance(uq.traj_prop);
  const double trans_rot_ratio =
      dom.rot_var_sum > 1e-24 ? dom.trans_var_sum / dom.rot_var_sum : 0.0;
  const double lw_trans_rot =
      dom.lw_rot_var > 1e-24 ? dom.lw_trans_var / dom.lw_rot_var : 0.0;
  const double lw_tz_decouple = gate.decouple_traj_over_fixed[5];
  const int lw_trans_in_band = CountLwTranslationInBand(uq, bands.decomp_lo,
                                                        bands.decomp_hi);

  std::cout << "\n=== DECISION GATE 4 — UQ reproduction summary ===\n";
  std::cout << "  Additive closure (ratio band): " << gate.in_band << "/14 ("
            << gate.trans_in_band << "/5 translations)\n";
  std::cout << "  Headline (zero-mean qualified): " << gate.headline_in_band
            << "/14\n";
  std::cout << "  LW translation closure: " << lw_trans_in_band << "/3\n";
  std::cout << "  LW_tz traj/fixed decouple: " << std::fixed
            << std::setprecision(2) << lw_tz_decouple << "×\n";
  std::cout << "  Traj-prop trans/rot dominance: " << trans_rot_ratio << "×\n";
  std::cout << "  CW_tx bias caveat: " << caveat.cw_tx_mean_m * 1e3 << " ± "
            << caveat.cw_tx_std_m * 1e3 << " mm"
            << (caveat.cw_tx_zero_mean_violated ? " (|mean|>σ)" : "") << "\n";
  std::cout << "  CW_ty bias caveat: " << caveat.cw_ty_mean_m * 1e3 << " ± "
            << caveat.cw_ty_std_m * 1e3 << " mm"
            << (caveat.cw_ty_zero_mean_violated ? " (|mean|>σ)" : "") << "\n";

  if (gate.in_band >= 10 && gate.trans_in_band >= 4 && lw_trans_in_band >= 2 &&
      lw_tz_decouple >= 5.0 && trans_rot_ratio >= 100.0) {
    std::cout << "  → CONFIRMED: production UQ matches probe Gate3 thresholds\n";
    std::cout << "    (same-machine probe reference: 11/14 in band @ N=100).\n";
  } else if (gate.in_band >= 9 && lw_tz_decouple >= 5.0 &&
             trans_rot_ratio >= 100.0) {
    std::cout << "  → PARTIAL: core decoupling metrics match frozen §2; "
                 "ratio band "
              << gate.in_band << "/14 (probe 11/14 on this host).\n";
  } else {
    std::cout << "  → FAIL: decomposition does not close.\n";
  }

  EXPECT_GE(gate.in_band, 9)
      << "Need ≥9/14 DoF in decomposition band (probe same-host: 11/14)";
  EXPECT_GE(gate.trans_in_band, 4)
      << "Need ≥4/5 translation DoFs in decomposition band";
  EXPECT_GE(lw_trans_in_band, 2)
      << "LW translations should predominantly close";
  EXPECT_GE(lw_tz_decouple, 5.0)
      << "LW_tz traj/fixed decouple should be ~5.6× @ N=100";
  EXPECT_GE(trans_rot_ratio, 100.0)
      << "Traj-prop translation dominance should be ~223×";
  EXPECT_GE(lw_trans_rot, 100.0);

  EXPECT_TRUE(caveat.cw_tx_zero_mean_violated || caveat.cw_ty_zero_mean_violated)
      << "CW lateral bias caveat should remain visible @ N=100";
}

TEST(UqDecomposition, FixedTrajMcVsFimAtRep) {
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
      PipelineConfig(spline_cfg, t_d_nominal);
  const int num_seeds = NumUqSeeds();
  const clic_calib::UqDecompositionBands bands;

  clic_calib::experiments::SyntheticScenarioBundle rep_sc =
      clic_calib::experiments::BuildNearFieldFimNoisyScenarioWithAttitude(
          kRepSeed, kRepSeed, noise);
  const clic_calib::Stage1TrajectoryResult s1_rep =
      clic_calib::Stage1TrajectoryFitter::Fit(
          clic_calib::Stage1TrajectoryInput::FromRtkAttitudeStreams(
              rep_sc.rtk, rep_sc.attitude_obs),
          levers, cfg.stage1);
  ASSERT_TRUE(s1_rep.summary.IsSolutionUsable());
  ASSERT_TRUE(s1_rep.trajectory);

  clic_calib::ExtrinsicMcArmStats fixed_mc;
  clic_calib::Stage2FimDiagnostic fim_diag;
  for (int i = 0; i < num_seeds; ++i) {
    const uint32_t seed = kSeedBase + static_cast<uint32_t>(i);
    clic_calib::experiments::SyntheticScenarioBundle sc;
    clic_calib::ExtrinsicRefinerResult s2;
    if (!SolveFixedTrajArm(seed, *s1_rep.trajectory, levers, noise, cfg,
                           t_d_nominal, &sc, &s2)) {
      continue;
    }
    PushProductionSample(&fixed_mc, sc, s2);
    if (i == 0) {
      fim_diag = clic_calib::ComputeStage2FimDiagnostic(
          *s1_rep.trajectory, sc.lidar_obs, sc.tag_obs, s2.lidar, s2.camera,
          levers, noise, kSphereRadiusM, cfg.refine.camera_K,
          cfg.refine.camera_dist);
    }
  }

  std::cout << "\n=== Gate 1 (production) — Σ_fixed_MC vs FIM⁻¹ @ rep ===\n";
  int in_band = 0;
  clic_calib::PrintFimMcRatioTable(std::cout, fixed_mc, fim_diag.theo_std,
                                   bands.fim_ratio_lo, bands.fim_ratio_hi,
                                   &in_band);
  clic_calib::PrintStage2FimDiagnostic(std::cout, fim_diag, kRepSeed);

  EXPECT_GE(fixed_mc.converged, num_seeds - 5);
  EXPECT_GE(in_band, 10);
}
