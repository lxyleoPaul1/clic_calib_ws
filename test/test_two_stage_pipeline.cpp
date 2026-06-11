#include <clic_calib/estimator/stage2_extrinsic_fim.h>
#include <clic_calib/estimator/two_stage_pipeline.h>
#include <clic_calib/utils/lever_arm.h>
#include <clic_calib/utils/noise_model.h>
#include <clic_calib/utils/sophus_utils.hpp>
#include "gtest_ceres_guard.hpp"


#include <gtest/gtest.h>

#include "diagnostic/attitude_scenario_common.hpp"
#include "diagnostic/closed_form_init_common.hpp"
#include "diagnostic/two_stage_probe_common.hpp"
#include "experiments/noise_regime_common.hpp"

#include <cmath>
#include <iomanip>
#include <iostream>

namespace {

constexpr int kNumSeeds = 50;
constexpr uint32_t kSeedBase = 13000;
constexpr uint32_t kRepSeed = 13025;
constexpr double kStage1AlphaP = 0.01;
constexpr double kStage1AlphaR = 0.01;
constexpr double kSphereRadiusM = 0.10;
constexpr double kCmTransMm = 50.0;
constexpr double kCmPitchDeg = 1.0;
constexpr double kWrongBasinTransMm = 500.0;
constexpr double kJointCondRef = 5.6865465329150156e+12;
constexpr int kExpectedStage2Rank = 11;

// Stage-2 FIM column layout: t_d_L, LW_rot×3, LW_t×3, t_d_C, CW_rot×3, CW_t×3.
constexpr int kColCwYaw = 10;
constexpr int kColCwTx = 11;
constexpr int kColCwTy = 12;

struct RunningStats {
  double sum = 0.0;
  double sum_sq = 0.0;
  int n = 0;
  void Push(double x) {
    sum += x;
    sum_sq += x * x;
    ++n;
  }
  double Mean() const { return n > 0 ? sum / n : 0.0; }
  double Std() const {
    if (n < 2) {
      return 0.0;
    }
    const double m = Mean();
    return std::sqrt(std::max(0.0, sum_sq / n - m * m));
  }
};

struct ExtrinsicErr {
  Eigen::Vector3d rot_rad = Eigen::Vector3d::Zero();
  Eigen::Vector3d trans_m = Eigen::Vector3d::Zero();
};

ExtrinsicErr ExtrinsicError(const clic_calib::SE3d& T_est,
                            const clic_calib::SE3d& T_gt) {
  ExtrinsicErr e;
  const clic_calib::SO3d R_err = T_gt.so3().inverse() * T_est.so3();
  e.rot_rad = R_err.log();
  e.trans_m = T_est.translation() - T_gt.translation();
  return e;
}

bool IsCmLevel(const ExtrinsicErr& lw, const ExtrinsicErr& cw) {
  return lw.trans_m.norm() * 1e3 < kCmTransMm &&
         cw.trans_m.norm() * 1e3 < kCmTransMm &&
         std::abs(lw.rot_rad.y()) * 180.0 / M_PI < kCmPitchDeg;
}

bool IsWrongBasin(const ExtrinsicErr& lw, const ExtrinsicErr& cw) {
  return lw.trans_m.norm() * 1e3 > kWrongBasinTransMm ||
         cw.trans_m.norm() * 1e3 > kWrongBasinTransMm;
}

clic_calib::TwoStagePipelineConfig PipelineConfig(
    const clic_calib::two_stage_probe::SplineConfig& spline_cfg,
    const clic_calib::two_stage_probe::CoarseExtrinsicInit& t_d_nominal) {
  clic_calib::TwoStagePipelineConfig cfg;
  cfg.stage1.knot_interval_s = spline_cfg.knot_interval_s;
  cfg.stage1.alpha_p = kStage1AlphaP;
  cfg.stage1.alpha_R = kStage1AlphaR;
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

void PrintBiasStd(const char* label, const RunningStats& s, const char* unit,
                  int prec) {
  std::cout << "  " << label << " bias=" << std::fixed << std::setprecision(prec)
            << s.Mean() << " ± " << s.Std() << " " << unit << "\n";
}

clic_calib::TwoStagePipelineResult RunConfigCAtSeed(
    uint32_t seed, const clic_calib::LeverArmConfig& levers,
    const clic_calib::NoiseModel& noise,
    const clic_calib::TwoStagePipelineConfig& cfg,
    const clic_calib::two_stage_probe::CoarseExtrinsicInit& t_d_nominal,
    clic_calib::experiments::SyntheticScenarioBundle* sc_out) {
  const clic_calib::PinholeIntrinsics K{600.0, 600.0, 320.0, 240.0};
  const clic_calib::RadtanDistortion dist;
  const Eigen::Vector3d L_G_to_M = levers.L_G_to_M.at(0);

  clic_calib::experiments::SyntheticScenarioBundle sc =
      clic_calib::experiments::BuildNearFieldFimNoisyScenarioWithAttitude(seed,
                                                                          noise);
  const clic_calib::Stage1TrajectoryInput s1_in =
      clic_calib::Stage1TrajectoryInput::FromRtkAttitudeStreams(
          sc.rtk, sc.attitude_obs);
  const clic_calib::Stage1TrajectoryResult s1 =
      clic_calib::Stage1TrajectoryFitter::Fit(s1_in, levers, cfg.stage1);
  if (!s1.summary.IsSolutionUsable() || !s1.trajectory) {
    return {};
  }

  sc.tag_obs = clic_calib::two_stage_probe::SynthesizeTagObsForTrajectory(
      sc.tag_obs, *s1.trajectory, sc.gt.T_CW, levers.L_B_to_G, L_G_to_M,
      t_d_nominal.t_d_C_s, K, dist, noise, seed);

  if (sc_out) {
    *sc_out = sc;
  }

  return clic_calib::TwoStagePipeline::Run(
      sc.rtk, sc.attitude_obs, sc.lidar_obs, sc.tag_obs, levers, noise, cfg);
}

}  // namespace

TEST(TwoStagePipeline, Stage2FimCameraColumnsNonZeroAtRepSeed) {
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
  const clic_calib::TwoStagePipelineResult result =
      RunConfigCAtSeed(kRepSeed, levers, noise, cfg, t_d_nominal, &sc);
  ASSERT_TRUE(result.stage1_ok);
  ASSERT_TRUE(result.stage2_ok);

  const clic_calib::ExtrinsicOptimizeState lw{
      result.extrinsics.lidar.q, result.extrinsics.lidar.t,
      result.extrinsics.lidar.t_d};
  const clic_calib::ExtrinsicOptimizeState cw{
      result.extrinsics.camera.q, result.extrinsics.camera.t,
      result.extrinsics.camera.t_d};

  const Eigen::MatrixXd F = clic_calib::BuildStage2ExtrinsicInformation(
      *result.trajectory, sc.lidar_obs, sc.tag_obs, lw, cw, levers, noise,
      kSphereRadiusM, cfg.refine.camera_K, cfg.refine.camera_dist);
  ASSERT_EQ(F.rows(), 14);
  ASSERT_EQ(F.cols(), 14);

  const clic_calib::SymmetricInformationReport rep =
      clic_calib::AnalyzeInformationMatrix(F);

  std::cout << "\n=== Stage-2 assembled FIM @ rep seed " << kRepSeed << " ===\n";
  std::cout << std::scientific << std::setprecision(4);
  std::cout << "  rank=" << rep.rank << "/14  cond=" << rep.cond
            << "  λ_min=" << rep.lambda_min << "  λ_max=" << rep.lambda_max
            << std::defaultfloat << "\n";
  std::cout << "  F(CW_yaw,CW_yaw)=" << F(kColCwYaw, kColCwYaw) << "\n";
  std::cout << "  F(CW_tx,CW_tx)=" << F(kColCwTx, kColCwTx) << "\n";
  std::cout << "  F(CW_ty,CW_ty)=" << F(kColCwTy, kColCwTy) << "\n";
  std::cout << "  ||F(:,CW_yaw)||=" << F.col(kColCwYaw).norm() << "\n";
  std::cout << "  ||F(:,CW_tx)||=" << F.col(kColCwTx).norm() << "\n";
  std::cout << "  ||F(:,CW_ty)||=" << F.col(kColCwTy).norm() << "\n";

  // Assembly-level H2 check: camera yaw / lateral translation columns must carry information.
  EXPECT_GT(std::abs(F(kColCwYaw, kColCwYaw)), 1e-6);
  EXPECT_GT(std::abs(F(kColCwTx, kColCwTx)), 1e-6);
  EXPECT_GT(std::abs(F(kColCwTy, kColCwTy)), 1e-6);
  EXPECT_GT(F.col(kColCwYaw).norm(), 1e-4);
  EXPECT_GT(F.col(kColCwTx).norm(), 1e-4);
  EXPECT_GT(F.col(kColCwTy).norm(), 1e-4);

  EXPECT_TRUE(rep.is_pd);
  EXPECT_EQ(rep.rank, kExpectedStage2Rank);
  EXPECT_LT(rep.cond, kJointCondRef);
  EXPECT_LT(rep.cond, 1e12);
}

TEST(TwoStagePipeline, ReproducesConfigCFromClosedFormInit) {
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

  RunningStats lw_trans_mm, cw_trans_mm, lw_pitch_deg, stage2_cond_log10;
  int converged = 0;
  int cm_success = 0;
  int wrong_basin = 0;

  std::cout << "\n=== Production TwoStagePipeline Config C (N=" << kNumSeeds
            << ") ===\n";
  std::cout << "  Stage-1 + closed-form init + prior-free Stage-2 refine\n";

  for (int i = 0; i < kNumSeeds; ++i) {
    const uint32_t seed = kSeedBase + static_cast<uint32_t>(i);
    clic_calib::experiments::SyntheticScenarioBundle sc;
    const clic_calib::TwoStagePipelineResult result =
        RunConfigCAtSeed(seed, levers, noise, cfg, t_d_nominal, &sc);
    if (!result.stage2_ok) {
      continue;
    }
    ++converged;

    const clic_calib::SE3d T_LW(result.extrinsics.lidar.q,
                                result.extrinsics.lidar.t);
    const clic_calib::SE3d T_CW(result.extrinsics.camera.q,
                                result.extrinsics.camera.t);
    const ExtrinsicErr lw_e = ExtrinsicError(T_LW, sc.gt.T_LW);
    const ExtrinsicErr cw_e = ExtrinsicError(T_CW, sc.gt.T_CW);

    lw_trans_mm.Push(lw_e.trans_m.norm() * 1e3);
    cw_trans_mm.Push(cw_e.trans_m.norm() * 1e3);
    lw_pitch_deg.Push(lw_e.rot_rad.y() * 180.0 / M_PI);

    if (IsCmLevel(lw_e, cw_e)) {
      ++cm_success;
    }
    if (IsWrongBasin(lw_e, cw_e)) {
      ++wrong_basin;
    }

    const clic_calib::ExtrinsicOptimizeState lw{
        result.extrinsics.lidar.q, result.extrinsics.lidar.t,
        result.extrinsics.lidar.t_d};
    const clic_calib::ExtrinsicOptimizeState cw{
        result.extrinsics.camera.q, result.extrinsics.camera.t,
        result.extrinsics.camera.t_d};
    const Eigen::MatrixXd F = clic_calib::BuildStage2ExtrinsicInformation(
        *result.trajectory, sc.lidar_obs, sc.tag_obs, lw, cw, levers, noise,
        kSphereRadiusM, cfg.refine.camera_K, cfg.refine.camera_dist);
    const clic_calib::SymmetricInformationReport fim =
        clic_calib::AnalyzeInformationMatrix(F);
    if (fim.is_pd) {
      stage2_cond_log10.Push(std::log10(fim.cond));
    }
  }

  std::cout << "  converged=" << converged << "/" << kNumSeeds << "\n";
  std::cout << "  cm-level=" << cm_success << "/" << kNumSeeds << "\n";
  std::cout << "  wrong-basin=" << wrong_basin << "/" << kNumSeeds << "\n";
  PrintBiasStd("LW pitch err", lw_pitch_deg, "deg", 4);
  PrintBiasStd("|LW trans| err", lw_trans_mm, "mm", 3);
  PrintBiasStd("|CW trans| err", cw_trans_mm, "mm", 3);
  if (stage2_cond_log10.n > 0) {
    std::cout << "  Stage-2 cond (log10) mean=" << stage2_cond_log10.Mean()
              << "\n";
  }

  EXPECT_GE(converged, 45);
  EXPECT_EQ(cm_success, kNumSeeds);
  EXPECT_EQ(wrong_basin, 0);
  EXPECT_LT(lw_trans_mm.Mean(), kCmTransMm);
  EXPECT_LT(cw_trans_mm.Mean(), kCmTransMm);
  EXPECT_LT(std::abs(lw_pitch_deg.Mean()), kCmPitchDeg);
  if (stage2_cond_log10.n > 0) {
    EXPECT_LT(std::pow(10.0, stage2_cond_log10.Mean()), kJointCondRef);
    EXPECT_LT(std::pow(10.0, stage2_cond_log10.Mean()), 1e12);
  }
}

int main(int argc, char** argv) { return ClicGTestRunAll(argc, argv); }
