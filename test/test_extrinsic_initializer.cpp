#include <clic_calib/estimator/extrinsic_initializer.h>
#include <clic_calib/estimator/stage1_trajectory_fitter.h>
#include <clic_calib/utils/lever_arm.h>
#include <clic_calib/utils/noise_model.h>
#include <clic_calib/utils/sophus_utils.hpp>

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
constexpr double kStage1AlphaP = 0.01;
constexpr double kStage1AlphaR = 0.01;
constexpr double kSphereRadiusM = 0.10;

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
  const clic_calib::SE3d T_err = T_gt.inverse() * T_est;
  e.rot_rad = T_err.so3().log();
  e.trans_m = T_err.translation();
  return e;
}

clic_calib::Stage1TrajectoryConfig Stage1Config(
    const clic_calib::two_stage_probe::SplineConfig& spline_cfg) {
  clic_calib::Stage1TrajectoryConfig cfg;
  cfg.knot_interval_s = spline_cfg.knot_interval_s;
  cfg.alpha_p = kStage1AlphaP;
  cfg.alpha_R = kStage1AlphaR;
  cfg.attitude_stride = 25;
  cfg.trim_to_observation_support = true;
  return cfg;
}

clic_calib::ExtrinsicInitializerConfig InitConfig(
    const clic_calib::two_stage_probe::CoarseExtrinsicInit& t_d_nominal) {
  clic_calib::ExtrinsicInitializerConfig cfg;
  cfg.sphere_radius_m = kSphereRadiusM;
  cfg.nominal_t_d_L_s = t_d_nominal.t_d_L_s;
  cfg.nominal_t_d_C_s = t_d_nominal.t_d_C_s;
  cfg.camera_K = {600.0, 600.0, 320.0, 240.0};
  return cfg;
}

void PrintBiasStd(const char* label, const RunningStats& s, const char* unit,
                  int prec) {
  std::cout << "  " << label << " bias=" << std::fixed << std::setprecision(prec)
            << s.Mean() << " ± " << s.Std() << " " << unit << "\n";
}

}  // namespace

TEST(ExtrinsicInitializer, ReproducesProbeUmeyamaPreIteration) {
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
  const clic_calib::Stage1TrajectoryConfig s1_cfg = Stage1Config(spline_cfg);
  const clic_calib::ExtrinsicInitializerConfig init_cfg =
      InitConfig(t_d_nominal);

  RunningStats umeyama_rot_deg, umeyama_trans_mm, umeyama_rms_mm, lw_pairs;
  int ok_count = 0;

  std::cout << "\n=== Production ExtrinsicInitializer T_LW (Umeyama, N="
            << kNumSeeds << ") ===\n";

  for (int i = 0; i < kNumSeeds; ++i) {
    const uint32_t seed = kSeedBase + static_cast<uint32_t>(i);
    const clic_calib::experiments::SyntheticScenarioBundle sc =
        clic_calib::experiments::BuildNearFieldFimNoisyScenarioWithAttitude(
            seed, noise);
    const clic_calib::Stage1TrajectoryInput input =
        clic_calib::Stage1TrajectoryInput::FromRtkAttitudeStreams(
            sc.rtk, sc.attitude_obs);
    const clic_calib::Stage1TrajectoryResult s1 =
        clic_calib::Stage1TrajectoryFitter::Fit(input, levers, s1_cfg);
    if (!s1.summary.IsSolutionUsable() || !s1.trajectory) {
      continue;
    }

    try {
      const clic_calib::GeometricInitReport geo =
          clic_calib::ExtrinsicInitializer::FromGeometric(
              *s1.trajectory, sc.lidar_obs, sc.tag_obs, levers, init_cfg);
      const auto e =
          ExtrinsicError(geo.init.T_LW, sc.gt.T_LW);
      umeyama_rot_deg.Push(e.rot_rad.norm() * 180.0 / M_PI);
      umeyama_trans_mm.Push(e.trans_m.norm() * 1e3);
      umeyama_rms_mm.Push(geo.lw_umeyama.rms_m * 1e3);
      lw_pairs.Push(static_cast<double>(geo.lw_pairs));
      EXPECT_DOUBLE_EQ(geo.init.t_d_L_s, 0.0);
      EXPECT_DOUBLE_EQ(geo.init.t_d_C_s, 0.0);
      ++ok_count;
    } catch (const std::exception&) {
      continue;
    }
  }

  std::cout << "  Umeyama succeeded=" << ok_count << "/" << kNumSeeds << "\n";
  PrintBiasStd("Umeyama rot |LW| err", umeyama_rot_deg, "deg", 4);
  PrintBiasStd("Umeyama |LW trans| err", umeyama_trans_mm, "mm", 3);
  PrintBiasStd("pair residual rms", umeyama_rms_mm, "mm", 3);

  EXPECT_GE(ok_count, 45);
  // Probe reference (Config C): ~0.017° / ~6.4 mm pre-iteration.
  EXPECT_NEAR(umeyama_rot_deg.Mean(), 0.017, 0.05);
  EXPECT_NEAR(umeyama_trans_mm.Mean(), 6.4, 15.0);
}

TEST(ExtrinsicInitializer, ReproducesProbePnPPreIteration) {
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
  const clic_calib::PinholeIntrinsics K{600.0, 600.0, 320.0, 240.0};
  const clic_calib::RadtanDistortion dist;
  const Eigen::Vector3d L_G_to_M = levers.L_G_to_M.at(0);
  const clic_calib::Stage1TrajectoryConfig s1_cfg = Stage1Config(spline_cfg);
  const clic_calib::ExtrinsicInitializerConfig init_cfg =
      InitConfig(t_d_nominal);

  RunningStats pnp_rot_deg, pnp_trans_mm, global_reproj_px, frames_ok;
  int ok_count = 0;

  std::cout << "\n=== Production ExtrinsicInitializer T_CW (tag-local IPPE, N="
            << kNumSeeds << ") ===\n";

  for (int i = 0; i < kNumSeeds; ++i) {
    const uint32_t seed = kSeedBase + static_cast<uint32_t>(i);
    const clic_calib::experiments::SyntheticScenarioBundle sc =
        clic_calib::experiments::BuildNearFieldFimNoisyScenarioWithAttitude(
            seed, noise);
    const clic_calib::Stage1TrajectoryInput input =
        clic_calib::Stage1TrajectoryInput::FromRtkAttitudeStreams(
            sc.rtk, sc.attitude_obs);
    const clic_calib::Stage1TrajectoryResult s1 =
        clic_calib::Stage1TrajectoryFitter::Fit(input, levers, s1_cfg);
    if (!s1.summary.IsSolutionUsable() || !s1.trajectory) {
      continue;
    }

    const std::vector<clic_calib::AprilTagObservation> tags =
        clic_calib::two_stage_probe::SynthesizeTagObsForTrajectory(
            sc.tag_obs, *s1.trajectory, sc.gt.T_CW, levers.L_B_to_G, L_G_to_M,
            t_d_nominal.t_d_C_s, K, dist, noise, seed);

    try {
      const clic_calib::GeometricInitReport geo =
          clic_calib::ExtrinsicInitializer::FromGeometric(
              *s1.trajectory, sc.lidar_obs, tags, levers, init_cfg);
      const auto e =
          ExtrinsicError(geo.init.T_CW, sc.gt.T_CW);
      pnp_rot_deg.Push(e.rot_rad.norm() * 180.0 / M_PI);
      pnp_trans_mm.Push(e.trans_m.norm() * 1e3);
      global_reproj_px.Push(geo.pnp_reproj_px_rms);
      frames_ok.Push(static_cast<double>(geo.pnp_frames_ok));
      ++ok_count;
    } catch (const std::exception&) {
      continue;
    }
  }

  std::cout << "  PnP succeeded=" << ok_count << "/" << kNumSeeds << "\n";
  PrintBiasStd("PnP rot |CW| err", pnp_rot_deg, "deg", 4);
  PrintBiasStd("PnP |CW trans| err", pnp_trans_mm, "mm", 3);
  PrintBiasStd("global reproj (picked)", global_reproj_px, "px", 3);

  EXPECT_GE(ok_count, 45);
  // Probe reference (Config C): ~0.0002° / ~0.16 mm pre-iteration.
  EXPECT_NEAR(pnp_rot_deg.Mean(), 0.0002, 0.01);
  EXPECT_NEAR(pnp_trans_mm.Mean(), 0.16, 2.0);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
