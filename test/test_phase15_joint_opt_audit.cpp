#include <clic_calib/estimator/extrinsic_initializer.h>
#include <clic_calib/estimator/extrinsic_refiner.h>
#include <clic_calib/estimator/stage1_trajectory_fitter.h>
#include <clic_calib/target/body_centroid_analysis.h>
#include <clic_calib/utils/lever_arm.h>
#include <clic_calib/utils/noise_model.h>
#include "gtest_ceres_guard.hpp"


#include <gtest/gtest.h>

#include "diagnostic/closed_form_init_common.hpp"
#include "diagnostic/two_stage_probe_common.hpp"
#include "experiments/noise_regime_common.hpp"
#include "experiments/phase15_ablation_common.hpp"

#include <iomanip>
#include <iostream>

namespace {

constexpr uint32_t kRepSeed = 13025;
constexpr double kSphereRadiusM = 0.10;
constexpr double kStage1AlphaP = 0.01;
constexpr double kStage1AlphaR = 0.01;

clic_calib::ExtrinsicRefinerConfig MakeRefineCfg(
    const clic_calib::two_stage_probe::SplineConfig& spline_cfg,
    const clic_calib::two_stage_probe::CoarseExtrinsicInit& t_d_nominal) {
  clic_calib::ExtrinsicRefinerConfig cfg;
  cfg.lidar_target_mode = clic_calib::LidarTargetMode::kBodyCluster;
  cfg.body_lever_arm_mode = clic_calib::BodyLeverArmMode::kJointOptimize;
  cfg.sphere_radius_m = kSphereRadiusM;
  cfg.t_d_max_abs_s = spline_cfg.t_d_max_abs_s;
  cfg.lidar_cauchy_scale = 1.0;
  cfg.camera_huber_delta_px = 2.0;
  cfg.camera_K = {600.0, 600.0, 320.0, 240.0};
  cfg.max_iterations = 500;
  return cfg;
}

bool PrepareTags(clic_calib::experiments::Phase15Scenario* ps,
                 const clic_calib::LeverArmConfig& levers,
                 const clic_calib::NoiseModel& noise,
                 const clic_calib::two_stage_probe::CoarseExtrinsicInit& t_d,
                 const clic_calib::two_stage_probe::SplineConfig& spline_cfg) {
  const clic_calib::PinholeIntrinsics K{600.0, 600.0, 320.0, 240.0};
  const clic_calib::RadtanDistortion dist;
  const Eigen::Vector3d L_G_to_M = levers.L_G_to_M.at(0);
  clic_calib::Stage1TrajectoryConfig s1_cfg;
  s1_cfg.knot_interval_s = spline_cfg.knot_interval_s;
  s1_cfg.alpha_p = kStage1AlphaP;
  s1_cfg.alpha_R = kStage1AlphaR;
  s1_cfg.attitude_stride = 25;
  s1_cfg.trim_to_observation_support = true;
  const clic_calib::Stage1TrajectoryResult s1 =
      clic_calib::Stage1TrajectoryFitter::Fit(
          clic_calib::Stage1TrajectoryInput::FromRtkAttitudeStreams(
              ps->sc.rtk, ps->sc.attitude_obs),
          levers, s1_cfg);
  if (!s1.summary.IsSolutionUsable() || !s1.trajectory) {
    return false;
  }
  ps->sc.tag_obs = clic_calib::two_stage_probe::SynthesizeTagObsForTrajectory(
      ps->sc.tag_obs, *s1.trajectory, ps->sc.gt.T_CW, levers.L_B_to_G, L_G_to_M,
      t_d.t_d_C_s, K, dist, noise, kRepSeed);
  return true;
}

void PrintAudit(const char* label,
                const clic_calib::ExtrinsicRefinerProblemAudit& a) {
  std::cout << "\n=== joint-opt audit: " << label << " ===\n";
  std::cout << std::fixed << std::setprecision(4);
  std::cout << "  joint_path_active=" << a.joint_path_active << "\n";
  std::cout << "  residual blocks: joint=" << a.body_centroid_joint_blocks
            << " fixed-centroid=" << a.body_centroid_fixed_blocks
            << " sphere=" << a.sphere_blocks
            << " apriltag=" << a.apriltag_blocks
            << " total=" << a.total_residual_blocks << "\n";
  std::cout << "  L_B present=" << a.L_B_block_present
            << " constant=" << a.L_B_block_constant << "\n";
  std::cout << std::setprecision(6);
  std::cout << "  cost@init=" << a.cost_at_init
            << " |grad L_B|=" << a.L_B_gradient_norm_at_init
            << " dCost(+1mm x)=" << a.L_B_numeric_cost_drop_1mm_x << "\n";
}

}  // namespace

TEST(Phase15JointOptAudit, DiverseAspectWiringAndGradient) {
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

  clic_calib::experiments::Phase15Scenario ps_div =
      clic_calib::experiments::BuildPhase15ScenarioWithGeom(
          kRepSeed, noise,
          clic_calib::experiments::NearFieldHighAspectScenarioGeometry());
  ASSERT_TRUE(PrepareTags(&ps_div, levers, noise, t_d_nominal, spline_cfg));

  const clic_calib::Stage1TrajectoryInput s1_in =
      clic_calib::Stage1TrajectoryInput::FromRtkAttitudeStreams(
          ps_div.sc.rtk, ps_div.sc.attitude_obs);
  clic_calib::Stage1TrajectoryConfig s1_cfg;
  s1_cfg.knot_interval_s = spline_cfg.knot_interval_s;
  s1_cfg.alpha_p = kStage1AlphaP;
  s1_cfg.alpha_R = kStage1AlphaR;
  s1_cfg.attitude_stride = 25;
  s1_cfg.trim_to_observation_support = true;
  const clic_calib::Stage1TrajectoryResult s1 =
      clic_calib::Stage1TrajectoryFitter::Fit(s1_in, levers, s1_cfg);
  ASSERT_TRUE(s1.summary.IsSolutionUsable());
  ASSERT_TRUE(s1.trajectory);

  clic_calib::ExtrinsicInitializerConfig init_cfg;
  init_cfg.lidar_target_mode = clic_calib::LidarTargetMode::kBodyCluster;
  init_cfg.sphere_radius_m = kSphereRadiusM;
  init_cfg.nominal_t_d_L_s = t_d_nominal.t_d_L_s;
  init_cfg.nominal_t_d_C_s = t_d_nominal.t_d_C_s;
  init_cfg.camera_K = {600.0, 600.0, 320.0, 240.0};
  const clic_calib::GeometricInitReport geo =
      clic_calib::ExtrinsicInitializer::FromGeometric(
          *s1.trajectory, {}, ps_div.body_cluster, ps_div.sc.tag_obs, levers,
          init_cfg);

  const clic_calib::ExtrinsicRefinerConfig refine_cfg =
      MakeRefineCfg(spline_cfg, t_d_nominal);
  const auto audit = clic_calib::ExtrinsicRefiner::AuditProblem(
      *s1.trajectory, {}, ps_div.body_cluster, ps_div.sc.tag_obs, levers, noise,
      geo.init, refine_cfg);
  PrintAudit("NearFieldHighAspect", audit);

  EXPECT_TRUE(audit.joint_path_active);
  EXPECT_GT(audit.body_centroid_joint_blocks, 0);
  EXPECT_EQ(audit.body_centroid_fixed_blocks, 0);
  EXPECT_TRUE(audit.L_B_block_present);
  EXPECT_FALSE(audit.L_B_block_constant);
}

int main(int argc, char** argv) { return ClicGTestRunAll(argc, argv); }
