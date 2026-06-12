#include <clic_calib/config/body_model_config.h>
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

#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>

namespace {

constexpr uint32_t kRepSeed = 13025;
constexpr double kSphereRadiusM = 0.10;
constexpr double kStage1AlphaP = 0.01;
constexpr double kStage1AlphaR = 0.01;

clic_calib::TwoStagePipelineConfig MakeBaseConfig(
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

struct CellMetrics {
  double rot_deg = 0.0;
  double trans_mm = 0.0;
  bool ok = false;
};

void PrintCell(const CellMetrics& c) {
  std::cout << std::fixed << std::setprecision(3) << c.rot_deg << " / "
            << c.trans_mm;
}

CellMetrics FromPhase15(const clic_calib::experiments::Phase15TlwMetrics& m) {
  CellMetrics c;
  c.rot_deg = m.rot_deg;
  c.trans_mm = m.trans_mm;
  c.ok = m.stage2_ok;
  return c;
}

}  // namespace

TEST(Phase15MainTable, DiverseAspectVariantsAndTwoByFiveTable) {
  const std::string config_dir =
      clic_calib::experiments::ConfigDirFromExperiments();
  const auto levers =
      clic_calib::LeverArmConfig::from_yaml(config_dir + "/lever_arms.yaml");
  const auto body_model = clic_calib::BodyModelConfig::FromConfigDir(config_dir);
  const auto spline_cfg =
      clic_calib::two_stage_probe::LoadSplineConfig(config_dir + "/spline.yaml");
  const clic_calib::two_stage_probe::CoarseExtrinsicInit t_d_nominal =
      clic_calib::two_stage_probe::LoadCoarseExtrinsicsFromYaml(config_dir);
  const clic_calib::NoiseModel noise =
      clic_calib::NoiseModel::FromConfigDir(config_dir);
  const clic_calib::TwoStagePipelineConfig base_cfg =
      MakeBaseConfig(spline_cfg, t_d_nominal);
  const Eigen::Vector3d L_nom = levers.L_B_to_body_centroid;

  clic_calib::experiments::Phase15Scenario ps_conc =
      clic_calib::experiments::BuildPhase15Scenario(kRepSeed, noise);
  clic_calib::experiments::Phase15Scenario ps_div =
      clic_calib::experiments::BuildPhase15ScenarioWithGeom(
          kRepSeed, noise,
          clic_calib::experiments::NearFieldHighAspectScenarioGeometry());
  ASSERT_TRUE(PrepareTags(&ps_conc, levers, noise, t_d_nominal, spline_cfg));
  ASSERT_TRUE(PrepareTags(&ps_div, levers, noise, t_d_nominal, spline_cfg));

  const Eigen::Vector3d b_const_conc =
      clic_calib::ComputeBConstFromCentroidBackproject(
          ps_conc.sc.gt_traj, ps_conc.sc.gt.T_LW, ps_conc.sc.gt.t_d_L_s, L_nom,
          ps_conc.body_cluster);
  const Eigen::Vector3d b_const_div =
      clic_calib::ComputeBConstFromCentroidBackproject(
          ps_div.sc.gt_traj, ps_div.sc.gt.T_LW, ps_div.sc.gt.t_d_L_s, L_nom,
          ps_div.body_cluster);
  const Eigen::Vector3d L_target_div = L_nom + b_const_div;

  // (1) Diverse aspect: variants ① and ②.
  clic_calib::experiments::Phase15TlwMetrics m_obs_div =
      clic_calib::experiments::RunBodyPath(
          ps_div, levers, noise, base_cfg,
          clic_calib::BodyLeverArmMode::kObservedMean, ps_div.body_cluster);
  clic_calib::experiments::Phase15TlwMetrics m_joint_div;
  const auto joint_div = clic_calib::experiments::RunBodyPathJointDiagnostic(
      ps_div, levers, noise, base_cfg, ps_div.body_cluster, b_const_div,
      &m_joint_div);

  std::cout << "\n=== (1) NearFieldHighAspect @ seed " << kRepSeed << " ===\n";
  std::cout << std::fixed << std::setprecision(4);
  std::cout << "  b_const(GT) B [m]: " << b_const_div.transpose()
            << "  |b|=" << b_const_div.norm() * 1e3 << " mm\n";
  std::cout << "  L_nom + b_const B [m]: " << L_target_div.transpose() << "\n";
  std::cout << std::setprecision(3);
  std::cout << "  ① observed-mean p_B: rot=" << m_obs_div.rot_deg
            << " deg |trans|=" << m_obs_div.trans_mm << " mm\n";
  std::cout << "  ② joint-opt p_B:     rot=" << m_joint_div.rot_deg
            << " deg |trans|=" << m_joint_div.trans_mm << " mm\n";
  std::cout << std::setprecision(4);
  std::cout << "  joint p_B before [m]: " << joint_div.L_B_before.transpose()
            << "\n";
  std::cout << "  joint p_B after  [m]: " << joint_div.L_B_after.transpose()
            << "\n";
  std::cout << std::setprecision(3);
  std::cout << "  |Δp_B|=" << joint_div.L_B_delta_mm
            << " mm  |p_B - (L_nom+b_const)|="
            << joint_div.dist_to_L_nom_plus_bconst_mm << " mm\n";
  std::cout << "  |Δt_LW|=" << joint_div.t_LW_delta_mm << " mm\n";

  const auto iter_obs = clic_calib::experiments::RunBodyPathIterativeObservedMean(
      ps_div, levers, noise, base_cfg, ps_div.body_cluster, 3);
  std::cout << std::setprecision(3);
  std::cout << "  ③ iterative observed-mean (3x): |trans|="
            << iter_obs.final_metrics.trans_mm << " mm  rot="
            << iter_obs.final_metrics.rot_deg << " deg\n";
  std::cout << "     |trans| per iter:";
  for (double t : iter_obs.trans_mm_per_iter) {
    std::cout << " " << t;
  }
  std::cout << " mm\n";

  // (2) 2×5 table.
  clic_calib::TwoStagePipelineConfig cfg_sphere = base_cfg;
  cfg_sphere.init.lidar_target_mode = clic_calib::LidarTargetMode::kSphere;
  cfg_sphere.refine.lidar_target_mode = clic_calib::LidarTargetMode::kSphere;

  const auto sphere_conc_res =
      clic_calib::experiments::RunSpherePath(ps_conc, levers, noise, cfg_sphere);
  const auto sphere_div_res =
      clic_calib::experiments::RunSpherePath(ps_div, levers, noise, cfg_sphere);

  const CellMetrics c_sphere_conc = FromPhase15(clic_calib::experiments::TlwMetrics(
      clic_calib::SE3d(sphere_conc_res.extrinsics.lidar.q,
                       sphere_conc_res.extrinsics.lidar.t),
      ps_conc.sc.gt.T_LW, sphere_conc_res.stage2_ok));
  const CellMetrics c_sphere_div = FromPhase15(clic_calib::experiments::TlwMetrics(
      clic_calib::SE3d(sphere_div_res.extrinsics.lidar.q,
                       sphere_div_res.extrinsics.lidar.t),
      ps_div.sc.gt.T_LW, sphere_div_res.stage2_ok));

  const CellMetrics c_cent_conc = FromPhase15(
      clic_calib::experiments::RunBodyPath(
          ps_conc, levers, noise, base_cfg,
          clic_calib::BodyLeverArmMode::kNominalYaml, ps_conc.body_cluster));
  const CellMetrics c_cent_div = FromPhase15(
      clic_calib::experiments::RunBodyPath(
          ps_div, levers, noise, base_cfg,
          clic_calib::BodyLeverArmMode::kNominalYaml, ps_div.body_cluster));

  const CellMetrics c_obs_conc = FromPhase15(
      clic_calib::experiments::RunBodyPath(
          ps_conc, levers, noise, base_cfg,
          clic_calib::BodyLeverArmMode::kObservedMean, ps_conc.body_cluster));
  const CellMetrics c_obs_div = FromPhase15(m_obs_div);

  clic_calib::experiments::Phase15TlwMetrics m_joint_conc;
  const auto joint_conc = clic_calib::experiments::RunBodyPathJointDiagnostic(
      ps_conc, levers, noise, base_cfg, ps_conc.body_cluster, b_const_conc,
      &m_joint_conc);
  const CellMetrics c_joint_conc = FromPhase15(m_joint_conc);
  const CellMetrics c_joint_div = FromPhase15(m_joint_div);

  const CellMetrics c_mreg_conc = FromPhase15(
      clic_calib::experiments::RunModelRegPath(
          ps_conc, levers, noise, base_cfg, body_model, ps_conc.body_cluster));
  const CellMetrics c_mreg_div = FromPhase15(
      clic_calib::experiments::RunModelRegPath(
          ps_div, levers, noise, base_cfg, body_model, ps_div.body_cluster));

  std::cout << "\n=== (2) 2×5 T_LW error table @ seed " << kRepSeed
            << "  (rot [deg] / |trans| [mm]) ===\n";
  std::cout << "  | method           | concentrated      | diverse aspect    |\n";
  std::cout << "  |------------------|-------------------|-------------------|\n";
  auto row = [](const char* name, const CellMetrics& a, const CellMetrics& b) {
    std::cout << "  | " << std::setw(16) << std::left << name << " | ";
    PrintCell(a);
    std::cout << "       | ";
    PrintCell(b);
    std::cout << "       |\n";
  };
  row("sphere", c_sphere_conc, c_sphere_div);
  row("centroid-only", c_cent_conc, c_cent_div);
  row("observed-mean", c_obs_conc, c_obs_div);
  row("joint-opt", c_joint_conc, c_joint_div);
  row("model-reg", c_mreg_conc, c_mreg_div);

  // (3) Mechanism diagnostic.
  const auto spread_conc = clic_calib::ComputeAttitudeSpreadAtObservations(
      ps_conc.sc.gt_traj, ps_conc.sc.gt.t_d_L_s, ps_conc.body_cluster);
  const auto spread_div = clic_calib::ComputeAttitudeSpreadAtObservations(
      ps_div.sc.gt_traj, ps_div.sc.gt.t_d_L_s, ps_div.body_cluster);
  const auto wbias_conc = clic_calib::ComputeWorldBiasDirectionStats(
      ps_conc.sc.gt_traj, ps_conc.sc.gt.T_LW, ps_conc.sc.gt.t_d_L_s,
      b_const_conc, ps_conc.body_cluster);
  const auto wbias_div = clic_calib::ComputeWorldBiasDirectionStats(
      ps_div.sc.gt_traj, ps_div.sc.gt.T_LW, ps_div.sc.gt.t_d_L_s, b_const_div,
      ps_div.body_cluster);

  std::cout << "\n=== (3) mechanism diagnostic ===\n";
  std::cout << std::fixed << std::setprecision(2);
  std::cout << "  R_WB yaw circular variance (1=uniform; span is misleading)\n";
  std::cout << "    concentrated: " << spread_conc.yaw_circular_variance
            << "  diverse: " << spread_div.yaw_circular_variance << "\n";
  std::cout << "  R_WB pitch std [deg]  concentrated: " << spread_conc.pitch_std_deg
            << "  diverse: " << spread_div.pitch_std_deg
            << "  (span diverse: " << spread_div.pitch_span_deg << " deg)\n";
  std::cout << "  **mean bias vector norm** ||mean(R_LW R_WB b_const)_W|| [mm]\n";
  std::cout << "    concentrated: " << wbias_conc.mean_horizontal_norm_mm
            << "  diverse: " << wbias_div.mean_horizontal_norm_mm
            << "  (primary t_LW floor predictor)\n";
  std::cout << "  horizontal RMS [mm]  concentrated: "
            << wbias_conc.horizontal_rms_mm
            << "  diverse: " << wbias_div.horizontal_rms_mm << "\n";
  std::cout << "  vertical RMS [mm]    concentrated: "
            << wbias_conc.vertical_rms_mm
            << "  diverse: " << wbias_div.vertical_rms_mm << "\n";
  std::cout << "  |mean_W.z| [mm]      concentrated: "
            << wbias_conc.mean_abs_vertical_mm
            << "  diverse: " << wbias_div.mean_abs_vertical_mm << "\n";

  EXPECT_TRUE(c_cent_conc.ok);
  EXPECT_NEAR(c_cent_conc.trans_mm, 211.4, 5.0);
  EXPECT_TRUE(m_obs_div.stage2_ok);
  EXPECT_TRUE(m_joint_div.stage2_ok);
  // Factual: joint p_B may remain a no-op even on diverse aspect (coupled with t_LW).
}

int main(int argc, char** argv) { return ClicGTestRunAll(argc, argv); }
