#include <clic_calib/estimator/calibration_estimator.h>
#include <clic_calib/estimator/multi_lidar_body_calibration.h>
#include <clic_calib/estimator/observability_analyzer.h>
#include <clic_calib/utils/camera_projection.h>
#include <clic_calib/utils/ceres_gflags_guard.h>
#include <clic_calib/utils/lever_arm.h>
#include <clic_calib/utils/noise_model.h>
#include <clic_calib/utils/sophus_utils.hpp>

#include "gtest_ceres_guard.hpp"

#include <gtest/gtest.h>

#include "diagnostic/two_stage_probe_common.hpp"
#include "experiments/dual_lidar_diagonal_geometry.hpp"
#include "experiments/dual_lidar_scenario_common.hpp"
#include "experiments/phase15_ablation_common.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <random>
#include <regex>
#include <string>
#include <vector>

#include <sys/wait.h>
#include <unistd.h>

namespace {

constexpr double kFrozenLambdaMin = 1.07357;
constexpr uint32_t kRepSeed = 13025;
constexpr int kObservedMeanIters = 3;

std::string ConfigDir() {
  const std::filesystem::path from_source =
      std::filesystem::path(__FILE__).parent_path().parent_path() / "config";
  if (std::filesystem::exists(from_source / "lever_arms.yaml")) {
    return from_source.string();
  }
  return "config";
}

clic_calib::BodyTrajectory MakeMultiLayerTrajectory() {
  clic_calib::BodyTrajectory traj(0.05, 0.0);
  const int num_knots = 24;
  const clic_calib::SE3d k0(clic_calib::SO3d::rotZ(0.0), Eigen::Vector3d::Zero());
  traj.setKnots(k0, num_knots);
  for (int i = 0; i < num_knots; ++i) {
    const double s = static_cast<double>(i) * 0.05;
    const clic_calib::SO3d R = clic_calib::SO3d::rotZ(0.05 * s) *
                               clic_calib::SO3d::rotY(0.12 * std::sin(s));
    const Eigen::Vector3d p(0.5 * s, 0.3 * std::sin(s), 2.0 + 0.4 * std::sin(s));
    traj.setKnot(clic_calib::SE3d(R, p), i);
  }
  const int64_t t_end_ns = static_cast<int64_t>(5.0 * clic_calib::S_TO_NS);
  traj.extendKnotsTo(t_end_ns, traj.getKnotSO3(num_knots - 1),
                     traj.getKnotPos(num_knots - 1));
  return traj;
}

struct SyntheticScenario {
  std::vector<clic_calib::RTKMeasurement> rtk;
  std::vector<clic_calib::LiDARTargetObservation> lidar_obs;
  std::vector<clic_calib::AprilTagObservation> tag_obs;
};

SyntheticScenario MakeSyntheticScenario(const clic_calib::BodyTrajectory& gt_traj) {
  const auto levers =
      clic_calib::LeverArmConfig::from_yaml(ConfigDir() + "/lever_arms.yaml");
  const clic_calib::SE3d T_LW_gt(clic_calib::SO3d::rotY(-0.15),
                                 Eigen::Vector3d(3.0, -1.0, 0.5));
  const clic_calib::SE3d T_CW_gt(clic_calib::SO3d::rotX(0.1),
                                 Eigen::Vector3d(2.0, 1.5, 0.2));
  const double t_d_L_gt = 0.030;
  const double t_d_C_gt = -0.015;
  const double R_ball = 0.10;
  clic_calib::PinholeIntrinsics K{600.0, 600.0, 320.0, 240.0};
  clic_calib::RadtanDistortion dist;
  const clic_calib::NoiseModel noise =
      clic_calib::NoiseModel::FromConfigDir(ConfigDir());
  std::mt19937 rng(123);
  SyntheticScenario scenario;
  for (double t = 0.2; t <= 4.8; t += 0.1) {
    clic_calib::RTKMeasurement m;
    m.t_world_ = t;
    m.fix_status_ = clic_calib::RTKMeasurement::FixStatus::FIXED;
    m.p_A_W_observed_ = gt_traj.antenna_position_w(t, levers.L_B_to_A) +
                          noise.SampleRtkNoise(rng);
    m.covariance_ = noise.RtkPositionCovariance();
    scenario.rtk.push_back(m);
  }
  for (double t = 0.5; t <= 4.5; t += 0.4) {
    clic_calib::LiDARTargetObservation scan;
    scan.t_sensor_ = t + t_d_L_gt;
    scan.sensor_id_ = 0;
    for (int k = 0; k < 24; ++k) {
      const double phi = 2.0 * M_PI * k / 24.0;
      const Eigen::Vector3d p_G_W = gt_traj.sphere_center_w(t, levers.L_B_to_G);
      const Eigen::Vector3d p_G_L = T_LW_gt * p_G_W;
      const Eigen::Vector3d dir(std::cos(phi), std::sin(phi), 0.0);
      scan.points_L_.push_back(p_G_L + R_ball * dir.normalized());
    }
    scenario.lidar_obs.push_back(scan);
  }
  const Eigen::Vector3d corners[4] = {
      Eigen::Vector3d(-0.025, -0.025, 0.0), Eigen::Vector3d(0.025, -0.025, 0.0),
      Eigen::Vector3d(0.025, 0.025, 0.0), Eigen::Vector3d(-0.025, 0.025, 0.0)};
  for (double t = 0.6; t <= 4.4; t += 0.35) {
    clic_calib::AprilTagObservation det;
    det.t_sensor_ = t + t_d_C_gt;
    det.tag_id_ = 0;
    det.sensor_id_ = 0;
    det.detection_confidence_ = 1.0;
    const Eigen::Vector3d L_G_to_M = levers.L_G_to_M.at(0);
    for (int c = 0; c < 4; ++c) {
      const Eigen::Vector3d L_corner = levers.L_B_to_G + L_G_to_M + corners[c];
      const clic_calib::SE3d T_WB = gt_traj.pose_wb(t);
      const Eigen::Vector3d p_M_C = T_CW_gt * (T_WB * L_corner);
      det.corners_pixel_[c] =
          clic_calib::ProjectRadtan(p_M_C, K, dist, nullptr);
    }
    scenario.tag_obs.push_back(det);
  }
  return scenario;
}

struct EstimatorRunResult {
  double lambda_min = 0.0;
  clic_calib::SE3d T_LW;
};

EstimatorRunResult RunEstimatorAnalysis(const SyntheticScenario& scenario) {
  clic_calib::ResetGflagsForCeresSolve();
  clic_calib::CalibrationEstimator estimator(ConfigDir());
  estimator.add_rtk_measurements(scenario.rtk);
  estimator.add_lidar_target_observations(0, scenario.lidar_obs);
  estimator.add_apriltag_observations(0, scenario.tag_obs);
  estimator.solve(1000);
  clic_calib::ObservabilityAnalyzer analyzer;
  const clic_calib::ObservabilityReport report = analyzer.analyze(estimator);
  EstimatorRunResult out;
  out.lambda_min = report.lambda_min;
  out.T_LW = estimator.get_T_LW(0);
  return out;
}

bool InChildProcess() {
  return std::getenv("CLIC_MULTI_LIDAR_CHILD") != nullptr;
}

std::string SelfExecutablePath() {
  char buf[4096];
  const ssize_t n = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
  if (n <= 0) {
    return {};
  }
  buf[n] = '\0';
  return std::string(buf);
}

double RunIsolatedLambdaMinViaChild() {
  const std::string exe = SelfExecutablePath();
  if (exe.empty()) {
    return -1.0;
  }
  int pipefd[2] = {-1, -1};
  if (pipe(pipefd) != 0) {
    return -1.0;
  }
  const pid_t pid = fork();
  if (pid < 0) {
    close(pipefd[0]);
    close(pipefd[1]);
    return -1.0;
  }
  if (pid == 0) {
    close(pipefd[0]);
    dup2(pipefd[1], STDOUT_FILENO);
    dup2(pipefd[1], STDERR_FILENO);
    close(pipefd[1]);
    setenv("CLIC_MULTI_LIDAR_CHILD", "1", 1);
    const char* filter =
        "--gtest_filter=MultiLidarBodyCalibration.TwoEstimatorsInProcessStable";
    std::vector<std::string> args = {exe, filter};
    std::vector<char*> argv;
    for (std::string& a : args) {
      argv.push_back(a.data());
    }
    argv.push_back(nullptr);
    execv(exe.c_str(), argv.data());
    _exit(127);
  }
  close(pipefd[1]);
  std::string captured;
  char buf[512];
  ssize_t n = 0;
  while ((n = read(pipefd[0], buf, sizeof(buf))) > 0) {
    captured.append(buf, static_cast<size_t>(n));
  }
  close(pipefd[0]);
  int status = 0;
  waitpid(pid, &status, 0);
  if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
    return -1.0;
  }
  std::smatch m;
  if (std::regex_search(
          captured, m,
          std::regex(R"(\[multi_lidar_audit\] isolated lambda_min=([0-9.eE+-]+))"))) {
    return std::stod(m[1].str());
  }
  return -1.0;
}

clic_calib::TwoStagePipelineConfig MakeBaseConfig(
    const clic_calib::two_stage_probe::SplineConfig& spline_cfg,
    const clic_calib::two_stage_probe::CoarseExtrinsicInit& t_d_nominal) {
  clic_calib::TwoStagePipelineConfig cfg;
  cfg.stage1.knot_interval_s = spline_cfg.knot_interval_s;
  cfg.stage1.alpha_p = 0.01;
  cfg.stage1.alpha_R = 0.01;
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
  // Match experiment_a gate flights (乙); 戊 uses centroid_cov override in cfg_e05.
  cfg.refine.use_centroid_cov_whitening = false;
  return cfg;
}

clic_calib::MultiLidarBodyCalibrationResult RunProductionDual(
    const clic_calib::experiments::DualLidarPhase3Scenario& ds,
    const clic_calib::LeverArmConfig& levers,
    const clic_calib::NoiseModel& noise,
    const clic_calib::TwoStagePipelineConfig& cfg) {
  const auto ps_ne =
      clic_calib::experiments::ToPhase15SensorSlice(ds, "lidar_NE");
  const auto ps_sw =
      clic_calib::experiments::ToPhase15SensorSlice(ds, "lidar_SW");
  const Eigen::Vector3d post_ne =
      clic_calib::experiments::DiagonalLidarPostW(ds.geom.dual_preset,
                                                  "lidar_NE");
  const Eigen::Vector3d post_sw =
      clic_calib::experiments::DiagonalLidarPostW(ds.geom.dual_preset,
                                                  "lidar_SW");
  const std::vector<double> scales_ne =
      clic_calib::experiments::ComputeBodyTemporalDecorrelationScales(
          ds, "lidar_NE", levers, ps_ne.body_cluster, {});
  const std::vector<double> scales_sw =
      clic_calib::experiments::ComputeBodyTemporalDecorrelationScales(
          ds, "lidar_SW", levers, ps_sw.body_cluster, {});
  const std::vector<double>* scales_ne_ptr =
      scales_ne.empty() ? nullptr : &scales_ne;
  const std::vector<double>* scales_sw_ptr =
      scales_sw.empty() ? nullptr : &scales_sw;

  clic_calib::BodyGatedCalibrationInput shared;
  shared.rtk = ds.sc.rtk;
  shared.attitude = ds.sc.attitude_obs;
  shared.tags = ds.sc.tag_obs;
  shared.nominal_t_d_L_s = ds.sc.gt.t_d_L_s;

  clic_calib::LidarBodyCalibrationSpec spec_ne;
  spec_ne.sensor_id = 0;
  spec_ne.sensor_key = "lidar_NE";
  spec_ne.body_obs = ps_ne.body_cluster;
  spec_ne.lidar_post_W = post_ne;
  spec_ne.gt_T_LW = &ds.gt_lidars.T_LW.at("lidar_NE");
  spec_ne.temporal_sqrt_info_scales = scales_ne_ptr;
  spec_ne.aspect_trajectory = &ds.sc.gt_traj;

  clic_calib::LidarBodyCalibrationSpec spec_sw;
  spec_sw.sensor_id = 1;
  spec_sw.sensor_key = "lidar_SW";
  spec_sw.body_obs = ps_sw.body_cluster;
  spec_sw.lidar_post_W = post_sw;
  spec_sw.gt_T_LW = &ds.gt_lidars.T_LW.at("lidar_SW");
  spec_sw.temporal_sqrt_info_scales = scales_sw_ptr;
  spec_sw.aspect_trajectory = &ds.sc.gt_traj;

  return clic_calib::CalibrateMultiLidarBodyGated(
      shared, {spec_ne, spec_sw}, levers, noise, cfg, kObservedMeanIters);
}

}  // namespace

TEST(MultiLidarBodyCalibration, TwoEstimatorsInProcessStable) {
  if (InChildProcess()) {
    const SyntheticScenario scenario = MakeSyntheticScenario(MakeMultiLayerTrajectory());
    const EstimatorRunResult r = RunEstimatorAnalysis(scenario);
    std::cout << "[multi_lidar_audit] isolated lambda_min=" << r.lambda_min
              << " |T_LW|_mm=" << r.T_LW.translation().norm() * 1e3 << "\n";
    EXPECT_GT(r.lambda_min, 0.003);
    EXPECT_NEAR(r.lambda_min, kFrozenLambdaMin, 0.01);
    return;
  }

  const SyntheticScenario scenario = MakeSyntheticScenario(MakeMultiLayerTrajectory());
  const double iso_lambda = RunIsolatedLambdaMinViaChild();
  ASSERT_GT(iso_lambda, 0.0);

  const EstimatorRunResult in_process = RunEstimatorAnalysis(scenario);

  std::cout << "[multi_lidar_audit] in_process lambda_min=" << in_process.lambda_min
            << " isolated=" << iso_lambda << "\n";

  EXPECT_NEAR(iso_lambda, kFrozenLambdaMin, 0.01);
  EXPECT_NEAR(in_process.lambda_min, iso_lambda, 1e-4);

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
  clic_calib::TwoStagePipelineConfig cfg =
      MakeBaseConfig(spline_cfg, t_d_nominal);
  cfg.refine.use_centroid_cov_whitening = true;
  const auto geom =
      clic_calib::experiments::DiagonalFlightE_Geometry();
  const auto ds = clic_calib::experiments::BuildDualDiagonalScenario(
      kRepSeed, noise, geom, spline_cfg, t_d_nominal);

  const clic_calib::MultiLidarBodyCalibrationResult multi =
      RunProductionDual(ds, levers, noise, cfg);
  const auto gate_rep =
      clic_calib::experiments::CalibrateDualDiagonalFlightEntry(
          ds, levers, noise, cfg, kObservedMeanIters);
  const auto ps_ne =
      clic_calib::experiments::ToPhase15SensorSlice(ds, "lidar_NE");
  const auto ps_sw =
      clic_calib::experiments::ToPhase15SensorSlice(ds, "lidar_SW");
  const Eigen::Vector3d post_ne =
      clic_calib::experiments::DiagonalLidarPostW(ds.geom.dual_preset,
                                                  "lidar_NE");
  const Eigen::Vector3d post_sw =
      clic_calib::experiments::DiagonalLidarPostW(ds.geom.dual_preset,
                                                  "lidar_SW");
  const std::vector<double> scales_ne =
      clic_calib::experiments::ComputeBodyTemporalDecorrelationScales(
          ds, "lidar_NE", levers, ps_ne.body_cluster, {});
  const std::vector<double> scales_sw =
      clic_calib::experiments::ComputeBodyTemporalDecorrelationScales(
          ds, "lidar_SW", levers, ps_sw.body_cluster, {});
  const std::vector<double>* scales_ne_ptr =
      scales_ne.empty() ? nullptr : &scales_ne;
  const std::vector<double>* scales_sw_ptr =
      scales_sw.empty() ? nullptr : &scales_sw;
  const double u_B_ne = clic_calib::experiments::ComputeUBAzimuthStdDeg(
      ds.sc.gt_traj, ds.sc.gt.t_d_L_s, post_ne, ps_ne.body_cluster);
  const double u_B_sw = clic_calib::experiments::ComputeUBAzimuthStdDeg(
      ds.sc.gt_traj, ds.sc.gt.t_d_L_s, post_sw, ps_sw.body_cluster);
  const auto ne_solo = clic_calib::experiments::CalibrateDualSensorViaPhase15(
      ps_ne, levers, noise, cfg, "lidar_NE", kObservedMeanIters, u_B_ne,
      scales_ne_ptr);
  const auto sw_solo = clic_calib::experiments::CalibrateDualSensorViaPhase15(
      ps_sw, levers, noise, cfg, "lidar_SW", kObservedMeanIters, u_B_sw,
      scales_sw_ptr);

  ASSERT_EQ(multi.per_sensor.count(0), 1u);
  ASSERT_EQ(multi.per_sensor.count(1), 1u);
  const double ne_multi = multi.per_sensor.at(0).calib.observed_mean.trans_mm;
  const double sw_multi = multi.per_sensor.at(1).calib.observed_mean.trans_mm;
  const double ne_solo_mm = ne_solo.calib.observed_mean.trans_mm;
  const double sw_solo_mm = sw_solo.calib.observed_mean.trans_mm;

  std::cout << "[multi_lidar_audit] production NE=" << ne_multi
            << " solo_NE=" << ne_solo_mm << " SW=" << sw_multi
            << " solo_SW=" << sw_solo_mm << "\n";

  EXPECT_NEAR(ne_multi, ne_solo_mm, 0.01);
  EXPECT_NEAR(sw_multi, sw_solo_mm, 0.01);
  EXPECT_NEAR(ne_multi, gate_rep.calib.ne.calib.observed_mean.trans_mm, 0.01);
  EXPECT_NEAR(sw_multi, gate_rep.calib.sw.calib.observed_mean.trans_mm, 0.01);
  EXPECT_NEAR(ne_multi, 12.15, 0.5);
  EXPECT_NEAR(sw_multi, 42.04, 0.5);
}

int main(int argc, char** argv) { return ClicGTestRunAll(argc, argv); }
