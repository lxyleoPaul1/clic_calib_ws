#include <clic_calib/estimator/calibration_estimator.h>
#include <clic_calib/estimator/observability_analyzer.h>
#include <clic_calib/utils/camera_projection.h>
#include <clic_calib/utils/lever_arm.h>
#include <clic_calib/utils/noise_model.h>
#include <clic_calib/utils/sophus_utils.hpp>

#include "gtest_ceres_guard.hpp"

#include <gtest/gtest.h>

#include <cstdlib>
#include <cstring>

#include <cmath>
#include <filesystem>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include <sys/wait.h>
#include <unistd.h>

namespace {

bool ObservabilityInChildProcess() {
  return std::getenv("CLIC_OBS_TEST_CHILD") != nullptr;
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

/** Parent runs child with single-test filter; child executes the test body. */
bool RunObservabilityTestInChild(const char* gtest_filter) {
  if (ObservabilityInChildProcess()) {
    return false;
  }
  const std::string exe = SelfExecutablePath();
  if (exe.empty()) {
    ADD_FAILURE() << "readlink(/proc/self/exe) failed";
    return true;
  }
  const pid_t pid = fork();
  if (pid < 0) {
    ADD_FAILURE() << "fork failed";
    return true;
  }
  if (pid == 0) {
    setenv("CLIC_OBS_TEST_CHILD", "1", 1);
    std::string filter_arg = std::string("--gtest_filter=") + gtest_filter;
    std::vector<std::string> arg_storage = {exe, filter_arg};
    std::vector<char*> argv;
    argv.reserve(arg_storage.size() + 1);
    for (std::string& arg : arg_storage) {
      argv.push_back(arg.data());
    }
    argv.push_back(nullptr);
    execv(exe.c_str(), argv.data());
    _exit(127);
  }
  int status = 0;
  if (waitpid(pid, &status, 0) < 0) {
    ADD_FAILURE() << "waitpid failed";
    return true;
  }
  if (WIFSIGNALED(status)) {
    ADD_FAILURE() << "child terminated by signal " << WTERMSIG(status);
    return true;
  }
  if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
    ADD_FAILURE() << "child exit code "
                  << (WIFEXITED(status) ? WEXITSTATUS(status) : -1);
    return true;
  }
  return true;
}

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

clic_calib::BodyTrajectory MakeCoplanarTrajectory() {
  clic_calib::BodyTrajectory traj(0.05, 0.0);
  const int num_knots = 24;
  const clic_calib::SE3d k0(clic_calib::SO3d::rotZ(0.0), Eigen::Vector3d::Zero());
  traj.setKnots(k0, num_knots);
  for (int i = 0; i < num_knots; ++i) {
    const double s = static_cast<double>(i) * 0.05;
    const clic_calib::SO3d R = clic_calib::SO3d(Eigen::Quaterniond::Identity());
    const Eigen::Vector3d p(0.5 * s, 0.3 * std::sin(s), 2.0);
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

struct ScenarioOptions {
  bool vertical_motion = true;
  bool include_camera = true;
  /** If >= 0, scale RTK z injection only (whitening always from noise_model.yaml). */
  double rtk_z_injection_scale = 1.0;
};

SyntheticScenario MakeSyntheticScenario(const clic_calib::BodyTrajectory& gt_traj,
                                        const ScenarioOptions& options) {
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
    const Eigen::Vector3d p_A = gt_traj.antenna_position_w(t, levers.L_B_to_A);
    Eigen::Vector3d rtk_noise = noise.SampleRtkNoise(rng);
    rtk_noise.z() *= options.rtk_z_injection_scale;
    m.p_A_W_observed_ = p_A + rtk_noise;
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
      Eigen::Vector3d dir(std::cos(phi), std::sin(phi), 0.0);
      if (!options.vertical_motion) {
        dir.z() = 0.0;
      }
      scan.points_L_.push_back(p_G_L + R_ball * dir.normalized());
    }
    scenario.lidar_obs.push_back(scan);
  }

  if (options.include_camera) {
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
  }
  return scenario;
}

struct AnalysisResult {
  clic_calib::ObservabilityReport report;
  clic_calib::AnalysisParameterLayout layout;
  clic_calib::SE3d T_LW_after_solve;
  int ceres_num_residuals = 0;
  double ceres_final_cost = 0.0;
};

AnalysisResult RunObservabilityAnalysis(const SyntheticScenario& scenario) {
  clic_calib::CalibrationEstimator estimator(ConfigDir());
  estimator.add_rtk_measurements(scenario.rtk);
  estimator.add_lidar_target_observations(0, scenario.lidar_obs);
  estimator.add_apriltag_observations(0, scenario.tag_obs);
  const ceres::Solver::Summary summary = estimator.solve(1000);

  AnalysisResult out;
  out.layout = estimator.analysis_parameter_layout();
  clic_calib::ObservabilityAnalyzer analyzer;
  out.report = analyzer.analyze(estimator);
  out.T_LW_after_solve = estimator.get_T_LW(0);
  out.ceres_final_cost = summary.final_cost;
  const ceres::Problem& problem = estimator.problem();
  out.ceres_num_residuals = problem.NumResiduals();
  return out;
}

void PrintFimAudit(const char* link_tag, const AnalysisResult& ar,
                   const SyntheticScenario& sc) {
  const auto& r = ar.report;
  const auto& ly = ar.layout;
  std::cout << "[fim_audit] link=" << link_tag << " rtk_n=" << sc.rtk.size()
            << " lidar_scans=" << sc.lidar_obs.size()
            << " tag_n=" << sc.tag_obs.size()
            << " attitude_n=0"
            << " num_local=" << ly.num_local_parameters
            << " ext_n=" << ly.extrinsic_local_indices.size()
            << " rest_n=" << ly.rest_local_indices.size()
            << " ceres_residuals=" << ar.ceres_num_residuals
            << " ceres_cost=" << ar.ceres_final_cost
            << " F_ext=" << r.information_matrix.rows() << "x"
            << r.information_matrix.cols() << " lambda_min=" << r.lambda_min
            << " lambda_max=" << r.lambda_max << " cond=" << r.condition_number
            << " pdop=" << r.pdop_ext
            << " |T_LW|_mm=" << ar.T_LW_after_solve.translation().norm() * 1e3
            << " eigenvalues=[";
  for (int i = 0; i < r.eigenvalues.size(); ++i) {
    if (i > 0) {
      std::cout << ",";
    }
    std::cout << r.eigenvalues(i);
  }
  std::cout << "]\n";
}

double DominanceOnPitchOrZTranslation(
    const Eigen::VectorXd& v,
    const clic_calib::AnalysisParameterLayout& layout) {
  double pitch = 0.0;
  double z_trans = 0.0;
  if (layout.lidar_rot_fext_indices.size() == 3) {
    pitch = std::abs(v(layout.lidar_rot_fext_indices[1]));
  }
  if (layout.lidar_trans_fext_indices.size() == 3) {
    z_trans = std::abs(v(layout.lidar_trans_fext_indices[2]));
  }
  return std::max(pitch, z_trans);
}

TEST(ObservabilitySynthetic, CoplanarAblationIsDegenerate) {
  if (RunObservabilityTestInChild(
          "ObservabilitySynthetic.CoplanarAblationIsDegenerate")) {
    return;
  }
  const SyntheticScenario multi = MakeSyntheticScenario(
      MakeMultiLayerTrajectory(), ScenarioOptions{true, true, 1.0});
  const SyntheticScenario coplanar = MakeSyntheticScenario(
      MakeCoplanarTrajectory(),
      ScenarioOptions{false, false, 0.0});

  const AnalysisResult multi_result = RunObservabilityAnalysis(multi);
  const AnalysisResult coplanar_result = RunObservabilityAnalysis(coplanar);

  std::cout << "[lambda_audit] multi_lambda_min=" << multi_result.report.lambda_min
            << " coplanar_lambda_min=" << coplanar_result.report.lambda_min
            << " coplanar/multi="
            << (coplanar_result.report.lambda_min /
                multi_result.report.lambda_min)
            << "\n";

  // Joint FIM gate: CalibrationEstimator RTK-only path (no attitude → no PW).
  EXPECT_GT(multi_result.report.lambda_min, 0.003);
  EXPECT_LT(coplanar_result.report.lambda_min,
            multi_result.report.lambda_min * 0.1)
      << "multi lambda_min=" << multi_result.report.lambda_min
      << " coplanar lambda_min=" << coplanar_result.report.lambda_min;

  EXPECT_GT(coplanar_result.report.condition_number,
            multi_result.report.condition_number * 5.0)
      << "multi cond=" << multi_result.report.condition_number
      << " coplanar cond=" << coplanar_result.report.condition_number;

  EXPECT_GT(coplanar_result.report.pdop_ext, multi_result.report.pdop_ext * 1.2)
      << "multi PDOP_ext=" << multi_result.report.pdop_ext
      << " coplanar PDOP_ext=" << coplanar_result.report.pdop_ext;

  const double multi_dom = DominanceOnPitchOrZTranslation(
      multi_result.report.worst_eigenvector, multi_result.layout);
  const double coplanar_dom = DominanceOnPitchOrZTranslation(
      coplanar_result.report.worst_eigenvector, coplanar_result.layout);
  EXPECT_GT(coplanar_dom + multi_dom, 0.2)
      << "expected identifiable pitch/z components in worst eigenvectors";
}

TEST(ObservabilitySynthetic, MultiLayerFlightIsWellObserved) {
  if (RunObservabilityTestInChild(
          "ObservabilitySynthetic.MultiLayerFlightIsWellObserved")) {
    return;
  }
  const SyntheticScenario scenario = MakeSyntheticScenario(
      MakeMultiLayerTrajectory(), ScenarioOptions{true, true, 1.0});
  const AnalysisResult result = RunObservabilityAnalysis(scenario);
  const auto& report = result.report;

  std::cout << "[lambda_audit] multi_lambda_min=" << report.lambda_min << "\n";

  EXPECT_EQ(report.information_matrix.rows(), 12);
  EXPECT_EQ(report.information_matrix.cols(), 12);
  EXPECT_GT(report.lambda_min, 0.003);
  EXPECT_GT(report.pdop_ext, 0.0);
  EXPECT_LT(report.condition_number, 1e8);
}

TEST(ObservabilitySynthetic, FimLinkAuditMulti) {
  if (RunObservabilityTestInChild("ObservabilitySynthetic.FimLinkAuditMulti")) {
    return;
  }
  const SyntheticScenario multi = MakeSyntheticScenario(
      MakeMultiLayerTrajectory(), ScenarioOptions{true, true, 1.0});
  const AnalysisResult multi_result = RunObservabilityAnalysis(multi);
  const char* link_tag = std::getenv("CLIC_FIM_LINK_TAG");
  if (!link_tag) {
    link_tag = "unknown";
  }
  PrintFimAudit(link_tag, multi_result, multi);
  EXPECT_GT(multi_result.report.lambda_min, 0.003);
}

}  // namespace

int main(int argc, char** argv) { return ClicGTestRunAll(argc, argv); }
