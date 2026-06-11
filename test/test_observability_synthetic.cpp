#include <clic_calib/estimator/calibration_estimator.h>
#include <clic_calib/estimator/observability_analyzer.h>
#include <clic_calib/utils/camera_projection.h>
#include <clic_calib/utils/lever_arm.h>
#include <clic_calib/utils/noise_model.h>
#include <clic_calib/utils/sophus_utils.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <random>
#include <string>
#include <vector>

namespace {

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
};

AnalysisResult RunObservabilityAnalysis(const SyntheticScenario& scenario) {
  clic_calib::CalibrationEstimator estimator(ConfigDir());
  estimator.add_rtk_measurements(scenario.rtk);
  estimator.add_lidar_target_observations(0, scenario.lidar_obs);
  estimator.add_apriltag_observations(0, scenario.tag_obs);
  estimator.solve(1000);

  AnalysisResult out;
  out.layout = estimator.analysis_parameter_layout();
  clic_calib::ObservabilityAnalyzer analyzer;
  out.report = analyzer.analyze(estimator);
  return out;
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

}  // namespace

TEST(ObservabilitySynthetic, MultiLayerFlightIsWellObserved) {
  const SyntheticScenario scenario = MakeSyntheticScenario(
      MakeMultiLayerTrajectory(), ScenarioOptions{true, true, 1.0});
  const AnalysisResult result = RunObservabilityAnalysis(scenario);
  const auto& report = result.report;

  EXPECT_EQ(report.information_matrix.rows(), 12);
  EXPECT_EQ(report.information_matrix.cols(), 12);
  EXPECT_GT(report.lambda_min, 0.0);
  EXPECT_GT(report.pdop_ext, 0.0);
  EXPECT_LT(report.condition_number, 1e8);
}

TEST(ObservabilitySynthetic, CoplanarAblationIsDegenerate) {
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

  // Joint FIM gate: CalibrationEstimator + independent RTK factors @ 0.1 s
  // (not Stage-1 PW). Measured multi λ_min ≈ 0.00252 @ seal; 0.003 fails.
  EXPECT_GT(multi_result.report.lambda_min, 0.0025);
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

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
