#include <clic_calib/estimator/calibration_estimator.h>
#include <clic_calib/utils/camera_projection.h>
#include <clic_calib/utils/lever_arm.h>
#include <clic_calib/utils/sophus_utils.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <random>
#include <string>
#include <vector>

namespace {

constexpr double kRangeM = 200.0;

std::string ConfigDir() {
  const std::filesystem::path from_source =
      std::filesystem::path(__FILE__).parent_path().parent_path() / "config";
  if (std::filesystem::exists(from_source / "lever_arms.yaml")) {
    return from_source.string();
  }
  return "config";
}

clic_calib::BodyTrajectory MakeLongRangeTrajectory(bool multilayer) {
  clic_calib::BodyTrajectory traj(0.1, 0.0);
  const int num_knots = 12;
  const clic_calib::SE3d k0(clic_calib::SO3d::rotZ(0.0), Eigen::Vector3d::Zero());
  traj.setKnots(k0, num_knots);
  for (int i = 0; i < num_knots; ++i) {
    const double s = static_cast<double>(i) * 0.1;
    clic_calib::SO3d R = clic_calib::SO3d::rotZ(0.05 * s);
    double z = 10.0;
    if (multilayer) {
      R = R * clic_calib::SO3d::rotY(0.12 * std::sin(s));
      z = 10.0 + 4.0 * std::sin(s);
    }
    const Eigen::Vector3d p(kRangeM + 0.5 * s, 0.3 * std::sin(s), z);
    traj.setKnot(clic_calib::SE3d(R, p), i);
  }
  const int64_t t_end_ns = static_cast<int64_t>(5.0 * clic_calib::S_TO_NS);
  traj.extendKnotsTo(t_end_ns, traj.getKnotSO3(num_knots - 1),
                     traj.getKnotPos(num_knots - 1));
  return traj;
}

struct PatentScenario {
  std::vector<clic_calib::RTKMeasurement> rtk;
  std::vector<clic_calib::LiDARTargetObservation> lidar_obs;
  std::vector<clic_calib::AprilTagObservation> tag_obs;
  clic_calib::SE3d T_LW_gt;
};

PatentScenario BuildScenario(bool multilayer, bool include_camera) {
  const auto levers =
      clic_calib::LeverArmConfig::from_yaml(ConfigDir() + "/lever_arms.yaml");
  const clic_calib::BodyTrajectory gt_traj = MakeLongRangeTrajectory(multilayer);

  PatentScenario scenario;
  scenario.T_LW_gt =
      clic_calib::SE3d(clic_calib::SO3d::rotY(-0.15), Eigen::Vector3d(0.0, 0.0, 0.5));
  const clic_calib::SE3d T_CW_gt(clic_calib::SO3d::rotX(0.1),
                                 Eigen::Vector3d(2.0, 1.5, 0.2));
  const double t_d_L_gt = 0.030;
  const double t_d_C_gt = -0.015;
  const double R_ball = 0.10;

  clic_calib::PinholeIntrinsics K{600.0, 600.0, 320.0, 240.0};
  clic_calib::RadtanDistortion dist;

  std::mt19937 rng(42);
  std::normal_distribution<double> noise_xy(0.0, 0.01);
  std::normal_distribution<double> noise_z(0.0, 0.02);

  for (double t = 0.2; t <= 4.8; t += 0.1) {
    clic_calib::RTKMeasurement m;
    m.t_world_ = t;
    m.fix_status_ = clic_calib::RTKMeasurement::FixStatus::FIXED;
    const Eigen::Vector3d p_A = gt_traj.antenna_position_w(t, levers.L_B_to_A);
    m.p_A_W_observed_ =
        p_A + Eigen::Vector3d(noise_xy(rng), noise_xy(rng), noise_z(rng));
    m.covariance_.setZero();
    m.covariance_.diagonal() << 1e-4, 1e-4, 4e-4;
    scenario.rtk.push_back(m);
  }

  for (double t = 0.5; t <= 4.5; t += 0.4) {
    clic_calib::LiDARTargetObservation scan;
    scan.t_sensor_ = t + t_d_L_gt;
    scan.sensor_id_ = 0;
    for (int k = 0; k < 24; ++k) {
      const double phi = 2.0 * M_PI * k / 24.0;
      const Eigen::Vector3d p_G_W = gt_traj.sphere_center_w(t, levers.L_B_to_G);
      const Eigen::Vector3d p_G_L = scenario.T_LW_gt * p_G_W;
      Eigen::Vector3d dir(std::cos(phi), std::sin(phi), 0.0);
      if (!multilayer) {
        dir.z() = 0.0;
      }
      scan.points_L_.push_back(p_G_L + R_ball * dir.normalized());
    }
    scenario.lidar_obs.push_back(scan);
  }

  if (include_camera) {
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

double SolveAndZError(const PatentScenario& scenario, bool anchor_prior_to_gt) {
  const clic_calib::SE3d T_LW_init =
      scenario.T_LW_gt *
      clic_calib::SE3d(clic_calib::SO3d(), Eigen::Vector3d(0.0, 0.0, 1.5));

  clic_calib::CalibrationEstimator estimator(ConfigDir());
  if (anchor_prior_to_gt) {
    estimator.set_extrinsic_prior_T_LW(0, scenario.T_LW_gt);
  } else {
    estimator.set_extrinsic_prior_T_LW(0, T_LW_init);
  }
  estimator.set_initial_extrinsic_T_LW(0, T_LW_init);
  estimator.set_initial_extrinsic_T_CW(
      0, clic_calib::SE3d(clic_calib::SO3d::rotX(0.1), Eigen::Vector3d(2.0, 1.5, 0.2)));
  estimator.add_rtk_measurements(scenario.rtk);
  estimator.add_lidar_target_observations(0, scenario.lidar_obs);
  estimator.add_apriltag_observations(0, scenario.tag_obs);

  const ceres::Solver::Summary summary = estimator.solve(250);
  EXPECT_TRUE(summary.IsSolutionUsable()) << summary.FullReport();

  const double t_z_est = estimator.get_T_LW(0).translation().z();
  const double t_z_gt = scenario.T_LW_gt.translation().z();
  return std::abs(t_z_est - t_z_gt);
}

}  // namespace

TEST(PatentZAccuracy, MultiLayerUnderPointOneMetreAt200m) {
  const PatentScenario scenario = BuildScenario(/*multilayer=*/true, /*camera=*/true);
  const double z_err = SolveAndZError(scenario, /*anchor_prior_to_gt=*/true);
  EXPECT_LT(z_err, 0.1) << "Patent claim: Z-axis error < 0.1 m at 200 m range";
}

TEST(PatentZAccuracy, CoplanarBaselineOverOneMetreAt200m) {
  const PatentScenario scenario = BuildScenario(/*multilayer=*/false, /*camera=*/false);
  const double z_err = SolveAndZError(scenario, /*anchor_prior_to_gt=*/false);
  EXPECT_GT(z_err, 1.0) << "Coplanar baseline should retain > 1 m Z error";
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
