#include <clic_calib/estimator/calibration_estimator.h>
#include <clic_calib/spline/spline_segment.h>
#include <clic_calib/utils/camera_projection.h>
#include <clic_calib/utils/lever_arm.h>
#include <clic_calib/utils/noise_model.h>
#include <clic_calib/utils/sophus_utils.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
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

double RotationErrorDeg(const clic_calib::SE3d& T_est,
                        const clic_calib::SE3d& T_gt) {
  const clic_calib::SO3d R_err = T_gt.so3().inverse() * T_est.so3();
  return R_err.log().norm() * 180.0 / M_PI;
}

double TranslationErrorM(const clic_calib::SE3d& T_est,
                         const clic_calib::SE3d& T_gt) {
  return (T_est.translation() - T_gt.translation()).norm();
}

clic_calib::BodyTrajectory MakeGroundTruthTrajectory() {
  clic_calib::BodyTrajectory traj(0.05, 0.0);
  const int num_knots = 24;
  const clic_calib::SE3d k0(clic_calib::SO3d::rotZ(0.0), Eigen::Vector3d::Zero());
  traj.setKnots(k0, num_knots);
  for (int i = 0; i < num_knots; ++i) {
    const double s = static_cast<double>(i) * 0.05;
    const clic_calib::SO3d R = clic_calib::SO3d::rotZ(0.05 * s);
    const Eigen::Vector3d p(0.5 * s, 0.3 * std::sin(s), 2.0 + 0.1 * s);
    traj.setKnot(clic_calib::SE3d(R, p), i);
  }
  const int64_t t_end_ns = static_cast<int64_t>(5.0 * clic_calib::S_TO_NS);
  traj.extendKnotsTo(t_end_ns, traj.getKnotSO3(num_knots - 1),
                     traj.getKnotPos(num_knots - 1));
  return traj;
}

}  // namespace

TEST(SmokeTest, NoiseFreeFullPipelineRecoversGroundTruth) {
  // Fast smoke: noise-free identity check (Jacobian / wiring). Not a noise evaluation.
  const auto levers =
      clic_calib::LeverArmConfig::from_yaml(ConfigDir() + "/lever_arms.yaml");

  const clic_calib::BodyTrajectory gt_traj = MakeGroundTruthTrajectory();

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

  // Noise-free RTK: LiDAR / camera observations are also exact, so the strict
  // regression tests the joint MLE without cross-modal noise mismatch. Whitening
  // still uses config/noise_model.yaml (same as realistic pipelines).
  std::vector<clic_calib::RTKMeasurement> rtk;
  for (double t = 0.2; t <= 4.8; t += 0.1) {
    clic_calib::RTKMeasurement m;
    m.t_world_ = t;
    m.fix_status_ = clic_calib::RTKMeasurement::FixStatus::FIXED;
    m.p_A_W_observed_ = gt_traj.antenna_position_w(t, levers.L_B_to_A);
    m.covariance_ = noise.RtkPositionCovariance();
    rtk.push_back(m);
  }

  std::vector<clic_calib::LiDARTargetObservation> lidar_obs;
  for (double t = 0.5; t <= 4.5; t += 0.4) {
    clic_calib::LiDARTargetObservation scan;
    scan.t_sensor_ = t + t_d_L_gt;
    scan.sensor_id_ = 0;
    for (int k = 0; k < 24; ++k) {
      const double phi = 2.0 * M_PI * k / 24.0;
      const Eigen::Vector3d p_G_W =
          gt_traj.sphere_center_w(t, levers.L_B_to_G);
      const Eigen::Vector3d p_G_L = T_LW_gt * p_G_W;
      const Eigen::Vector3d dir(std::cos(phi), std::sin(phi), 0.0);
      scan.points_L_.push_back(p_G_L + R_ball * dir.normalized());
    }
    lidar_obs.push_back(scan);
  }

  std::vector<clic_calib::AprilTagObservation> tag_obs;
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
      const Eigen::Vector3d p_M_C =
          T_CW_gt * (T_WB * L_corner);
      det.corners_pixel_[c] =
          clic_calib::ProjectRadtan(p_M_C, K, dist, nullptr);
    }
    tag_obs.push_back(det);
  }

  // Initial extrinsics / t_d from config/sensor_rig.yaml (same as calibrate_offline).
  clic_calib::CalibrationEstimator estimator(ConfigDir());
  estimator.add_rtk_measurements(rtk);
  estimator.add_lidar_target_observations(0, lidar_obs);
  estimator.add_apriltag_observations(0, tag_obs);

  const ceres::Solver::Summary summary = estimator.solve(1000);
  ASSERT_TRUE(summary.IsSolutionUsable()) << summary.FullReport();

  const clic_calib::SE3d T_LW_est = estimator.get_T_LW(0);
  const clic_calib::SE3d T_CW_est = estimator.get_T_CW(0);

  EXPECT_LT(RotationErrorDeg(T_LW_est, T_LW_gt), 0.5);
  EXPECT_LT(TranslationErrorM(T_LW_est, T_LW_gt), 0.05);

  EXPECT_LT(RotationErrorDeg(T_CW_est, T_CW_gt), 0.3);
  EXPECT_LT(TranslationErrorM(T_CW_est, T_CW_gt), 0.03);

  EXPECT_NEAR(estimator.get_t_d_lidar(0), t_d_L_gt, 0.002);
  EXPECT_NEAR(estimator.get_t_d_camera(0), t_d_C_gt, 0.002);

  double sq_sum = 0.0;
  int count = 0;
  auto est_traj = estimator.get_trajectory();
  for (double t = 0.3; t <= 4.7; t += 0.1) {
    const Eigen::Vector3d p_est = est_traj->position_wb(t);
    const Eigen::Vector3d p_gt = gt_traj.position_wb(t);
    sq_sum += (p_est - p_gt).squaredNorm();
    ++count;
  }
  const double rms = std::sqrt(sq_sum / std::max(count, 1));
  EXPECT_LT(rms, 0.03);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
