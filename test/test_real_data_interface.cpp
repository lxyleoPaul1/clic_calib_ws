#include <clic_calib/estimator/calibration_estimator.h>
#include <clic_calib/estimator/real_data_session.h>
#include <clic_calib/io/attitude_reader.h>
#include <clic_calib/utils/noise_model.h>
#include "gtest_ceres_guard.hpp"


#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <string>

namespace {

std::string ConfigDir() {
  const std::string candidates[] = {
      "config",
      "../config",
      "../../config",
      "src/clic_calib/config",
      "../src/clic_calib/config",
  };
  for (const std::string& c : candidates) {
    std::ifstream f(c + "/noise_model.yaml");
    if (f.good()) {
      return c;
    }
  }
  return "config";
}

void WriteTempAttitudeCsv(const std::string& path) {
  std::ofstream out(path);
  out << "t_world_s,qw,qx,qy,qz\n";
  out << "1.0,1,0,0,0\n";
  out << "1.02,1,0,0,0\n";
}

}  // namespace

TEST(RealDataInterface, ConfigReadinessChecklist) {
  const std::string config_dir = ConfigDir();
  const clic_calib::RealDataReadinessReport report =
      clic_calib::RealDataSession::CheckReadiness(config_dir);

  clic_calib::RealDataSession::PrintReadinessReport(std::cout, report);

  EXPECT_TRUE(report.config_dir_ok);
  EXPECT_TRUE(report.lever_arms_loaded);
  EXPECT_TRUE(report.noise_model_loaded);
  EXPECT_TRUE(report.attitude_sigma_configured);
  EXPECT_TRUE(report.attitude_stream_config_loaded);
  EXPECT_TRUE(report.target_geometry_loaded);
  EXPECT_TRUE(report.sensor_rig_loaded);
  EXPECT_EQ(report.attitude_stream.stride, 25);
  EXPECT_DOUBLE_EQ(report.attitude_stream.transport_delay_s, 0.0);
  EXPECT_DOUBLE_EQ(report.noise_model.attitude_sigma_roll_deg, 0.2);
  EXPECT_DOUBLE_EQ(report.noise_model.attitude_sigma_pitch_deg, 0.2);
  EXPECT_DOUBLE_EQ(report.noise_model.attitude_sigma_yaw_deg, 1.5);
  EXPECT_GE(report.stream_interfaces.size(), 6u);
}

TEST(RealDataInterface, AttitudeReaderAppliesSigmaAndTransportDelay) {
  const std::string config_dir = ConfigDir();
  const clic_calib::NoiseModel noise =
      clic_calib::NoiseModel::FromConfigDir(config_dir);

  const std::string csv = "/tmp/clic_test_attitude.csv";
  WriteTempAttitudeCsv(csv);

  clic_calib::AttitudeReader::Options opts;
  opts.noise_model = noise;
  opts.transport_delay_s = 0.05;
  const clic_calib::AttitudeReader reader(opts);
  const std::vector<clic_calib::AttitudeObservation> obs = reader.read(csv);

  ASSERT_EQ(obs.size(), 2u);
  EXPECT_NEAR(obs[0].t_world_, 0.95, 1e-9);

  const Eigen::Matrix3d expected = noise.AttitudeTangentCovarianceRad2();
  EXPECT_NEAR((obs[0].covariance_ - expected).norm(), 0.0, 1e-12);

  const double sr = noise.attitude_sigma_roll_deg * M_PI / 180.0;
  EXPECT_NEAR(obs[0].covariance_(0, 0), sr * sr, 1e-12);
}

TEST(RealDataInterface, EstimatorLoadsAttitudeStreamConfig) {
  const clic_calib::CalibrationEstimator estimator(ConfigDir());
  const clic_calib::AttitudeStreamConfig cfg =
      estimator.attitude_stream_config();
  EXPECT_EQ(cfg.stride, 25);
  EXPECT_DOUBLE_EQ(cfg.transport_delay_s, 0.0);
}

TEST(RealDataInterface, EstimatorAcceptsAllStreamTypes) {
  clic_calib::CalibrationEstimator estimator(ConfigDir());

  clic_calib::RTKMeasurement rtk;
  rtk.t_world_ = 0.0;
  rtk.fix_status_ = clic_calib::RTKMeasurement::FixStatus::FIXED;
  estimator.add_rtk_measurements({rtk});

  clic_calib::AttitudeObservation att;
  att.t_world_ = 0.0;
  att.covariance_ =
      clic_calib::NoiseModel::FromConfigDir(ConfigDir())
          .AttitudeTangentCovarianceRad2();
  estimator.add_attitude_observations({att});

  clic_calib::LiDARTargetObservation lidar;
  lidar.t_sensor_ = 0.0;
  estimator.add_lidar_target_observations(0, {lidar});

  clic_calib::AprilTagObservation tag;
  tag.t_sensor_ = 0.0;
  estimator.add_apriltag_observations(0, {tag});

  SUCCEED();
}

int main(int argc, char** argv) { return ClicGTestRunAll(argc, argv); }
