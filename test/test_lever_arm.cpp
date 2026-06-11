#include <clic_calib/utils/lever_arm.h>
#include <clic_calib/utils/sophus_utils.hpp>
#include "gtest_ceres_guard.hpp"


#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <string>

namespace {

constexpr double kTol = 1e-9;

std::string PackageTestDataPath(const std::string& name) {
  const std::filesystem::path from_source =
      std::filesystem::path(__FILE__).parent_path() / "data" / name;
  if (std::filesystem::exists(from_source)) {
    return from_source.string();
  }
  const std::filesystem::path from_build =
      std::filesystem::path("test/data") / name;
  return from_build.string();
}

TEST(LeverArmHelpers, IdentityPose) {
  const clic_calib::SE3d T_WB;
  const Eigen::Vector3d L_B_to_A(0.12, 0.05, -0.58);
  const Eigen::Vector3d L_B_to_G(0.0, 0.0, -0.55);
  const Eigen::Vector3d L_G_to_M(0.10, 0.0, 0.0);

  const Eigen::Vector3d p_A =
      clic_calib::antenna_position_world(T_WB, L_B_to_A);
  EXPECT_NEAR((p_A - L_B_to_A).norm(), 0.0, kTol);

  const Eigen::Vector3d p_G =
      clic_calib::sphere_center_world(T_WB, L_B_to_G);
  EXPECT_NEAR((p_G - L_B_to_G).norm(), 0.0, kTol);

  const Eigen::Vector3d p_M =
      clic_calib::marker_position_world(T_WB, L_B_to_G, L_G_to_M);
  EXPECT_NEAR((p_M - (L_B_to_G + L_G_to_M)).norm(), 0.0, kTol);
}

TEST(LeverArmHelpers, Yaw90Degrees) {
  const double yaw = M_PI / 2.0;
  const clic_calib::SE3d T_WB(
      clic_calib::SO3d::rotZ(yaw), Eigen::Vector3d(10.0, 20.0, 30.0));
  const Eigen::Vector3d L_B_to_A(1.0, 0.0, 0.0);
  const Eigen::Vector3d L_B_to_G(0.0, 1.0, 0.0);
  const Eigen::Vector3d L_G_to_M(0.0, 0.0, 0.5);

  const Eigen::Vector3d p_A =
      clic_calib::antenna_position_world(T_WB, L_B_to_A);
  const Eigen::Vector3d expected_A =
      T_WB.translation() + T_WB.so3() * L_B_to_A;
  EXPECT_NEAR((p_A - expected_A).norm(), 0.0, kTol);

  const Eigen::Vector3d p_M =
      clic_calib::marker_position_world(T_WB, L_B_to_G, L_G_to_M);
  const Eigen::Vector3d expected_M =
      clic_calib::sphere_center_world(T_WB, L_B_to_G) + T_WB.so3() * L_G_to_M;
  EXPECT_NEAR((p_M - expected_M).norm(), 0.0, kTol);
}

TEST(LeverArmConfig, ParseYaml) {
  const std::string path = PackageTestDataPath("lever_arms_sample.yaml");
  ASSERT_TRUE(std::filesystem::exists(path)) << path;

  const clic_calib::LeverArmConfig cfg =
      clic_calib::LeverArmConfig::from_yaml(path);
  EXPECT_EQ(cfg.body_convention, "FRD");
  EXPECT_NEAR(cfg.L_B_to_A.x(), 0.12, kTol);
  EXPECT_NEAR(cfg.L_B_to_A.y(), 0.05, kTol);
  EXPECT_NEAR(cfg.L_B_to_A.z(), -0.58, kTol);
  EXPECT_NEAR(cfg.L_B_to_G.z(), -0.55, kTol);
  ASSERT_EQ(cfg.L_G_to_M.size(), 4u);
  EXPECT_NEAR(cfg.L_G_to_M.at(1).y(), 0.094, kTol);
  EXPECT_NEAR(cfg.L_G_to_M.at(2).z(), 0.082, kTol);
}

}  // namespace

int main(int argc, char** argv) { return ClicGTestRunAll(argc, argv); }
