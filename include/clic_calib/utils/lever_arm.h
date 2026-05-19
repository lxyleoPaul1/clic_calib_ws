/*
 * clic_calib — constant lever arms from config/lever_arms.yaml.
 * Not optimized in the main calibration pipeline (§0).
 */

#pragma once

#include <Eigen/Core>
#include <map>
#include <string>
#include <vector>

namespace clic_calib {

struct LeverArms {
  Eigen::Vector3d lever_b_to_a = Eigen::Vector3d::Zero();
  Eigen::Vector3d lever_b_to_g = Eigen::Vector3d::Zero();
  std::map<int, Eigen::Vector3d> lever_g_to_mj;

  static LeverArms LoadFromYaml(const std::string& path);
};

}  // namespace clic_calib
