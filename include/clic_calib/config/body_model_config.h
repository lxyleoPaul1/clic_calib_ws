#pragma once

#include <Eigen/Core>

#include <string>

namespace clic_calib {

/** @brief UAV body target geometry from config/target_geometry.yaml body_model. */
struct BodyModelConfig {
  double effective_radius_m = 0.35;
  /** Axis-aligned box half-extents in body frame B [m] (sim + registration). */
  Eigen::Vector3d half_extent_B = Eigen::Vector3d(0.25, 0.25, 0.12);
  std::string cad_path;
  Eigen::Vector3d centroid_lever_arm_B = Eigen::Vector3d::Zero();
  bool enable_point_to_model = false;

  static BodyModelConfig FromYaml(const std::string& path);
  static BodyModelConfig FromConfigDir(const std::string& config_dir);
};

}  // namespace clic_calib
