/*
 * clic_calib — constant lever arms from config/lever_arms.yaml.
 * Not optimized in the main calibration pipeline (§0).
 */

#pragma once

#include <clic_calib/utils/sophus_utils.hpp>

#include <Eigen/Core>
#include <map>
#include <string>

namespace clic_calib {

/** @brief Body-frame lever arms and convention loaded from YAML. */
struct LeverArmConfig {
  Eigen::Vector3d L_B_to_A = Eigen::Vector3d::Zero();  ///< RTK antenna in B
  Eigen::Vector3d L_B_to_G = Eigen::Vector3d::Zero();  ///< sphere center in B
  Eigen::Vector3d L_B_to_body_centroid =
      Eigen::Vector3d::Zero();  ///< board-free body centroid in B
  std::map<int, Eigen::Vector3d> L_G_to_M;           ///< tag_id → offset G→M_j
  std::string body_convention = "FRD";               ///< "FRD" or "FLU"

  static LeverArmConfig from_yaml(const std::string& path);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/** @brief §4.1: p_A^W = T_WB * L_{B→A}. */
inline Eigen::Vector3d antenna_position_world(const SE3d& T_WB,
                                              const Eigen::Vector3d& L_B_to_A) {
  return T_WB * L_B_to_A;
}

/** @brief §4.1: p_G^W = T_WB * L_{B→G}. */
inline Eigen::Vector3d sphere_center_world(const SE3d& T_WB,
                                         const Eigen::Vector3d& L_B_to_G) {
  return T_WB * L_B_to_G;
}

/** @brief Board-free: p_body^W = T_WB * L_{B→body_centroid}. */
inline Eigen::Vector3d body_centroid_world(
    const SE3d& T_WB, const Eigen::Vector3d& L_B_to_body_centroid) {
  return T_WB * L_B_to_body_centroid;
}

/** @brief §4.1: p_{M_j}^W = p_G^W + R_WB * L_{G→M_j}. */
inline Eigen::Vector3d marker_position_world(const SE3d& T_WB,
                                             const Eigen::Vector3d& L_B_to_G,
                                             const Eigen::Vector3d& L_G_to_M_j) {
  return sphere_center_world(T_WB, L_B_to_G) + T_WB.so3() * L_G_to_M_j;
}

/** @brief Legacy struct (Phase 0). */
struct LeverArms {
  Eigen::Vector3d lever_b_to_a = Eigen::Vector3d::Zero();
  Eigen::Vector3d lever_b_to_g = Eigen::Vector3d::Zero();
  std::map<int, Eigen::Vector3d> lever_g_to_mj;

  static LeverArms LoadFromYaml(const std::string& path);
};

}  // namespace clic_calib
