#pragma once

#include <Eigen/Core>

#include <string>
#include <vector>

namespace clic_calib {

/** @brief Board-free LiDAR body cluster observation (centroid in LiDAR frame L). */
struct BodyClusterObservation {
  double t_sensor_ = 0.0;  ///< raw LiDAR timestamp [s], before t_d^L
  std::string sensor_key_;  ///< unique sensor id (v2 primary key)
  int sensor_id_ = 0;       ///< legacy int id; synced when sensor_key_ is numeric
  Eigen::Vector3d centroid_L_ = Eigen::Vector3d::Zero();
  std::uint32_t point_count_ = 0;
  double mean_range_m_ = 0.0;
  bool has_centroid_cov_ = false;
  Eigen::Matrix3d centroid_cov_ = Eigen::Matrix3d::Zero();  ///< [m^2], optional
  std::vector<Eigen::Vector3d> raw_points_L_;  ///< optional, for point-to-model

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/** @brief Parse numeric sensor_key_ into sensor_id_; non-numeric keys keep sensor_id_=0. */
inline void SyncBodyClusterLegacySensorId(BodyClusterObservation* obs) {
  if (!obs || obs->sensor_key_.empty()) {
    return;
  }
  bool all_digits = true;
  for (char c : obs->sensor_key_) {
    if (c < '0' || c > '9') {
      all_digits = false;
      break;
    }
  }
  if (all_digits) {
    obs->sensor_id_ = std::stoi(obs->sensor_key_);
  }
}

}  // namespace clic_calib
