#pragma once

#include <Eigen/Core>

namespace clic_calib {

/** @brief RTK fix position in world frame W at receiver time [s]. */
struct RtkMeasurement {
  double timestamp_s = 0.0;
  Eigen::Vector3d position_w = Eigen::Vector3d::Zero();
  Eigen::Matrix3d covariance_w = Eigen::Matrix3d::Identity();
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace clic_calib
