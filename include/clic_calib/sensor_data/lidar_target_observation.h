#pragma once

#include <Eigen/Core>
#include <vector>

namespace clic_calib {

struct LidarTargetObservation {
  double scan_timestamp_s = 0.0;
  std::vector<Eigen::Vector3d> points_l;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace clic_calib
