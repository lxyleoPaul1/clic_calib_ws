#pragma once

#include <Eigen/Core>
#include <vector>

namespace clic_calib {

struct AprilTagCornerObservation {
  int marker_id = 0;
  int corner_index = 0;
  Eigen::Vector2d pixel = Eigen::Vector2d::Zero();
};

struct AprilTagImageObservation {
  double timestamp_s = 0.0;
  int camera_id = 0;
  std::vector<AprilTagCornerObservation> corners;
};

}  // namespace clic_calib
