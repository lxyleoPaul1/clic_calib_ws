#pragma once

#include <Eigen/Core>
#include <array>

namespace clic_calib {

/** @brief One AprilTag detection in a single camera image. */
struct AprilTagObservation {
  double t_sensor_ = 0.0;  ///< raw image timestamp [s], before t_d^C
  int tag_id_ = 0;
  std::array<Eigen::Vector2d, 4> corners_pixel_{};  ///< pixel coordinates, order TL,TR,BR,BL
  double detection_confidence_ = 0.0;
  int sensor_id_ = 0;  ///< index into config sensor_rig.yaml cameras[]

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace clic_calib
