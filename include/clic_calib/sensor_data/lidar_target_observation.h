#pragma once

#include <Eigen/Core>
#include <vector>

namespace clic_calib {

/** @brief Cluster of LiDAR returns on the calibration sphere in sensor frame L. */
struct LiDARTargetObservation {
  double t_sensor_ = 0.0;  ///< raw sensor timestamp [s], before time offset t_d^L
  std::vector<Eigen::Vector3d> points_L_;  ///< points on target, in LiDAR frame
  /** Per-point time offset from t_sensor_ [s]; empty ⇒ instantaneous scan. */
  std::vector<double> per_point_dt_;
  int sensor_id_ = 0;  ///< index into config sensor_rig.yaml lidars[]

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace clic_calib
