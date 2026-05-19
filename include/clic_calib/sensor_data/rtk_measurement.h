#pragma once

#include <cstdint>

#include <Eigen/Core>

namespace clic_calib {

/** @brief Single RTK/GNSS position fix in world frame W. */
struct RTKMeasurement {
  double t_world_ = 0.0;  ///< UTC time [s] (GPS week time or Unix as provided by reader)
  Eigen::Vector3d p_A_W_observed_ =
      Eigen::Vector3d::Zero();  ///< antenna position in W [m], local ENU / UTM
  Eigen::Matrix3d covariance_ = Eigen::Matrix3d::Identity();  ///< 3×3 [m²]

  enum class FixStatus : std::uint8_t {
    FIXED,
    FLOAT,
    SINGLE,
    INVALID
  };
  FixStatus fix_status_ = FixStatus::INVALID;

  double cn0_dbHz_ = 0.0;            ///< carrier-to-noise ratio [dB-Hz]
  double multipath_metric_ = 0.0;  ///< sensor-reported multipath indicator

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/** @brief Legacy name (Phase 0). */
using RtkMeasurement = RTKMeasurement;

}  // namespace clic_calib
