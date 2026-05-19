/*
 * clic_calib — coordinate frames for UAV–roadside calibration.
 * Convention: T_XY maps p_X = T_XY * p_Y (see §0).
 */

#pragma once

#include <cstdint>

namespace clic_calib {

/** @brief Static coordinate frames in the calibration problem. */
enum class Frame : std::uint8_t {
  W,  ///< World: UTM / local ENU, RTK reference
  B,  ///< UAV body (FRD or FLU per lever_arms.yaml)
  A,  ///< RTK antenna phase center
  G,  ///< Reflective sphere center (LiDAR target)
  C,  ///< Roadside camera
  L   ///< Roadside LiDAR
};

/** @brief AprilTag marker index on the sphere (IDs from config/target_geometry.yaml). */
using AprilTagId = int;

/** @brief Legacy alias (Phase 0). */
enum class FrameId : int {
  kWorld = 0,
  kBody,
  kAntenna,
  kTarget,
  kMarker,
  kCamera,
  kLidar
};

}  // namespace clic_calib
