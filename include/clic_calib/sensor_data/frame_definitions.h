/*
 * clic_calib — coordinate frames for UAV–roadside calibration.
 * Convention: T_XY maps p_X = T_XY * p_Y (see §0 CURSOR_MASTER_PROMPT).
 */

#pragma once

namespace clic_calib {

enum class FrameId : int {
  kWorld = 0,   ///< W: UTM / ENU, RTK reference
  kBody,        ///< B: UAV body (DJI M3E FLU/FRD per config)
  kAntenna,     ///< A: RTK antenna phase center
  kTarget,      ///< G: reflective sphere center
  kMarker,      ///< M_j: AprilTag j on sphere surface
  kCamera,      ///< C: roadside camera
  kLidar        ///< L: roadside LiDAR
};

}  // namespace clic_calib
