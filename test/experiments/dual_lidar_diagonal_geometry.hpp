#pragma once

/**
 * Phase 3 (A): diagonal dual roadside LiDAR at a large intersection.
 * Two posts at NE / SW corners, FOV toward intersection center, ~50–80 m spacing.
 */

#include "experiments/noise_regime_common.hpp"
#include "experiments/synthetic_flight_geometry.hpp"

#include <cmath>
#include <string>

namespace clic_calib {
namespace experiments {

enum class DualDiagonalFlightMode {
  kConstantHeading,              ///< 飞行甲: route yaw fixed in both lidar sectors
  kHighAspectPerLidar,           ///< 飞行乙: pitch/roll; yaw locked to −p_WB (velocity-coupled)
  kConstantHeadingSectorJitter,  ///< 飞行丙: fixed route yaw + per-sector pitch/roll jitter
  kPerSegmentYawSweep,           ///< 飞行丁: yaw sweep decoupled from translation per sector
  kPerSectorTidalLockPOI,        ///< 飞行戊: POI lock toward active-sector LiDAR (u_B ~ const)
};

struct DiagonalDualLidarPreset {
  /** Post offset along diagonal: NE (+h,+h), SW (−h,−h) [m]. */
  double post_half_extent_m = 25.0;
  double post_height_ne_m = 4.5;
  double post_height_sw_m = 5.5;
  Eigen::Vector3d look_target_W = Eigen::Vector3d(0.0, 0.0, 6.0);
};

/** Roadside post position in W for named sensor. */
inline Eigen::Vector3d DiagonalLidarPostW(const DiagonalDualLidarPreset& preset,
                                          const std::string& sensor_key) {
  const double h = preset.post_half_extent_m;
  if (sensor_key == "lidar_NE") {
    return Eigen::Vector3d(h, h, preset.post_height_ne_m);
  }
  if (sensor_key == "lidar_SW") {
    return Eigen::Vector3d(-h, -h, preset.post_height_sw_m);
  }
  throw std::runtime_error("unknown diagonal lidar key: " + sensor_key);
}

/** Inter-post ground distance along diagonal [m]. */
inline double DiagonalLidarSpacingM(const DiagonalDualLidarPreset& preset) {
  const double d = 2.0 * preset.post_half_extent_m * std::sqrt(2.0);
  return d;
}

/**
 * T_LW for a roadside post looking at @p look_target_W.
 * LiDAR +X ≈ horizontal toward target; +Z up in world when level.
 */
inline SE3d T_LWFromRoadsidePost(const Eigen::Vector3d& post_W,
                                 const Eigen::Vector3d& look_target_W) {
  Eigen::Vector3d fwd = look_target_W - post_W;
  fwd.z() = 0.0;
  if (fwd.norm() < 1e-6) {
    fwd = Eigen::Vector3d(1.0, 0.0, 0.0);
  } else {
    fwd.normalize();
  }
  const Eigen::Vector3d up_W(0.0, 0.0, 1.0);
  Eigen::Vector3d right_W = up_W.cross(fwd);
  if (right_W.norm() < 1e-6) {
    right_W = Eigen::Vector3d(0.0, 1.0, 0.0);
  } else {
    right_W.normalize();
  }
  const Eigen::Vector3d down_fwd = look_target_W - post_W;
  const double pitch =
      std::atan2(-down_fwd.z(), std::max(down_fwd.head<2>().norm(), 1e-6));
  SO3d R_WL = SO3d::rotZ(std::atan2(fwd.y(), fwd.x())) * SO3d::rotY(pitch);
  const SE3d T_WL(R_WL, post_W);
  return T_WL.inverse();
}

struct DualDiagonalFlightGeometry : SyntheticFlightGeometry {
  DiagonalDualLidarPreset dual_preset;
  DualDiagonalFlightMode dual_flight_mode = DualDiagonalFlightMode::kConstantHeading;
  /** Duration of NE-sector flight, then SW-sector (same length). */
  double sector_duration_s = 45.0;
  /** Optional temporal overlap [s]: both LiDAR sectors active near t=sector_duration. */
  double sector_overlap_s = 0.0;
  /** Route yaw for 飞行甲/丙 [rad] (body heading along NE→SW diagonal). */
  double route_yaw_rad = M_PI / 4.0;
  /** Orbit center offset from intersection along sector bisector [m]. */
  double sector_standoff_center_m = 18.0;
  /** Per-sector pitch/roll jitter (飞行丙; symmetric 甲 may set equal small values). */
  double sector_pitch_jitter_rad = 0.0;
  double sector_roll_jitter_rad = 0.0;
  /** 飞行丁: commanded yaw sweep span per sector [rad] (independent of orbit translation). */
  double sector_yaw_span_rad = 1.95 * M_PI;
  /**
   * 飞行戊 POI: scale roll on NE sector (sector 0); 1=full, 0=locked.
   * Set @p poi_roll_scale_all_sectors for diagnostic both-sector repro.
   */
  double poi_sector0_attitude_scale = 1.0;
  bool poi_roll_scale_all_sectors = false;
};

inline DualDiagonalFlightGeometry DiagonalDualLidarBaseGeometry() {
  DualDiagonalFlightGeometry g;
  g.label = "diagonal_dual_lidar";
  g.multilayer = true;
  g.flight_layers_m = {4.0, 8.0, 12.0};
  g.flight_dist_min_m = 14.0;
  g.flight_dist_max_m = 32.0;
  g.azimuth_span_deg = 120.0;
  g.layer_duration_s = 15.0;
  g.sector_duration_s = 45.0;
  g.rtk_dt_s = 0.5;
  g.lidar_dt_s = 0.5;
  g.camera_dt_s = 0.5;
  g.dual_preset.post_half_extent_m = 25.0;
  g.dual_preset.post_height_ne_m = 5.0;
  g.dual_preset.post_height_sw_m = 5.0;
  g.dual_preset.look_target_W = Eigen::Vector3d(0.0, 0.0, 6.0);
  return g;
}

/** Original Phase-3 (A) asymmetric post heights (root-cause repro only). */
inline DualDiagonalFlightGeometry DiagonalFlightA_LegacyAsymmetricGeometry() {
  DualDiagonalFlightGeometry g = DiagonalDualLidarBaseGeometry();
  g.label = "diagonal_flight_A_legacy_asymmetric";
  g.dual_flight_mode = DualDiagonalFlightMode::kConstantHeading;
  g.high_attitude_variation = false;
  g.dual_preset.post_height_ne_m = 4.5;
  g.dual_preset.post_height_sw_m = 5.5;
  return g;
}

/** 飞行甲: symmetric posts + constant route heading (coverage-matched sectors). */
inline DualDiagonalFlightGeometry DiagonalFlightA_Geometry() {
  DualDiagonalFlightGeometry g = DiagonalDualLidarBaseGeometry();
  g.label = "diagonal_flight_A_constant_heading";
  g.dual_flight_mode = DualDiagonalFlightMode::kConstantHeading;
  g.high_attitude_variation = false;
  g.sector_pitch_jitter_rad = 0.0;
  g.sector_roll_jitter_rad = 0.0;
  return g;
}

inline DualDiagonalFlightGeometry DiagonalFlightB_Geometry() {
  DualDiagonalFlightGeometry g = DiagonalDualLidarBaseGeometry();
  g.label = "diagonal_flight_B_high_aspect";
  g.dual_flight_mode = DualDiagonalFlightMode::kHighAspectPerLidar;
  g.high_attitude_variation = true;
  g.pitch_amp_rad = 0.40;
  g.roll_amp_rad = 0.25;
  g.azimuth_span_deg = 150.0;
  return g;
}

/**
 * 飞行丁: dual-sector translation + Phase-1.5 NearFieldHighAspect attitude on
 * sector-local time (heading decoupled from dual-orbit −p_WB; 乙 uses velocity lock).
 */
inline DualDiagonalFlightGeometry DiagonalFlightD_Geometry() {
  DualDiagonalFlightGeometry g = DiagonalDualLidarBaseGeometry();
  g.label = "diagonal_flight_D_per_segment_yaw_sweep";
  g.dual_flight_mode = DualDiagonalFlightMode::kPerSegmentYawSweep;
  g.azimuth_span_deg = 165.0;
  g.flight_dist_min_m = 18.0;
  g.flight_dist_max_m = 38.0;
  g.pitch_amp_rad = 0.40;
  g.roll_amp_rad = 0.25;
  return g;
}

/** 飞行丙: global route yaw fixed; each sector gets matched pitch/roll jitter. */
/**
 * 飞行戊: per-sector arc + nose locked toward that sector's LiDAR (DJI POI).
 * u_B azimuth ~constant; R_WB sweeps with orbit pitch/roll.
 */
inline DualDiagonalFlightGeometry DiagonalFlightE_Geometry() {
  DualDiagonalFlightGeometry g = DiagonalDualLidarBaseGeometry();
  g.label = "diagonal_flight_E_poi_tidal_lock";
  g.dual_flight_mode = DualDiagonalFlightMode::kPerSectorTidalLockPOI;
  g.azimuth_span_deg = 180.0;
  g.flight_dist_min_m = 18.0;
  g.flight_dist_max_m = 38.0;
  g.sector_yaw_span_rad = 1.95 * M_PI;
  g.high_attitude_variation = true;
  g.pitch_amp_rad = 0.40;
  g.roll_amp_rad = 0.25;
  g.poi_sector0_attitude_scale = 0.35;
  return g;
}

/**
 * 飞行戊 + temporal overlap: both LiDARs see the drone for @p sector_overlap_s
 * around the sector handoff (board-free does not require spatial FOV overlap).
 */
inline DualDiagonalFlightGeometry DiagonalFlightE_OverlapGeometry(
    double overlap_s = 10.0) {
  DualDiagonalFlightGeometry g = DiagonalFlightE_Geometry();
  g.label = "diagonal_flight_E_poi_overlap";
  g.sector_overlap_s = overlap_s;
  return g;
}

/** 飞行戊 @ Ruby 10 Hz (0.1 s lidar/rtk/camera dt). */
inline DualDiagonalFlightGeometry DiagonalFlightE_10Hz_Geometry() {
  DualDiagonalFlightGeometry g = DiagonalFlightE_Geometry();
  g.label = "diagonal_flight_E_poi_10hz";
  g.lidar_dt_s = 0.1;
  g.rtk_dt_s = 0.1;
  g.camera_dt_s = 0.1;
  return g;
}

/** Diagnostic: no POI roll tighten (roll_scale=1 on sector 0). */
inline DualDiagonalFlightGeometry DiagonalFlightE_NoPoiTight_Geometry() {
  DualDiagonalFlightGeometry g = DiagonalFlightE_Geometry();
  g.label = "diagonal_flight_E_poi_no_tight";
  g.poi_sector0_attitude_scale = 1.0;
  return g;
}

/** Diagnostic: fbe701f both-sector roll lock repro (not production default). */
inline DualDiagonalFlightGeometry DiagonalFlightE_BothSectorsTight_Geometry() {
  DualDiagonalFlightGeometry g = DiagonalFlightE_Geometry();
  g.label = "diagonal_flight_E_poi_both_sectors_tight";
  g.poi_roll_scale_all_sectors = true;
  return g;
}

inline DualDiagonalFlightGeometry DiagonalFlightC_Geometry() {
  DualDiagonalFlightGeometry g = DiagonalDualLidarBaseGeometry();
  g.label = "diagonal_flight_C_constant_heading_sector_jitter";
  g.dual_flight_mode = DualDiagonalFlightMode::kConstantHeadingSectorJitter;
  g.high_attitude_variation = false;
  g.sector_pitch_jitter_rad = 0.35;
  g.sector_roll_jitter_rad = 0.20;
  return g;
}

inline int SectorIndexFromTime(double t, const DualDiagonalFlightGeometry& geom) {
  return (t < geom.sector_duration_s - 1e-6) ? 0 : 1;
}

inline Eigen::Vector3d SectorOrbitCenterW(int sector,
                                         const DualDiagonalFlightGeometry& geom) {
  const double c = geom.sector_standoff_center_m;
  if (sector == 0) {
    return Eigen::Vector3d(c, c, 0.0);
  }
  return Eigen::Vector3d(-c, -c, 0.0);
}

/** UAV pose for dual-diagonal intersection (two sequential in-FOV sectors). */
inline SE3d PoseWbFromDualDiagonalGeometry(
    double t, const DualDiagonalFlightGeometry& geom) {
  const int sector = SectorIndexFromTime(t, geom);
  const double t_local =
      (sector == 0) ? t : (t - geom.sector_duration_s);
  const int n_layers =
      std::max(1, static_cast<int>(geom.flight_layers_m.size()));
  const double phase = t_local / geom.layer_duration_s;
  const int layer =
      std::min(n_layers - 1, static_cast<int>(phase));
  const double t_in_layer = t_local - layer * geom.layer_duration_s;
  const double u = t_in_layer / geom.layer_duration_s;

  const double az_span_rad = geom.azimuth_span_deg * M_PI / 180.0;
  const double az = az_span_rad * std::sin(2.0 * M_PI * u);
  const double dist_center =
      0.5 * (geom.flight_dist_min_m + geom.flight_dist_max_m);
  const double dist_amp =
      0.5 * (geom.flight_dist_max_m - geom.flight_dist_min_m);
  const double dist = dist_center + dist_amp * std::cos(2.0 * M_PI * u);

  const Eigen::Vector3d center = SectorOrbitCenterW(sector, geom);
  const double orbit_yaw = (sector == 0) ? (M_PI / 4.0) : (-3.0 * M_PI / 4.0);
  const double cx = dist * std::cos(az + orbit_yaw);
  const double cy = dist * std::sin(az + orbit_yaw);
  Eigen::Vector3d p_wb = center + Eigen::Vector3d(cx, cy, 0.0);
  if (geom.multilayer) {
    p_wb.z() = geom.flight_layers_m[static_cast<size_t>(layer)];
  } else {
    p_wb.z() = geom.coplanar_z_m;
  }

  SO3d R;
  if (geom.dual_flight_mode == DualDiagonalFlightMode::kPerSegmentYawSweep) {
    // Translation: dual-sector orbit (above). Attitude: Phase-1.5 NearFieldHighAspect
    // law on sector-local time — heading NOT locked to −p_WB of this orbit.
    const SyntheticFlightGeometry p15_att =
        NearFieldHighAspectScenarioGeometry();
    R = PoseWbFromGeometry(t_local, p15_att).so3();
  } else if (geom.dual_flight_mode ==
             DualDiagonalFlightMode::kPerSectorTidalLockPOI) {
    const std::string active_lidar =
        (sector == 0) ? "lidar_NE" : "lidar_SW";
    const Eigen::Vector3d post =
        DiagonalLidarPostW(geom.dual_preset, active_lidar);
    Eigen::Vector3d to_lidar = post - p_wb;
    const double yaw_poi = std::atan2(to_lidar.y(), to_lidar.x());
    R = SO3d::rotZ(yaw_poi);
    if (geom.high_attitude_variation) {
      double roll_scale = 1.0;
      const bool tighten_roll =
          geom.poi_roll_scale_all_sectors || (sector == 0);
      if (tighten_roll && geom.poi_sector0_attitude_scale >= 0.0) {
        roll_scale = geom.poi_sector0_attitude_scale;
      }
      R = R * SO3d::rotY(geom.pitch_amp_rad * std::sin(2.0 * M_PI * u)) *
              SO3d::rotX(roll_scale * geom.roll_amp_rad *
                         std::cos(2.0 * M_PI * u));
    }
  } else if (geom.dual_flight_mode == DualDiagonalFlightMode::kConstantHeading ||
             geom.dual_flight_mode ==
                 DualDiagonalFlightMode::kConstantHeadingSectorJitter) {
    R = SO3d::rotZ(geom.route_yaw_rad);
    if (geom.dual_flight_mode == DualDiagonalFlightMode::kConstantHeadingSectorJitter ||
        geom.sector_pitch_jitter_rad > 1e-9 ||
        geom.sector_roll_jitter_rad > 1e-9) {
      R = R * SO3d::rotY(geom.sector_pitch_jitter_rad *
                         std::sin(2.0 * M_PI * u)) *
              SO3d::rotX(geom.sector_roll_jitter_rad *
                         std::cos(2.0 * M_PI * u));
    }
  } else {
    // 飞行乙: heading locked to horizontal −p_WB (velocity-pointing).
    const Eigen::Vector3d to_center = -p_wb;
    const double yaw =
        std::atan2(to_center.y(), to_center.x());
    R = SO3d::rotZ(yaw);
    if (geom.high_attitude_variation) {
      R = R * SO3d::rotY(geom.pitch_amp_rad * std::sin(2.0 * M_PI * u)) *
          SO3d::rotX(geom.roll_amp_rad * std::cos(2.0 * M_PI * u));
    }
  }
  return SE3d(R, p_wb);
}

inline BodyTrajectory BuildGtTrajectoryDualDiagonal(
    const DualDiagonalFlightGeometry& geom) {
  const double t_end = 2.0 * geom.sector_duration_s;
  BodyTrajectory traj(0.05, 0.0);
  const int num_knots =
      std::max(24, static_cast<int>(std::ceil(t_end / 0.05)) + 4);
  traj.setKnots(PoseWbFromDualDiagonalGeometry(0.0, geom), num_knots);
  for (int i = 0; i < num_knots; ++i) {
    const double t = std::min(t_end, static_cast<double>(i) * 0.05);
    traj.setKnot(PoseWbFromDualDiagonalGeometry(t, geom), i);
  }
  const int64_t t_end_ns = static_cast<int64_t>(t_end * S_TO_NS);
  traj.extendKnotsTo(t_end_ns, traj.getKnotSO3(num_knots - 1),
                     traj.getKnotPos(num_knots - 1));
  return traj;
}

inline bool IsTimeInSensorSector(double t_world, const std::string& sensor_key,
                                 const DualDiagonalFlightGeometry& geom) {
  const double t_split = geom.sector_duration_s;
  const double ov = std::max(0.0, geom.sector_overlap_s);
  if (sensor_key == "lidar_NE") {
    return t_world < t_split + ov - 1e-6;
  }
  if (sensor_key == "lidar_SW") {
    return t_world >= t_split - ov - 1e-6;
  }
  return false;
}

inline bool IsDroneVisibleFromPost(const Eigen::Vector3d& drone_W,
                                   const Eigen::Vector3d& post_W,
                                   double min_cos_fov = 0.25) {
  Eigen::Vector3d dir = drone_W - post_W;
  const double range = dir.norm();
  if (range < 5.0 || range > 90.0) {
    return false;
  }
  dir /= range;
  const SE3d T_LW = T_LWFromRoadsidePost(post_W, drone_W);
  const Eigen::Vector3d drone_L = T_LW * drone_W;
  return drone_L.x() > 0.5 && drone_L.x() / drone_L.norm() > min_cos_fov;
}

}  // namespace experiments
}  // namespace clic_calib
