#pragma once

/**
 * Parameterized synthetic flight + roadside sensor geometry for FIM / degeneracy tests.
 * See doc/diagnostics/geometry_sanity_check.md.
 */

#include <clic_calib/sensor_data/apriltag_observation.h>
#include <clic_calib/sensor_data/lidar_target_observation.h>
#include <clic_calib/sensor_data/rtk_measurement.h>
#include <clic_calib/spline/trajectory.h>
#include <clic_calib/utils/camera_projection.h>
#include <clic_calib/utils/lever_arm.h>
#include <clic_calib/utils/noise_model.h>
#include <clic_calib/utils/sophus_utils.hpp>

#include <algorithm>
#include <cmath>
#include <random>
#include <string>
#include <vector>

namespace clic_calib {
namespace experiments {

struct SyntheticFlightGeometry {
  std::string label = "unnamed";
  /** Roadside pole / sensor origin height in world W [m]. */
  double sensor_height_m = 6.0;
  /** UAV standoff from pole in world XY [m]. */
  double flight_dist_min_m = 15.0;
  double flight_dist_max_m = 40.0;
  /** Per-layer altitudes [m] (multi-layer); ignored when coplanar. */
  std::vector<double> flight_layers_m = {2.0, 6.0, 11.0};
  /** Half-span of azimuth sweep per layer [deg]. */
  double azimuth_span_deg = 30.0;
  /** Degeneracy eval target range along +X [m] (not used in FIM assembly). */
  double eval_target_dist_m = 60.0;
  double layer_duration_s = 40.0;
  double rtk_dt_s = 0.2;
  double lidar_dt_s = 0.1;
  double camera_dt_s = 0.1;
  bool multilayer = true;
  /** Coplanar: fixed z for all samples [m]. */
  double coplanar_z_m = 10.0;
  /** Legacy local / 200 m standoff modes. */
  bool use_legacy_local_pose = false;
  bool use_legacy_200m_pose = false;
  double legacy_range_m = 200.0;
  /** Extra pitch/roll on near-field poses for aspect-diversity experiments. */
  bool high_attitude_variation = false;
  double pitch_amp_rad = 0.35;
  double roll_amp_rad = 0.20;
};

inline SyntheticFlightGeometry LegacyLocalMultiLayerGeometry() {
  SyntheticFlightGeometry g;
  g.label = "legacy_local_multilayer";
  g.use_legacy_local_pose = true;
  g.multilayer = true;
  g.layer_duration_s = 5.0;
  g.rtk_dt_s = 0.1;
  g.lidar_dt_s = 0.4;
  g.camera_dt_s = 0.35;
  return g;
}

inline SyntheticFlightGeometry LegacyLocalCoplanarGeometry() {
  SyntheticFlightGeometry g = LegacyLocalMultiLayerGeometry();
  g.label = "legacy_local_coplanar";
  g.multilayer = false;
  return g;
}

inline SyntheticFlightGeometry Legacy200mMultiLayerGeometry() {
  SyntheticFlightGeometry g;
  g.label = "legacy_200m_multilayer";
  g.use_legacy_200m_pose = true;
  g.legacy_range_m = 200.0;
  g.multilayer = true;
  g.sensor_height_m = 0.5;
  g.layer_duration_s = 5.0;
  g.rtk_dt_s = 0.1;
  g.lidar_dt_s = 0.4;
  g.camera_dt_s = 0.35;
  g.flight_layers_m = {10.0, 10.0, 10.0};
  return g;
}

inline SyntheticFlightGeometry Legacy200mCoplanarGeometry() {
  SyntheticFlightGeometry g = Legacy200mMultiLayerGeometry();
  g.label = "legacy_200m_coplanar";
  g.multilayer = false;
  g.coplanar_z_m = 10.0;
  return g;
}

inline SyntheticFlightGeometry NearFieldMultiLayerGeometry() {
  SyntheticFlightGeometry g;
  g.label = "nearfield_multilayer";
  g.sensor_height_m = 6.0;
  g.flight_dist_min_m = 15.0;
  g.flight_dist_max_m = 40.0;
  g.flight_layers_m = {2.0, 6.0, 11.0};
  g.azimuth_span_deg = 30.0;
  g.eval_target_dist_m = 60.0;
  g.layer_duration_s = 40.0;
  g.rtk_dt_s = 0.2;
  g.lidar_dt_s = 0.1;
  g.camera_dt_s = 0.1;
  g.multilayer = true;
  return g;
}

inline SyntheticFlightGeometry NearFieldCoplanarGeometry() {
  SyntheticFlightGeometry g = NearFieldMultiLayerGeometry();
  g.label = "nearfield_coplanar";
  g.multilayer = false;
  g.coplanar_z_m = 6.0;
  return g;
}

/** Sensor at world (0,0,h), nominal horizontal view toward +X. */
inline void SensorExtrinsicsFromGeometry(const SyntheticFlightGeometry& geom,
                                         SE3d* T_LW, SE3d* T_CW) {
  const double h = geom.sensor_height_m;
  if (geom.use_legacy_local_pose || geom.use_legacy_200m_pose) {
    *T_LW = SE3d(SO3d::rotY(-0.15), Eigen::Vector3d(3.0, -1.0, 0.5));
    *T_CW = SE3d(SO3d::rotX(0.1), Eigen::Vector3d(2.0, 1.5, 0.2));
    return;
  }
  const SE3d T_WL(SO3d(Eigen::Quaterniond::Identity()), Eigen::Vector3d(0.0, 0.0, h));
  *T_LW = T_WL.inverse();
  *T_CW = SE3d(SO3d::rotX(0.1), Eigen::Vector3d(0.0, 0.0, -h));
}

inline SE3d PoseWbFromGeometry(double t, const SyntheticFlightGeometry& geom) {
  if (geom.use_legacy_local_pose) {
    const double s = t;
    SO3d R = SO3d::rotZ(0.05 * s);
    double z = 2.0;
    if (geom.multilayer) {
      R = R * SO3d::rotY(0.12 * std::sin(s));
      z = 2.0 + 0.4 * std::sin(s);
    }
    return SE3d(R, Eigen::Vector3d(0.5 * s, 0.3 * std::sin(s), z));
  }
  if (geom.use_legacy_200m_pose) {
    const double s = t;
    SO3d R = SO3d::rotZ(0.05 * s);
    double z = geom.coplanar_z_m;
    if (geom.multilayer) {
      R = R * SO3d::rotY(0.12 * std::sin(s));
      z = 10.0 + 4.0 * std::sin(s);
    }
    const double Rm = geom.legacy_range_m;
    return SE3d(R, Eigen::Vector3d(Rm + 0.5 * s, 0.3 * std::sin(s), z));
  }

  const int n_layers =
      std::max(1, static_cast<int>(geom.flight_layers_m.size()));
  const double t_total = n_layers * geom.layer_duration_s;
  const double t_wrap = std::min(std::max(t, 0.0), t_total - 1e-6);
  const int layer =
      std::min(n_layers - 1,
               static_cast<int>(t_wrap / geom.layer_duration_s));
  const double t_local = t_wrap - layer * geom.layer_duration_s;
  const double phase = t_local / geom.layer_duration_s;

  const double az_span_rad = geom.azimuth_span_deg * M_PI / 180.0;
  const double az = az_span_rad * std::sin(2.0 * M_PI * phase);
  const double dist_center =
      0.5 * (geom.flight_dist_min_m + geom.flight_dist_max_m);
  const double dist_amp =
      0.5 * (geom.flight_dist_max_m - geom.flight_dist_min_m);
  const double dist = dist_center + dist_amp * std::cos(2.0 * M_PI * phase);

  const double x = dist * std::cos(az);
  const double y = dist * std::sin(az);
  double z = geom.coplanar_z_m;
  if (geom.multilayer) {
    z = geom.flight_layers_m[static_cast<size_t>(layer)];
  }

  const Eigen::Vector3d p_wb(x, y, z);
  const Eigen::Vector3d to_sensor = -p_wb;
  const double yaw =
      std::atan2(to_sensor.y(), to_sensor.x());
  SO3d R = SO3d::rotZ(yaw);
  if (geom.high_attitude_variation) {
    R = R * SO3d::rotY(geom.pitch_amp_rad * std::sin(2.0 * M_PI * phase)) *
        SO3d::rotX(geom.roll_amp_rad * std::cos(2.0 * M_PI * phase));
  }
  return SE3d(R, p_wb);
}

inline BodyTrajectory BuildGtTrajectoryFromGeometry(
    const SyntheticFlightGeometry& geom) {
  const int n_layers =
      std::max(1, static_cast<int>(geom.flight_layers_m.size()));
  const double t_end = n_layers * geom.layer_duration_s;
  BodyTrajectory traj(0.05, 0.0);
  const int num_knots =
      std::max(24, static_cast<int>(std::ceil(t_end / 0.05)) + 4);
  traj.setKnots(PoseWbFromGeometry(0.0, geom), num_knots);
  for (int i = 0; i < num_knots; ++i) {
    const double t = std::min(t_end, static_cast<double>(i) * 0.05);
    traj.setKnot(PoseWbFromGeometry(t, geom), i);
  }
  const int64_t t_end_ns = static_cast<int64_t>(t_end * S_TO_NS);
  traj.extendKnotsTo(t_end_ns, traj.getKnotSO3(num_knots - 1),
                     traj.getKnotPos(num_knots - 1));
  return traj;
}

struct PitchSpanAudit {
  double min_elev_deg = 0.0;
  double max_elev_deg = 0.0;
  double span_deg = 0.0;
  double mean_horiz_m = 0.0;
};

/** Elevation at sensor: atan2(z_sensor - z_uav, d_horiz). */
inline PitchSpanAudit ComputePitchSpanAtSensor(
    const SyntheticFlightGeometry& geom, double t0, double t1, double dt) {
  SE3d T_LW, T_CW;
  SensorExtrinsicsFromGeometry(geom, &T_LW, &T_CW);
  const Eigen::Vector3d p_sensor_W = T_LW.inverse().translation();

  PitchSpanAudit out;
  double min_e = 1e9;
  double max_e = -1e9;
  double sum_h = 0.0;
  int n = 0;
  for (double t = t0; t <= t1 + 1e-9; t += dt) {
    const SE3d T_WB = PoseWbFromGeometry(t, geom);
    const Eigen::Vector3d p = T_WB.translation();
    const Eigen::Vector3d delta = p - p_sensor_W;
    const double d_h = std::hypot(delta.x(), delta.y());
    const double elev_rad =
        std::atan2(-delta.z(), std::max(d_h, 1e-6));
    const double elev_deg = elev_rad * 180.0 / M_PI;
    min_e = std::min(min_e, elev_deg);
    max_e = std::max(max_e, elev_deg);
    sum_h += d_h;
    ++n;
  }
  out.min_elev_deg = min_e;
  out.max_elev_deg = max_e;
  out.span_deg = max_e - min_e;
  out.mean_horiz_m = sum_h / std::max(n, 1);
  return out;
}

}  // namespace experiments
}  // namespace clic_calib
