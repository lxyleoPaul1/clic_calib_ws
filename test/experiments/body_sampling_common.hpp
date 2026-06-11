#pragma once

#include "experiments/synthetic_flight_geometry.hpp"

#include <clic_calib/sensor_data/body_cluster_observation.h>
#include <clic_calib/spline/trajectory.h>
#include <clic_calib/utils/lever_arm.h>
#include <clic_calib/utils/noise_model.h>
#include <clic_calib/utils/sophus_utils.hpp>

#include <cmath>
#include <random>
#include <vector>

namespace clic_calib {
namespace experiments {

/** Box half-extents [m] in body frame (matches simulate_uav_trajectory.py). */
inline constexpr double kBodyHalfExtent[3] = {0.25, 0.25, 0.12};

struct BodySampleResult {
  std::vector<Eigen::Vector3d> points_L;
  Eigen::Vector3d empirical_centroid_L = Eigen::Vector3d::Zero();
  int visible_faces = 0;
};

struct BoxLidarRangeAudit {
  double range_m = 0.0;
  int point_count = 0;
  int visible_faces = 0;
  Eigen::Vector3d empirical_centroid_L = Eigen::Vector3d::Zero();
  Eigen::Vector3d bias_vs_nominal_L = Eigen::Vector3d::Zero();
};

/**
 * Visible-hull sampling in LiDAR frame L: back-face cull, partial occlusion,
 * forward depth gate. Centroid is the empirical mean of kept points (no GT offset).
 */
inline BodySampleResult SampleBodyPointsDeterministic(
    const SO3d& R_wb, const Eigen::Vector3d& p_wb, const SE3d& T_LW,
    const Eigen::Vector3d& L_B_to_body, std::mt19937* rng,
    bool apply_occlusion_dropout) {
  BodySampleResult out;
  const Eigen::Vector3d p_body_w = p_wb + R_wb * L_B_to_body;
  const SE3d T_WL = T_LW.inverse();
  const Eigen::Vector3d lidar_origin_w = T_WL * Eigen::Vector3d::Zero();
  Eigen::Vector3d view_dir_w = p_body_w - lidar_origin_w;
  const double view_norm = view_dir_w.norm();
  if (view_norm < 1e-9) {
    return out;
  }
  view_dir_w /= view_norm;

  std::uniform_real_distribution<double> uni(0.0, 1.0);
  for (int axis = 0; axis < 3; ++axis) {
    for (double sign : {-1.0, 1.0}) {
      Eigen::Vector3d normal_b = Eigen::Vector3d::Zero();
      normal_b[axis] = sign;
      const Eigen::Vector3d normal_w = R_wb * normal_b;
      const double cos_theta = normal_w.dot(view_dir_w);
      if (cos_theta <= 0.05) {
        continue;
      }
      ++out.visible_faces;
      const double visible_frac = std::min(1.0, std::max(0.2, cos_theta));
      const int grid_n = std::max(3, static_cast<int>(8.0 * visible_frac));
      for (int iu = 0; iu < grid_n; ++iu) {
        for (int iv = 0; iv < grid_n; ++iv) {
          const double u = -1.0 + 2.0 * static_cast<double>(iu) /
                                       std::max(grid_n - 1, 1);
          const double v = -1.0 + 2.0 * static_cast<double>(iv) /
                                       std::max(grid_n - 1, 1);
          if (apply_occlusion_dropout && rng &&
              uni(*rng) > std::pow(cos_theta, 1.5)) {
            continue;
          }
          Eigen::Vector3d local = Eigen::Vector3d::Zero();
          if (axis == 0) {
            local = Eigen::Vector3d(sign * kBodyHalfExtent[0],
                                    u * kBodyHalfExtent[1],
                                    v * kBodyHalfExtent[2]);
          } else if (axis == 1) {
            local = Eigen::Vector3d(u * kBodyHalfExtent[0],
                                    sign * kBodyHalfExtent[1],
                                    v * kBodyHalfExtent[2]);
          } else {
            local = Eigen::Vector3d(u * kBodyHalfExtent[0],
                                    v * kBodyHalfExtent[1],
                                    sign * kBodyHalfExtent[2]);
          }
          const Eigen::Vector3d p_w = p_wb + R_wb * (L_B_to_body + local);
          const Eigen::Vector3d p_l = T_LW * p_w;
          if (p_l.z() > 0.5) {
            out.points_L.push_back(p_l);
          }
        }
      }
    }
  }
  if (out.points_L.size() < 8) {
    out.points_L.clear();
    return out;
  }
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  for (const auto& p : out.points_L) {
    sum += p;
  }
  out.empirical_centroid_L = sum / static_cast<double>(out.points_L.size());
  return out;
}

inline BodySampleResult SampleBodyPointsLidar(
    const SO3d& R_wb, const Eigen::Vector3d& p_wb, const SE3d& T_LW,
    const Eigen::Vector3d& L_B_to_body, std::mt19937* rng) {
  return SampleBodyPointsDeterministic(R_wb, p_wb, T_LW, L_B_to_body, rng,
                                       true);
}

/** Nominal lever-arm body centroid in LiDAR frame (for bias diagnostics). */
inline Eigen::Vector3d NominalBodyCentroidL(const SO3d& R_wb,
                                            const Eigen::Vector3d& p_wb,
                                            const SE3d& T_LW,
                                            const Eigen::Vector3d& L_B_to_body) {
  const Eigen::Vector3d p_body_w = p_wb + R_wb * L_B_to_body;
  return T_LW * p_body_w;
}

/** Visible-hull audit at standoff range (sensor-side geometry). */
inline BoxLidarRangeAudit Audit128LineBoxAtRange(
    double range_m, const SE3d& T_LW, const Eigen::Vector3d& L_B_to_body,
    const double half_extent[3]) {
  BoxLidarRangeAudit audit;
  audit.range_m = range_m;
  const Eigen::Vector3d p_wb(range_m, 0.0, 10.0);
  const Eigen::Vector3d to_sensor = -p_wb;
  const double yaw = std::atan2(to_sensor.y(), to_sensor.x());
  const SO3d R_wb = SO3d::rotZ(yaw);

  BodySampleResult sample = SampleBodyPointsDeterministic(
      R_wb, p_wb, T_LW, L_B_to_body, nullptr, false);
  audit.point_count = static_cast<int>(sample.points_L.size());
  audit.visible_faces = sample.visible_faces;
  audit.empirical_centroid_L = sample.empirical_centroid_L;
  audit.bias_vs_nominal_L =
      sample.empirical_centroid_L -
      NominalBodyCentroidL(R_wb, p_wb, T_LW, L_B_to_body);
  (void)half_extent;
  return audit;
}

inline BodyClusterObservation MakeBodyClusterObservation(
    double t_sensor, const BodySampleResult& sample, const NoiseModel& noise,
    std::mt19937* rng, bool add_residual_noise = true) {
  BodyClusterObservation obs;
  obs.t_sensor_ = t_sensor;
  obs.sensor_key_ = "0";
  obs.sensor_id_ = 0;
  obs.raw_points_L_ = sample.points_L;
  obs.point_count_ = static_cast<std::uint32_t>(sample.points_L.size());
  double mean_range = 0.0;
  for (const auto& p : sample.points_L) {
    mean_range += p.norm();
  }
  mean_range /= static_cast<double>(sample.points_L.size());
  obs.mean_range_m_ = mean_range;
  obs.centroid_L_ = sample.empirical_centroid_L;
  if (obs.point_count_ > 0) {
    // v2: Var(c̄) = σ_r²/N_pts (σ_r = ranging σ; N_pts per scan, frame-rate invariant).
    const double sigma_r = noise.lidar_ranging_sigma_m;
    const double n_pts = static_cast<double>(obs.point_count_);
    obs.has_centroid_cov_ = true;
    obs.centroid_cov_ = Eigen::Matrix3d::Identity() * (sigma_r * sigma_r / n_pts);
  }
  if (add_residual_noise && rng) {
    const double sigma =
        noise.BodyCentroidSigmaM(mean_range, static_cast<int>(obs.point_count_));
    std::normal_distribution<double> normal(0.0, 1.0);
    for (int k = 0; k < 3; ++k) {
      obs.centroid_L_[k] += normal(*rng) * sigma;
    }
  }
  return obs;
}

inline std::vector<BodyClusterObservation> BuildBodyClusterObsForGeometry(
    const BodyTrajectory& traj, const SE3d& T_LW, double t_d_L_s,
    const LeverArmConfig& levers, const NoiseModel& noise,
    const SyntheticFlightGeometry& geom, uint32_t seed_obs,
    bool add_residual_noise = true) {
  const int n_layers =
      std::max(1, static_cast<int>(geom.flight_layers_m.size()));
  const double t_end = geom.use_legacy_local_pose || geom.use_legacy_200m_pose
                           ? 5.0
                           : n_layers * geom.layer_duration_s;

  std::vector<BodyClusterObservation> out;
  std::mt19937 rng(seed_obs);
  for (double t = geom.lidar_dt_s; t <= t_end - 1e-9; t += geom.lidar_dt_s) {
    const SE3d T_WB = traj.pose_wb(t);
    const BodySampleResult sample = SampleBodyPointsLidar(
        T_WB.so3(), T_WB.translation(), T_LW, levers.L_B_to_body_centroid, &rng);
    if (sample.points_L.size() < 10) {
      continue;
    }
    out.push_back(MakeBodyClusterObservation(t + t_d_L_s, sample, noise, &rng,
                                             add_residual_noise));
  }
  return out;
}

}  // namespace experiments
}  // namespace clic_calib
