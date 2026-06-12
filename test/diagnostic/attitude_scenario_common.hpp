#pragma once

#include "experiments/noise_regime_common.hpp"
#include "experiments/synthetic_flight_geometry.hpp"

#include <clic_calib/sensor_data/attitude_observation.h>

#include <cmath>
#include <random>

namespace clic_calib {
namespace experiments {

/** PSDK-style anisotropic attitude noise (body tangent, degrees). */
struct AttitudeNoiseSpec {
  double sigma_roll_deg = 0.2;
  double sigma_pitch_deg = 0.2;
  double sigma_yaw_deg = 1.5;
  double sample_hz = 50.0;

  Eigen::Matrix3d TangentCovarianceRad2() const {
    const double sr = sigma_roll_deg * M_PI / 180.0;
    const double sp = sigma_pitch_deg * M_PI / 180.0;
    const double sy = sigma_yaw_deg * M_PI / 180.0;
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    cov(0, 0) = sr * sr;
    cov(1, 1) = sp * sp;
    cov(2, 2) = sy * sy;
    return cov;
  }
};

inline void AppendAttitudeObservationsForDuration(
    SyntheticScenarioBundle* bundle, double t_end,
    const AttitudeNoiseSpec& att_noise, std::mt19937* rng) {
  if (!bundle || !rng || t_end <= 0.0) {
    return;
  }
  const double dt = 1.0 / att_noise.sample_hz;
  const Eigen::Matrix3d cov = att_noise.TangentCovarianceRad2();
  std::normal_distribution<double> normal(0.0, 1.0);

  bundle->attitude_obs.clear();
  bundle->attitude_obs.reserve(
      static_cast<size_t>(std::ceil(t_end / dt)) + 4);

  for (double t = dt; t <= t_end - 1e-9; t += dt) {
    AttitudeObservation obs;
    obs.t_world_ = t;
    obs.covariance_ = cov;
    const SO3d R_gt = bundle->gt_traj.rotation_wb(t);
    Eigen::Vector3d n(normal(*rng), normal(*rng), normal(*rng));
    n(0) *= std::sqrt(cov(0, 0));
    n(1) *= std::sqrt(cov(1, 1));
    n(2) *= std::sqrt(cov(2, 2));
    obs.R_WB_observed_ = R_gt * SO3d::exp(n);
    bundle->attitude_obs.push_back(obs);
  }
}

inline void AppendAttitudeObservations(SyntheticScenarioBundle* bundle,
                                       const SyntheticFlightGeometry& geom,
                                       const AttitudeNoiseSpec& att_noise,
                                       std::mt19937* rng) {
  if (!bundle || !rng) {
    return;
  }
  const int n_layers =
      std::max(1, static_cast<int>(geom.flight_layers_m.size()));
  const double t_end = geom.use_legacy_local_pose || geom.use_legacy_200m_pose
                           ? 5.0
                           : n_layers * geom.layer_duration_s;
  AppendAttitudeObservationsForDuration(bundle, t_end, att_noise, rng);
}

inline SyntheticScenarioBundle BuildNearFieldFimNoisyScenarioWithAttitude(
    uint32_t seed_traj, uint32_t seed_obs, const RealisticNoiseSpec& noise,
    const AttitudeNoiseSpec& att_noise = AttitudeNoiseSpec()) {
  SyntheticScenarioBundle bundle = BuildNoisyScenarioFromGeometry(
      seed_traj, seed_obs, noise, NearFieldFimScenarioGeometry());
  std::mt19937 rng(seed_traj + 7919u);
  AppendAttitudeObservations(&bundle, NearFieldFimScenarioGeometry(), att_noise,
                             &rng);
  return bundle;
}

inline SyntheticScenarioBundle BuildNearFieldFimNoisyScenarioWithAttitude(
    uint32_t seed, const RealisticNoiseSpec& noise,
    const AttitudeNoiseSpec& att_noise = AttitudeNoiseSpec()) {
  return BuildNearFieldFimNoisyScenarioWithAttitude(seed, seed, noise, att_noise);
}

inline SyntheticScenarioBundle BuildNearFieldCoplanarFimNoisyScenarioWithAttitude(
    uint32_t seed, const RealisticNoiseSpec& noise,
    const AttitudeNoiseSpec& att_noise = AttitudeNoiseSpec()) {
  SyntheticScenarioBundle bundle =
      BuildNearFieldCoplanarFimNoisyScenario(seed, noise);
  SyntheticFlightGeometry g = NearFieldMultiLayerGeometry();
  g.multilayer = false;
  g.coplanar_z_m = 6.0;
  g.layer_duration_s = 15.0;
  g.rtk_dt_s = 0.5;
  std::mt19937 rng(seed + 7919u);
  AppendAttitudeObservations(&bundle, g, att_noise, &rng);
  return bundle;
}

/** Attitude observation error vs GT [deg] in body tangent (roll, pitch, yaw). */
inline Eigen::Vector3d AttitudeObsErrorDeg(const AttitudeObservation& obs,
                                           const BodyTrajectory& gt_traj) {
  const SO3d R_gt = gt_traj.rotation_wb(obs.t_world_);
  const SO3d R_err = R_gt.inverse() * obs.R_WB_observed_;
  return R_err.log() * 180.0 / M_PI;
}

}  // namespace experiments
}  // namespace clic_calib
