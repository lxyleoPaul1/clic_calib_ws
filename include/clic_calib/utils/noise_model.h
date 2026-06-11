/*
 * clic_calib — unified observation noise model (simulator + factor whitening).
 */

#pragma once

#include <clic_calib/sensor_data/body_cluster_observation.h>

#include <Eigen/Core>

#include <iostream>
#include <random>
#include <string>

namespace clic_calib {

/** @brief Diagonal Gaussian noise shared by simulators and Ceres factor weights. */
struct NoiseModel {
  double rtk_sigma_horizontal_m = 0.01;
  double rtk_sigma_vertical_m = 0.02;
  double lidar_ranging_sigma_m = 0.02;
  double camera_pixel_sigma = 1.0;

  /** PSDK attitude tangent σ [deg] — Σ_att = diag(σ_roll², σ_pitch², σ_yaw²).
   *  Defaults (0.2/0.2/1.5°) match synthetic; replace with hover-measured STD
   *  in config/noise_model.yaml before field calibration. */
  double attitude_sigma_roll_deg = 0.2;
  double attitude_sigma_pitch_deg = 0.2;
  double attitude_sigma_yaw_deg = 1.5;

  /** Board-free body centroid noise (config/noise_model.yaml body_centroid). */
  double body_centroid_sigma_base_m = 0.05;
  double body_centroid_range_coeff = 0.001;
  int body_centroid_min_points_for_valid = 10;

  static NoiseModel FromYaml(const std::string& path);
  static NoiseModel FromConfigDir(const std::string& config_dir);

  /** RTK position covariance [m²] used by RTKPositionFactor whitening. */
  Eigen::Matrix3d RtkPositionCovariance() const;

  /** Body-tangent attitude covariance [rad²] for AttitudeFactorPoseForm. */
  Eigen::Matrix3d AttitudeTangentCovarianceRad2() const;

  Eigen::Vector3d SampleRtkNoise(std::mt19937& rng) const;
  double SampleLidarRangeNoise(std::mt19937& rng) const;
  Eigen::Vector2d SamplePixelNoise(std::mt19937& rng) const;

  /**
   * @brief Isotropic body-centroid σ [m] vs mean range (linear growth model).
   * σ(r) = sigma_base + range_coeff * r; invalid if point_count too low → inf.
   */
  double BodyCentroidSigmaM(double mean_range_m, int point_count) const;

  /** @brief Diagonal covariance [m²] for body-centroid residual whitening. */
  Eigen::Matrix3d BodyCentroidCovariance(double mean_range_m,
                                          int point_count) const;

  /** @brief Cholesky sqrt information Σ^{-1/2} (3×3) for Ceres whitening. */
  Eigen::Matrix3d BodyCentroidSqrtInformation(double mean_range_m,
                                              int point_count) const;

  /**
   * Whitening from v2 @p centroid_cov when present; else legacy range model.
   * Simulated cov is isotropic (σ² I) with σ ∝ 1/√N.
   */
  Eigen::Matrix3d BodyCentroidSqrtInformationFromObservation(
      const BodyClusterObservation& obs) const;

  void Log(std::ostream& os = std::cout) const;
};

}  // namespace clic_calib
