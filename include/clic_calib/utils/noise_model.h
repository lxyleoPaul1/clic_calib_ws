/*
 * clic_calib — unified observation noise model (simulator + factor whitening).
 */

#pragma once

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

  static NoiseModel FromYaml(const std::string& path);
  static NoiseModel FromConfigDir(const std::string& config_dir);

  /** RTK position covariance [m²] used by RTKPositionFactor whitening. */
  Eigen::Matrix3d RtkPositionCovariance() const;

  Eigen::Vector3d SampleRtkNoise(std::mt19937& rng) const;
  double SampleLidarRangeNoise(std::mt19937& rng) const;
  Eigen::Vector2d SamplePixelNoise(std::mt19937& rng) const;

  void Log(std::ostream& os = std::cout) const;
};

}  // namespace clic_calib
