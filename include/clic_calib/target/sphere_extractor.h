/*
 * clic_calib — LiDAR reflective-sphere target extraction (Phase 4 / patent S4).
 * Outputs raw inlier points for §4.3 SphereImplicitFactor (not fitted center).
 */

#pragma once

#include <clic_calib/sensor_data/lidar_target_observation.h>
#include <clic_calib/target/sphere_center_fit.h>

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace clic_calib {

/** @brief PCL pipeline: intensity filter → ROI → cluster → RANSAC sphere. */
class SphereExtractor {
 public:
  struct Options {
    double intensity_min = 180.0;
    double cluster_tolerance_m = 0.3;
    int min_cluster_size = 30;
    double radius_prior_m = 0.10;
    double ransac_inlier_threshold_m = 0.02;
    double min_inlier_ratio = 0.80;
    double max_radius_deviation_ratio = 0.15;
    /** ROI half-extent [m] around @p roi_center_prior_l when provided. */
    double roi_half_extent_m = 0.8;
  };

  SphereExtractor() : SphereExtractor(Options{}) {}
  explicit SphereExtractor(const Options& options);

  /**
   * @brief Fit sphere center from inlier points (same routine as Umeyama init).
   * SphereExtractor::Extract() produces @p points_L_; pass them here for init.
   */
  static bool FitCenterFromPoints(const std::vector<Eigen::Vector3d>& points_l,
                                  double radius_m, Eigen::Vector3d* center,
                                  double* fit_rms_m = nullptr) {
    return sphere_center::FitSphereCenterKnownRadius(points_l, radius_m, center,
                                                     fit_rms_m);
  }

  /**
   * @brief Extract sphere target points from a LiDAR scan.
   * @param roi_center_prior_l Optional rough sphere center in LiDAR frame (RTK extrapolation / last frame).
   * @param fitted_center_l Optional RANSAC center for diagnostics / next-frame prior.
   * @return false if no cluster passes validation.
   */
  bool Extract(const pcl::PointCloud<pcl::PointXYZI>& cloud, double timestamp_s,
               int sensor_id,
               const Eigen::Vector3d* roi_center_prior_l,
               LiDARTargetObservation* observation,
               Eigen::Vector3d* fitted_center_l = nullptr) const;

 private:
  Options options_;
};

}  // namespace clic_calib
