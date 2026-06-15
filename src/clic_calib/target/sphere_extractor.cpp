#include <clic_calib/target/sphere_extractor.h>

#include <pcl/common/copy_point.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <algorithm>
#include <cmath>

namespace clic_calib {

SphereExtractor::SphereExtractor(const Options& options) : options_(options) {}

bool SphereExtractor::Extract(const pcl::PointCloud<pcl::PointXYZI>& cloud,
                              double timestamp_s, int sensor_id,
                              const Eigen::Vector3d* roi_center_prior_l,
                              LiDARTargetObservation* observation,
                              Eigen::Vector3d* fitted_center_l) const {
  if (!observation) {
    return false;
  }
  observation->t_sensor_ = timestamp_s;
  observation->sensor_id_ = sensor_id;
  observation->points_L_.clear();
  observation->per_point_dt_.clear();

  if (cloud.empty()) {
    return false;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>);
  filtered->reserve(cloud.size());
  for (const auto& pt : cloud.points) {
    if (pt.intensity >= options_.intensity_min) {
      filtered->push_back(pt);
    }
  }
  if (static_cast<int>(filtered->size()) < options_.min_cluster_size) {
    return false;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr working = filtered;
  pcl::PointCloud<pcl::PointXYZI>::Ptr roi_cloud(new pcl::PointCloud<pcl::PointXYZI>);

  if (roi_center_prior_l) {
    pcl::CropBox<pcl::PointXYZI> crop;
    crop.setInputCloud(filtered);
    const double h = options_.roi_half_extent_m;
    Eigen::Vector4f min_pt(static_cast<float>(roi_center_prior_l->x() - h),
                           static_cast<float>(roi_center_prior_l->y() - h),
                           static_cast<float>(roi_center_prior_l->z() - h), 1.0f);
    Eigen::Vector4f max_pt(static_cast<float>(roi_center_prior_l->x() + h),
                           static_cast<float>(roi_center_prior_l->y() + h),
                           static_cast<float>(roi_center_prior_l->z() + h), 1.0f);
    crop.setMin(min_pt);
    crop.setMax(max_pt);
    crop.filter(*roi_cloud);
    if (static_cast<int>(roi_cloud->size()) >= options_.min_cluster_size) {
      working = roi_cloud;
    }
  }

  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud(working);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  ec.setClusterTolerance(options_.cluster_tolerance_m);
  ec.setMinClusterSize(options_.min_cluster_size);
  ec.setMaxClusterSize(static_cast<int>(working->size()));
  ec.setSearchMethod(tree);
  ec.setInputCloud(working);
  ec.extract(cluster_indices);

  if (cluster_indices.empty()) {
    return false;
  }

  std::sort(cluster_indices.begin(), cluster_indices.end(),
            [](const pcl::PointIndices& a, const pcl::PointIndices& b) {
              return a.indices.size() > b.indices.size();
            });

  const double r_min =
      options_.radius_prior_m * (1.0 - options_.max_radius_deviation_ratio);
  const double r_max =
      options_.radius_prior_m * (1.0 + options_.max_radius_deviation_ratio);

  for (const auto& indices : cluster_indices) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::copyPointCloud(*working, indices, *cluster);

    pcl::SACSegmentation<pcl::PointXYZI> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_SPHERE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(options_.ransac_inlier_threshold_m);
    seg.setRadiusLimits(r_min, r_max);
    seg.setMaxIterations(1000);
    seg.setInputCloud(cluster);

    pcl::ModelCoefficients coefficients;
    pcl::PointIndices inliers;
    seg.segment(inliers, coefficients);

    if (inliers.indices.empty() || coefficients.values.size() < 4) {
      continue;
    }

    const double inlier_ratio =
        static_cast<double>(inliers.indices.size()) /
        static_cast<double>(cluster->size());
    if (inlier_ratio < options_.min_inlier_ratio) {
      continue;
    }

    const double radius = coefficients.values[3];
    if (std::abs(radius - options_.radius_prior_m) >
        options_.max_radius_deviation_ratio * options_.radius_prior_m) {
      continue;
    }

    if (fitted_center_l) {
      *fitted_center_l =
          Eigen::Vector3d(coefficients.values[0], coefficients.values[1],
                          coefficients.values[2]);
    }

    observation->points_L_.reserve(inliers.indices.size());
    for (const int idx : inliers.indices) {
      const auto& p = cluster->points[idx];
      observation->points_L_.emplace_back(p.x, p.y, p.z);
    }
    return !observation->points_L_.empty();
  }

  return false;
}

}  // namespace clic_calib
