/*
 * clic_calib — offline rosbag preprocessing (Phase 4).
 *
 * Usage:
 *   preprocess_rosbag --bag <path.bag> --config <dir> --output <observations.clicob>
 */

#include <clic_calib/io/observation_archive.h>
#include <clic_calib/target/apriltag_detector_wrapper.h>
#include <clic_calib/target/sphere_extractor.h>

#include <Eigen/Core>
#include <cv_bridge/cv_bridge.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <yaml-cpp/yaml.h>

#include <iostream>
#include <map>
#include <string>

namespace {

std::string GetArg(int argc, char** argv, const std::string& key) {
  for (int i = 1; i + 1 < argc; ++i) {
    if (key == argv[i]) {
      return argv[i + 1];
    }
  }
  return "";
}

clic_calib::SphereExtractor::Options LoadSphereOptions(const std::string& config_dir) {
  clic_calib::SphereExtractor::Options opts;
  const std::string path = config_dir + "/target_detection.yaml";
  try {
    const YAML::Node node = YAML::LoadFile(path);
    if (node["intensity_min"]) {
      opts.intensity_min = node["intensity_min"].as<double>();
    }
    if (node["cluster_tolerance_m"]) {
      opts.cluster_tolerance_m = node["cluster_tolerance_m"].as<double>();
    }
    if (node["min_cluster_size"]) {
      opts.min_cluster_size = node["min_cluster_size"].as<int>();
    }
    if (node["radius_prior_m"]) {
      opts.radius_prior_m = node["radius_prior_m"].as<double>();
    }
    if (node["ransac_inlier_threshold_m"]) {
      opts.ransac_inlier_threshold_m = node["ransac_inlier_threshold_m"].as<double>();
    }
    if (node["min_inlier_ratio"]) {
      opts.min_inlier_ratio = node["min_inlier_ratio"].as<double>();
    }
    if (node["max_radius_deviation_ratio"]) {
      opts.max_radius_deviation_ratio = node["max_radius_deviation_ratio"].as<double>();
    }
    if (node["roi_half_extent_m"]) {
      opts.roi_half_extent_m = node["roi_half_extent_m"].as<double>();
    }
  } catch (...) {
    // defaults
  }
  return opts;
}

clic_calib::AprilTagDetectorWrapper::Options LoadAprilTagOptions(
    const std::string& config_dir) {
  clic_calib::AprilTagDetectorWrapper::Options opts;
  const std::string path = config_dir + "/target_detection.yaml";
  try {
    const YAML::Node node = YAML::LoadFile(path);
    if (node["apriltag_family"]) {
      opts.family = node["apriltag_family"].as<std::string>();
    }
    if (node["quad_decimate"]) {
      opts.quad_decimate = node["quad_decimate"].as<double>();
    }
    if (node["nthreads"]) {
      opts.nthreads = node["nthreads"].as<int>();
    }
  } catch (...) {
  }
  return opts;
}

struct SensorTopics {
  std::map<int, std::string> lidar_topics;
  std::map<int, std::string> camera_topics;
};

SensorTopics LoadTopics(const std::string& config_dir) {
  SensorTopics out;
  const YAML::Node node = YAML::LoadFile(config_dir + "/sensor_rig.yaml");
  if (node["lidars"]) {
    for (const auto& L : node["lidars"]) {
      out.lidar_topics[L["id"].as<int>()] = L["topic"].as<std::string>();
    }
  }
  if (node["cameras"]) {
    for (const auto& C : node["cameras"]) {
      out.camera_topics[C["id"].as<int>()] = C["topic"].as<std::string>();
    }
  }
  return out;
}

int SensorIdFromTopic(const std::map<int, std::string>& topic_map,
                      const std::string& topic) {
  for (const auto& kv : topic_map) {
    if (kv.second == topic) {
      return kv.first;
    }
  }
  return -1;
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "preprocess_rosbag");
  ros::Time::init();

  const std::string bag_path = GetArg(argc, argv, "--bag");
  const std::string config_dir = GetArg(argc, argv, "--config");
  const std::string output_path = GetArg(argc, argv, "--output");

  if (bag_path.empty() || config_dir.empty() || output_path.empty()) {
    std::cerr << "Usage: preprocess_rosbag --bag <path.bag> --config <dir> "
                 "--output <observations.clicob>\n";
    return 1;
  }

  try {
    const SensorTopics topics = LoadTopics(config_dir);
    clic_calib::SphereExtractor sphere_extractor(LoadSphereOptions(config_dir));
    clic_calib::AprilTagDetectorWrapper tag_detector(LoadAprilTagOptions(config_dir));

    clic_calib::ObservationArchive::LidarBySensor lidar_obs;
    clic_calib::ObservationArchive::AprilTagBySensor tag_obs;

    std::map<int, Eigen::Vector3d> last_sphere_center_l;

    rosbag::Bag bag;
    bag.open(bag_path, rosbag::bagmode::Read);
    rosbag::View view(bag);

    size_t lidar_msgs = 0;
    size_t image_msgs = 0;
    size_t lidar_hits = 0;
    size_t tag_hits = 0;

    for (const rosbag::MessageInstance& msg : view) {
      const std::string topic = msg.getTopic();

      const int lidar_id = SensorIdFromTopic(topics.lidar_topics, topic);
      if (lidar_id >= 0) {
        ++lidar_msgs;
        const auto cloud_msg = msg.instantiate<sensor_msgs::PointCloud2>();
        if (!cloud_msg) {
          continue;
        }
        pcl::PointCloud<pcl::PointXYZI> cloud;
        pcl::fromROSMsg(*cloud_msg, cloud);
        const double t_s = cloud_msg->header.stamp.toSec();

        const Eigen::Vector3d* prior = nullptr;
        auto it = last_sphere_center_l.find(lidar_id);
        if (it != last_sphere_center_l.end()) {
          prior = &it->second;
        }

        clic_calib::LiDARTargetObservation obs;
        Eigen::Vector3d center_l;
        if (sphere_extractor.Extract(cloud, t_s, lidar_id, prior, &obs, &center_l)) {
          lidar_obs[lidar_id].push_back(std::move(obs));
          last_sphere_center_l[lidar_id] = center_l;
          ++lidar_hits;
        }
        continue;
      }

      const int cam_id = SensorIdFromTopic(topics.camera_topics, topic);
      if (cam_id >= 0) {
        ++image_msgs;
        const auto image_msg = msg.instantiate<sensor_msgs::Image>();
        if (!image_msg) {
          continue;
        }
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
          cv_ptr = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8);
        } catch (const cv_bridge::Exception& e) {
          std::cerr << "cv_bridge: " << e.what() << "\n";
          continue;
        }
        const double t_s = image_msg->header.stamp.toSec();
        std::vector<clic_calib::AprilTagObservation> detections;
        if (tag_detector.Detect(cv_ptr->image, t_s, cam_id, &detections)) {
          auto& dst = tag_obs[cam_id];
          dst.insert(dst.end(), detections.begin(), detections.end());
          ++tag_hits;
        }
      }
    }
    bag.close();

    if (!clic_calib::ObservationArchive::Write(output_path, lidar_obs, tag_obs)) {
      std::cerr << "Failed to write " << output_path << "\n";
      return 2;
    }

    std::cout << "Processed bag: " << bag_path << "\n"
              << "  LiDAR messages: " << lidar_msgs << ", sphere detections: " << lidar_hits
              << "\n"
              << "  Image messages: " << image_msgs << ", tag frames: " << tag_hits << "\n"
              << "  Wrote: " << output_path << "\n";
  } catch (const std::exception& e) {
    std::cerr << "preprocess_rosbag failed: " << e.what() << "\n";
    return 1;
  }

  return 0;
}
