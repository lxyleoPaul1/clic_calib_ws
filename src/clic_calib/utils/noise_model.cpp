#include <clic_calib/utils/noise_model.h>

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <stdexcept>

namespace clic_calib {

NoiseModel NoiseModel::FromYaml(const std::string& path) {
  const YAML::Node node = YAML::LoadFile(path);
  NoiseModel model;
  if (node["rtk"]) {
    const YAML::Node rtk = node["rtk"];
    if (rtk["sigma_horizontal_m"]) {
      model.rtk_sigma_horizontal_m = rtk["sigma_horizontal_m"].as<double>();
    }
    if (rtk["sigma_vertical_m"]) {
      model.rtk_sigma_vertical_m = rtk["sigma_vertical_m"].as<double>();
    }
  }
  if (node["lidar"] && node["lidar"]["ranging_sigma_m"]) {
    model.lidar_ranging_sigma_m = node["lidar"]["ranging_sigma_m"].as<double>();
  }
  if (node["camera"] && node["camera"]["pixel_sigma"]) {
    model.camera_pixel_sigma = node["camera"]["pixel_sigma"].as<double>();
  }
  return model;
}

NoiseModel NoiseModel::FromConfigDir(const std::string& config_dir) {
  std::string prefix = config_dir;
  if (!prefix.empty() && prefix.back() == '/') {
    prefix.pop_back();
  }
  return FromYaml(prefix + "/noise_model.yaml");
}

Eigen::Matrix3d NoiseModel::RtkPositionCovariance() const {
  Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
  const double sh = rtk_sigma_horizontal_m;
  const double sv = rtk_sigma_vertical_m;
  cov.diagonal() << sh * sh, sh * sh, sv * sv;
  return cov;
}

Eigen::Vector3d NoiseModel::SampleRtkNoise(std::mt19937& rng) const {
  std::normal_distribution<double> noise_xy(0.0, rtk_sigma_horizontal_m);
  std::normal_distribution<double> noise_z(0.0, rtk_sigma_vertical_m);
  return Eigen::Vector3d(noise_xy(rng), noise_xy(rng), noise_z(rng));
}

double NoiseModel::SampleLidarRangeNoise(std::mt19937& rng) const {
  std::normal_distribution<double> noise_r(0.0, lidar_ranging_sigma_m);
  return noise_r(rng);
}

Eigen::Vector2d NoiseModel::SamplePixelNoise(std::mt19937& rng) const {
  std::normal_distribution<double> noise_pix(0.0, camera_pixel_sigma);
  return Eigen::Vector2d(noise_pix(rng), noise_pix(rng));
}

void NoiseModel::Log(std::ostream& os) const {
  os << "[NoiseModel] rtk σ_h=" << rtk_sigma_horizontal_m
     << " m, σ_v=" << rtk_sigma_vertical_m << " m; lidar σ_r="
     << lidar_ranging_sigma_m << " m; camera σ_pix=" << camera_pixel_sigma
     << " px (from config/noise_model.yaml)\n";
}

}  // namespace clic_calib
