#include <clic_calib/utils/noise_model.h>

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <limits>
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
  if (node["attitude"]) {
    const YAML::Node att = node["attitude"];
    if (att["sigma_roll_deg"]) {
      model.attitude_sigma_roll_deg = att["sigma_roll_deg"].as<double>();
    }
    if (att["sigma_pitch_deg"]) {
      model.attitude_sigma_pitch_deg = att["sigma_pitch_deg"].as<double>();
    }
    if (att["sigma_yaw_deg"]) {
      model.attitude_sigma_yaw_deg = att["sigma_yaw_deg"].as<double>();
    }
  }
  if (node["body_centroid"]) {
    const YAML::Node bc = node["body_centroid"];
    if (bc["sigma_base"]) {
      model.body_centroid_sigma_base_m = bc["sigma_base"].as<double>();
    } else if (bc["sigma_base_m"]) {
      model.body_centroid_sigma_base_m = bc["sigma_base_m"].as<double>();
    }
    if (bc["range_coeff"]) {
      model.body_centroid_range_coeff = bc["range_coeff"].as<double>();
    }
    if (bc["min_points_for_valid"]) {
      model.body_centroid_min_points_for_valid =
          bc["min_points_for_valid"].as<int>();
    }
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

Eigen::Matrix3d NoiseModel::AttitudeTangentCovarianceRad2() const {
  const double sr = attitude_sigma_roll_deg * M_PI / 180.0;
  const double sp = attitude_sigma_pitch_deg * M_PI / 180.0;
  const double sy = attitude_sigma_yaw_deg * M_PI / 180.0;
  Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
  cov(0, 0) = sr * sr;
  cov(1, 1) = sp * sp;
  cov(2, 2) = sy * sy;
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

double NoiseModel::BodyCentroidSigmaM(double mean_range_m,
                                      int point_count) const {
  if (point_count < body_centroid_min_points_for_valid) {
    return std::numeric_limits<double>::infinity();
  }
  const double r = std::max(0.0, mean_range_m);
  return body_centroid_sigma_base_m + body_centroid_range_coeff * r;
}

Eigen::Matrix3d NoiseModel::BodyCentroidCovariance(double mean_range_m,
                                                   int point_count) const {
  const double sigma = BodyCentroidSigmaM(mean_range_m, point_count);
  Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
  if (!std::isfinite(sigma) || sigma <= 0.0) {
    return cov;
  }
  const double v = sigma * sigma;
  cov.diagonal() << v, v, v;
  return cov;
}

Eigen::Matrix3d NoiseModel::BodyCentroidSqrtInformation(double mean_range_m,
                                                        int point_count) const {
  const double sigma = BodyCentroidSigmaM(mean_range_m, point_count);
  Eigen::Matrix3d info = Eigen::Matrix3d::Zero();
  if (!std::isfinite(sigma) || sigma <= 0.0) {
    return info;
  }
  const double inv_sigma = 1.0 / sigma;
  info.diagonal() << inv_sigma, inv_sigma, inv_sigma;
  return info;
}

Eigen::Matrix3d NoiseModel::BodyCentroidSqrtInformationFromObservation(
    const BodyClusterObservation& obs) const {
  if (obs.has_centroid_cov_) {
    Eigen::LLT<Eigen::Matrix3d> llt(obs.centroid_cov_);
    if (llt.info() == Eigen::Success) {
      const Eigen::Matrix3d L = llt.matrixL();
      if (L.diagonal().minCoeff() > 1e-12) {
        return L.inverse();
      }
    }
  }
  return BodyCentroidSqrtInformation(obs.mean_range_m_,
                                     static_cast<int>(obs.point_count_));
}

void NoiseModel::Log(std::ostream& os) const {
  os << "[NoiseModel] rtk σ_h=" << rtk_sigma_horizontal_m
     << " m, σ_v=" << rtk_sigma_vertical_m << " m; lidar σ_r="
     << lidar_ranging_sigma_m << " m; camera σ_pix=" << camera_pixel_sigma
     << " px; Σ_att σ_roll/pitch/yaw="
     << attitude_sigma_roll_deg << "/" << attitude_sigma_pitch_deg << "/"
     << attitude_sigma_yaw_deg << " deg; body_centroid σ_base="
     << body_centroid_sigma_base_m << " m + " << body_centroid_range_coeff
     << "*range (min_pts=" << body_centroid_min_points_for_valid
     << ") (config/noise_model.yaml)\n";
}

}  // namespace clic_calib
