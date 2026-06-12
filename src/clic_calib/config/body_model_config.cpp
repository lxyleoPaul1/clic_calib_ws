#include <clic_calib/config/body_model_config.h>

#include <yaml-cpp/yaml.h>

namespace clic_calib {
namespace {

Eigen::Vector3d ReadVec3(const YAML::Node& n) {
  return Eigen::Vector3d(n[0].as<double>(), n[1].as<double>(), n[2].as<double>());
}

}  // namespace

BodyModelConfig BodyModelConfig::FromYaml(const std::string& path) {
  const YAML::Node node = YAML::LoadFile(path);
  BodyModelConfig cfg;
  if (!node["body_model"]) {
    return cfg;
  }
  const YAML::Node bm = node["body_model"];
  if (bm["effective_radius"]) {
    cfg.effective_radius_m = bm["effective_radius"].as<double>();
  } else if (bm["effective_radius_m"]) {
    cfg.effective_radius_m = bm["effective_radius_m"].as<double>();
  }
  if (bm["cad_path"]) {
    cfg.cad_path = bm["cad_path"].as<std::string>();
  }
  if (bm["half_extent"] && bm["half_extent"].size() == 3) {
    cfg.half_extent_B = ReadVec3(bm["half_extent"]);
  } else if (bm["half_extent_m"] && bm["half_extent_m"].size() == 3) {
    cfg.half_extent_B = ReadVec3(bm["half_extent_m"]);
  }
  if (bm["centroid_lever_arm"] && bm["centroid_lever_arm"].size() == 3) {
    cfg.centroid_lever_arm_B = ReadVec3(bm["centroid_lever_arm"]);
  }
  if (bm["enable_point_to_model"]) {
    cfg.enable_point_to_model = bm["enable_point_to_model"].as<bool>();
  }
  return cfg;
}

BodyModelConfig BodyModelConfig::FromConfigDir(const std::string& config_dir) {
  std::string prefix = config_dir;
  if (!prefix.empty() && prefix.back() == '/') {
    prefix.pop_back();
  }
  return FromYaml(prefix + "/target_geometry.yaml");
}

}  // namespace clic_calib
