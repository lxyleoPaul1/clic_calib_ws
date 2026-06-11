#include <clic_calib/utils/lever_arm.h>

#include <yaml-cpp/yaml.h>

#include <stdexcept>

namespace clic_calib {

namespace {

Eigen::Vector3d ReadVec3(const YAML::Node& n) {
  if (!n || !n.IsSequence() || n.size() != 3) {
    throw std::runtime_error("Expected YAML sequence of length 3 for Vector3");
  }
  return Eigen::Vector3d(n[0].as<double>(), n[1].as<double>(), n[2].as<double>());
}

void LoadLeverMap(const YAML::Node& node, std::map<int, Eigen::Vector3d>* out) {
  if (!node) {
    return;
  }
  if (node.IsMap()) {
    for (const auto& item : node) {
      const int id = item.first.as<int>();
      (*out)[id] = ReadVec3(item.second);
    }
    return;
  }
  if (node.IsSequence()) {
    for (const auto& item : node) {
      const int id = item["id"].as<int>();
      (*out)[id] = ReadVec3(item["offset"]);
    }
    return;
  }
  throw std::runtime_error("L_G_to_M / lever_G_to_M must be a map or sequence");
}

}  // namespace

LeverArmConfig LeverArmConfig::from_yaml(const std::string& path) {
  const YAML::Node node = YAML::LoadFile(path);
  LeverArmConfig out;

  if (node["L_B_to_A"]) {
    out.L_B_to_A = ReadVec3(node["L_B_to_A"]);
  } else if (node["lever_B_to_A"]) {
    out.L_B_to_A = ReadVec3(node["lever_B_to_A"]);
  }

  if (node["L_B_to_G"]) {
    out.L_B_to_G = ReadVec3(node["L_B_to_G"]);
  } else if (node["lever_B_to_G"]) {
    out.L_B_to_G = ReadVec3(node["lever_B_to_G"]);
  }

  if (node["body_centroid"]) {
    out.L_B_to_body_centroid = ReadVec3(node["body_centroid"]);
  } else if (node["L_B_to_body_centroid"]) {
    out.L_B_to_body_centroid = ReadVec3(node["L_B_to_body_centroid"]);
  }

  if (node["L_G_to_M"]) {
    LoadLeverMap(node["L_G_to_M"], &out.L_G_to_M);
  } else if (node["lever_G_to_M"]) {
    LoadLeverMap(node["lever_G_to_M"], &out.L_G_to_M);
  }

  if (node["body_convention"]) {
    out.body_convention = node["body_convention"].as<std::string>();
  }

  return out;
}

LeverArms LeverArms::LoadFromYaml(const std::string& path) {
  const LeverArmConfig cfg = LeverArmConfig::from_yaml(path);
  LeverArms out;
  out.lever_b_to_a = cfg.L_B_to_A;
  out.lever_b_to_g = cfg.L_B_to_G;
  out.lever_g_to_mj = cfg.L_G_to_M;
  return out;
}

}  // namespace clic_calib
