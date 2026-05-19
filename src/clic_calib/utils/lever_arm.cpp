#include <clic_calib/utils/lever_arm.h>

#include <yaml-cpp/yaml.h>

#include <stdexcept>

namespace clic_calib {

LeverArms LeverArms::LoadFromYaml(const std::string& path) {
  YAML::Node node = YAML::LoadFile(path);
  LeverArms out;

  auto read_vec3 = [](const YAML::Node& n) {
    if (!n || !n.IsSequence() || n.size() != 3) {
      throw std::runtime_error("Expected YAML sequence of length 3 for Vector3");
    }
    return Eigen::Vector3d(n[0].as<double>(), n[1].as<double>(),
                           n[2].as<double>());
  };

  if (node["lever_B_to_A"]) {
    out.lever_b_to_a = read_vec3(node["lever_B_to_A"]);
  }
  if (node["lever_B_to_G"]) {
    out.lever_b_to_g = read_vec3(node["lever_B_to_G"]);
  }
  if (node["lever_G_to_M"]) {
    for (const auto& item : node["lever_G_to_M"]) {
      int id = item["id"].as<int>();
      out.lever_g_to_mj[id] = read_vec3(item["offset"]);
    }
  }
  return out;
}

}  // namespace clic_calib
