#include <clic_calib/estimator/attitude_stream_config.h>

#include <yaml-cpp/yaml.h>

#include <iostream>

namespace clic_calib {

AttitudeStreamConfig AttitudeStreamConfig::FromSplineYaml(
    const std::string& path) {
  AttitudeStreamConfig cfg;
  const YAML::Node node = YAML::LoadFile(path);
  if (node["attitude"]) {
    const YAML::Node att = node["attitude"];
    if (att["stride"]) {
      cfg.stride = att["stride"].as<int>();
    }
    if (att["transport_delay_s"]) {
      cfg.transport_delay_s = att["transport_delay_s"].as<double>();
    }
  }
  return cfg;
}

AttitudeStreamConfig AttitudeStreamConfig::FromConfigDir(
    const std::string& config_dir) {
  std::string prefix = config_dir;
  if (!prefix.empty() && prefix.back() == '/') {
    prefix.pop_back();
  }
  return FromSplineYaml(prefix + "/spline.yaml");
}

void AttitudeStreamConfig::Log(std::ostream& os) const {
  os << "[AttitudeStream] stride=" << stride
     << " (50 Hz PSDK → ~" << (50 / std::max(1, stride))
     << " Hz factors); transport_delay_s=" << transport_delay_s
     << " (absorbed with t_d in Stage 2)\n";
}

}  // namespace clic_calib
