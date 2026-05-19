#pragma once

#include <clic_calib/sensor_data/apriltag_observation.h>

#include <string>

namespace clic_calib {

class AprilTagDetectorWrapper {
 public:
  explicit AprilTagDetectorWrapper(const std::string& family);
  bool Detect(const std::string& image_path, AprilTagImageObservation* out);
};

}  // namespace clic_calib
