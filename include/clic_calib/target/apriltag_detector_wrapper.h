#pragma once

#include <clic_calib/sensor_data/apriltag_observation.h>

#include <string>
#include <vector>

namespace clic_calib {

/** @brief AprilTag detector wrapper (Phase 2 implementation). */
class AprilTagDetectorWrapper {
 public:
  explicit AprilTagDetectorWrapper(const std::string& family);
  bool Detect(const std::string& image_path,
              std::vector<AprilTagObservation>* out);
};

}  // namespace clic_calib
