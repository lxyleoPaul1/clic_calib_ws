#pragma once

#include <clic_calib/sensor_data/apriltag_observation.h>
#include <clic_calib/sensor_data/lidar_target_observation.h>

#include <map>
#include <string>
#include <vector>

namespace clic_calib {

/** @brief Compact binary cache of preprocessed target observations (magic CLICOB01). */
class ObservationArchive {
 public:
  using LidarBySensor = std::map<int, std::vector<LiDARTargetObservation>>;
  using AprilTagBySensor = std::map<int, std::vector<AprilTagObservation>>;

  static bool Write(const std::string& path, const LidarBySensor& lidar,
                    const AprilTagBySensor& apriltag);

  static bool Read(const std::string& path, LidarBySensor* lidar,
                     AprilTagBySensor* apriltag);
};

}  // namespace clic_calib
