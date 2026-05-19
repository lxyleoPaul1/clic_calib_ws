#pragma once

#include <clic_calib/sensor_data/rtk_measurement.h>

#include <string>
#include <vector>

namespace clic_calib {

class RtkReader {
 public:
  static std::vector<RtkMeasurement> LoadDat(const std::string& path);
};

}  // namespace clic_calib
