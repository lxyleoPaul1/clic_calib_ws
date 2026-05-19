#pragma once

#include <string>

namespace clic_calib {

/** @brief Load roadside LiDAR + camera bags (Phase 1). */
class RosbagLoader {
 public:
  explicit RosbagLoader(const std::string& bag_path);
};

}  // namespace clic_calib
