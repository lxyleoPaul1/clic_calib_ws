#pragma once

#include <Eigen/Core>
#include <vector>

namespace clic_calib {

class SphereExtractor {
 public:
  static bool Extract(const std::vector<Eigen::Vector3d>& points_l,
                      double radius_m, Eigen::Vector3d* center_l,
                      double* rmse);
};

}  // namespace clic_calib
