#pragma once

#include "experiments/body_sampling_common.hpp"

#include <clic_calib/target/body_centroid_analysis.h>

#include <iomanip>
#include <iostream>
#include <string>

namespace clic_calib {
namespace experiments {

inline const char* RangeBiasMechanismLabel(RangeBiasMechanism m) {
  switch (m) {
    case RangeBiasMechanism::kLinearPB:
      return "p_B(r) linear";
    case RangeBiasMechanism::kRangeWeighting:
      return "range down-weight";
    case RangeBiasMechanism::kRangeWindow:
      return "multi-face range window";
  }
  return "?";
}

inline std::vector<int> CollectVisibleFacesAtGt(
    const BodyTrajectory& traj, const SE3d& T_LW, double t_d_L_s,
    const Eigen::Vector3d& L_B_nominal,
    const std::vector<BodyClusterObservation>& observations) {
  std::vector<int> faces;
  faces.reserve(observations.size());
  for (const auto& obs : observations) {
    const double t_world = obs.t_sensor_ - t_d_L_s;
    const SE3d T_WB = traj.pose_wb(t_world);
    const BodySampleResult sample = SampleBodyPointsDeterministic(
        T_WB.so3(), T_WB.translation(), T_LW, L_B_nominal, nullptr, false);
    faces.push_back(sample.visible_faces);
  }
  return faces;
}

inline void PrintRangeBiasScatterReport(const char* sensor_tag,
                                        const RangeBiasScatterReport& r) {
  std::cout << std::fixed << std::setprecision(3);
  std::cout << "  [" << sensor_tag << "] scatter n=" << r.samples.size()
            << "  r↔|bias|=" << r.corr_range_bias_norm
            << "  slope=" << r.bias_norm_slope_mm_per_m << " mm/m"
            << "  r↔pts=" << r.corr_range_point_count
            << "  r↔faces=" << r.corr_range_visible_faces << "\n";
  std::cout << "    faces: 1=" << r.count_faces_1 << " (|b|="
            << r.mean_bias_norm_faces_1_mm << " mm)  2=" << r.count_faces_2
            << " (" << r.mean_bias_norm_faces_2_mm << " mm)  3+="
            << r.count_faces_3plus << " (" << r.mean_bias_norm_faces_3plus_mm
            << " mm)\n";
  std::cout << "    → mechanism: " << RangeBiasMechanismLabel(r.mechanism)
            << "\n";
}

inline RangeBiasMechanism MergeRangeBiasMechanisms(
    RangeBiasMechanism a, RangeBiasMechanism b) {
  if (a == b) {
    return a;
  }
  if (a == RangeBiasMechanism::kRangeWindow ||
      b == RangeBiasMechanism::kRangeWindow) {
    return RangeBiasMechanism::kRangeWindow;
  }
  if (a == RangeBiasMechanism::kLinearPB ||
      b == RangeBiasMechanism::kLinearPB) {
    return RangeBiasMechanism::kLinearPB;
  }
  return RangeBiasMechanism::kRangeWeighting;
}

}  // namespace experiments
}  // namespace clic_calib
