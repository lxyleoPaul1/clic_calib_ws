#include <clic_calib/estimator/observed_mean_gate.h>

#include <cmath>

namespace clic_calib {

bool ShouldApplyObservedMeanPB(const TrajectoryAttitudeSpread& spread,
                               const ObservedMeanAspectGate& gate,
                               double u_B_azimuth_std_deg) {
  if (spread.yaw_circular_variance >= gate.min_yaw_circular_variance) {
    return true;
  }
  if (u_B_azimuth_std_deg > 0.1 &&
      u_B_azimuth_std_deg <= gate.max_u_B_azimuth_std_deg &&
      spread.pitch_std_deg >= gate.min_pitch_std_deg_for_locked_look) {
    return true;
  }
  return false;
}

double ComputeUBAzimuthStdDeg(
    const BodyTrajectory& traj, double t_d_L_s,
    const Eigen::Vector3d& lidar_post_W,
    const std::vector<BodyClusterObservation>& observations) {
  if (observations.empty()) {
    return -1.0;
  }
  double mean = 0.0;
  std::vector<double> az_deg;
  az_deg.reserve(observations.size());
  for (const auto& obs : observations) {
    const double t_world = obs.t_sensor_ - t_d_L_s;
    const SE3d T_WB = traj.pose_wb(t_world);
    const Eigen::Vector3d u_B =
        LidarDirectionInBody(T_WB, T_WB.translation(), lidar_post_W);
    const double a = std::atan2(u_B.y(), u_B.x()) * 180.0 / M_PI;
    az_deg.push_back(a);
    mean += a;
  }
  mean /= static_cast<double>(az_deg.size());
  double sq = 0.0;
  for (double a : az_deg) {
    const double d = a - mean;
    sq += d * d;
  }
  return std::sqrt(sq / static_cast<double>(az_deg.size()));
}

}  // namespace clic_calib
