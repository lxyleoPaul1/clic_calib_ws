#pragma once

#include <clic_calib/sensor_data/body_cluster_observation.h>
#include <clic_calib/spline/trajectory.h>
#include <clic_calib/target/body_centroid_analysis.h>

#include <vector>

namespace clic_calib {

/** Data-calibrated gate for observed-mean p_B (yaw_cv ~0.65 @ diverse aspect). */
struct ObservedMeanAspectGate {
  double min_yaw_circular_variance = 0.65;
  double max_u_B_azimuth_std_deg = 5.0;
  double min_pitch_std_deg_for_locked_look = 14.0;
};

bool ShouldApplyObservedMeanPB(const TrajectoryAttitudeSpread& spread,
                               const ObservedMeanAspectGate& gate = {},
                               double u_B_azimuth_std_deg = -1.0);

double ComputeUBAzimuthStdDeg(
    const BodyTrajectory& traj, double t_d_L_s,
    const Eigen::Vector3d& lidar_post_W,
    const std::vector<BodyClusterObservation>& observations);

}  // namespace clic_calib
