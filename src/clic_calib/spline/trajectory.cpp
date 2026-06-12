#include <clic_calib/spline/trajectory.h>
#include <clic_calib/utils/lever_arm.h>

namespace clic_calib {

BodyTrajectory::BodyTrajectory(double knot_interval_s, double start_time_s)
    : Se3Spline<SplineOrder, double>(knot_interval_s * kSToNs,
                                     start_time_s * kSToNs) {
  extendKnotsTo(start_time_s * kSToNs, SO3d(Eigen::Quaterniond::Identity()),
                Eigen::Vector3d::Zero());
}

SE3d BodyTrajectory::pose_wb(double t_s) const {
  return poseNs(static_cast<int64_t>(t_s * kSToNs));
}

Eigen::Vector3d BodyTrajectory::position_wb(double t_s) const {
  return positionWorld(static_cast<int64_t>(t_s * kSToNs));
}

SO3d BodyTrajectory::rotation_wb(double t_s) const {
  return pose_wb(t_s).so3();
}

Eigen::Vector3d BodyTrajectory::antenna_position_w(
    double t_s, const Eigen::Vector3d& lever_b_to_a) const {
  return antenna_position_world(pose_wb(t_s), lever_b_to_a);
}

Eigen::Vector3d BodyTrajectory::sphere_center_w(
    double t_s, const Eigen::Vector3d& lever_b_to_g) const {
  return sphere_center_world(pose_wb(t_s), lever_b_to_g);
}

Eigen::Vector3d BodyTrajectory::body_centroid_w(
    double t_s, const Eigen::Vector3d& lever_b_to_body) const {
  return body_centroid_world(pose_wb(t_s), lever_b_to_body);
}

Eigen::Vector3d BodyTrajectory::marker_center_w(
    double t_s, const Eigen::Vector3d& lever_b_to_g,
    const Eigen::Vector3d& lever_g_to_mj) const {
  return marker_position_world(pose_wb(t_s), lever_b_to_g, lever_g_to_mj);
}

}  // namespace clic_calib
