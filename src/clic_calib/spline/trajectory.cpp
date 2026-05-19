#include <clic_calib/spline/trajectory.h>

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
  const SE3d T_wb = pose_wb(t_s);
  return T_wb * lever_b_to_a;
}

Eigen::Vector3d BodyTrajectory::sphere_center_w(
    double t_s, const Eigen::Vector3d& lever_b_to_g) const {
  const SE3d T_wb = pose_wb(t_s);
  return T_wb * lever_b_to_g;
}

Eigen::Vector3d BodyTrajectory::marker_center_w(
    double t_s, const Eigen::Vector3d& lever_b_to_g,
    const Eigen::Vector3d& lever_g_to_mj) const {
  const SE3d T_wb = pose_wb(t_s);
  return sphere_center_w(t_s, lever_b_to_g) + T_wb.so3() * lever_g_to_mj;
}

}  // namespace clic_calib
