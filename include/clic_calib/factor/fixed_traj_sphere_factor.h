#pragma once

#include <clic_calib/spline/trajectory.h>
#include <clic_calib/utils/sophus_utils.hpp>

#include <ceres/ceres.h>
#include <Eigen/Core>

namespace clic_calib {

inline Eigen::Vector3d WorldPointVelocity(const BodyTrajectory& traj,
                                          double t_s,
                                          const Eigen::Vector3d& lever_b) {
  const double eps = 1e-3;
  const Eigen::Vector3d p0 =
      traj.position_wb(t_s - eps) + traj.rotation_wb(t_s - eps) * lever_b;
  const Eigen::Vector3d p1 =
      traj.position_wb(t_s + eps) + traj.rotation_wb(t_s + eps) * lever_b;
  return (p1 - p0) / (2.0 * eps);
}

/** Stage-2 sphere residual; trajectory fixed, t_d shifts evaluation time. */
class FixedTrajSphereFactor : public ceres::SizedCostFunction<1, 1, 4, 3> {
 public:
  FixedTrajSphereFactor(const BodyTrajectory& traj, double t_bar,
                        const Eigen::Vector3d& q_L,
                        const Eigen::Vector3d& L_B_to_G, double R_ball,
                        double inv_sigma_r)
      : traj_(traj),
        t_bar_(t_bar),
        q_L_(q_L),
        L_B_to_G_(L_B_to_G),
        R_ball_(R_ball),
        inv_sigma_r_(inv_sigma_r) {}

  bool Evaluate(double const* const* parameters, double* residuals,
                double** jacobians) const override {
    const double t_d = parameters[0][0];
    const double t_eval = t_bar_ - t_d;
    const Eigen::Vector3d p_G_W = traj_.sphere_center_w(t_eval, L_B_to_G_);
    const Eigen::Vector3d dp_GW_dt =
        -WorldPointVelocity(traj_, t_eval, L_B_to_G_);
    const Eigen::Map<const Eigen::Quaterniond> q_LW(parameters[1]);
    const Eigen::Map<const Eigen::Vector3d> t_LW(parameters[2]);
    const SE3d T_LW(SO3d(q_LW), t_LW);
    const Eigen::Vector3d p_G_L = T_LW * p_G_W;
    const Eigen::Vector3d diff = q_L_ - p_G_L;
    const double dist = diff.norm();
    const double inv_dist = dist > 1e-12 ? 1.0 / dist : 0.0;
    const Eigen::Vector3d n = inv_dist * diff;
    residuals[0] = inv_sigma_r_ * (dist - R_ball_);
    if (!jacobians) {
      return true;
    }
    const Eigen::Matrix3d R_LW = T_LW.so3().matrix();
    if (jacobians[0]) {
      Eigen::Map<Eigen::Matrix<double, 1, 1>> J_td(jacobians[0]);
      J_td(0, 0) = inv_sigma_r_ * n.dot(R_LW * dp_GW_dt);
    }
    if (jacobians[1]) {
      Eigen::Map<Eigen::Matrix<double, 1, 4, Eigen::RowMajor>> J_q(jacobians[1]);
      J_q.setZero();
      J_q.block<1, 3>(0, 0) =
          inv_sigma_r_ * n.transpose() * (-R_LW * SO3d::hat(p_G_W));
    }
    if (jacobians[2]) {
      Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> J_t(jacobians[2]);
      J_t = inv_sigma_r_ * n.transpose();
    }
    return true;
  }

 private:
  const BodyTrajectory& traj_;
  double t_bar_;
  Eigen::Vector3d q_L_;
  Eigen::Vector3d L_B_to_G_;
  double R_ball_;
  double inv_sigma_r_;
};

}  // namespace clic_calib
