#pragma once

#include <clic_calib/factor/fixed_traj_sphere_factor.h>
#include <clic_calib/spline/trajectory.h>
#include <clic_calib/utils/sophus_utils.hpp>

#include <ceres/ceres.h>
#include <Eigen/Core>

namespace clic_calib {

/** Stage-2 body-centroid residual; trajectory fixed (§5.2). */
class FixedTrajBodyCentroidFactor : public ceres::SizedCostFunction<3, 1, 4, 3> {
 public:
  FixedTrajBodyCentroidFactor(const BodyTrajectory& traj, double t_bar,
                              const Eigen::Vector3d& c_L,
                              const Eigen::Vector3d& L_B_to_body,
                              const Eigen::Matrix3d& sqrt_info)
      : traj_(traj),
        t_bar_(t_bar),
        c_L_(c_L),
        L_B_to_body_(L_B_to_body),
        sqrt_info_(sqrt_info) {}

  bool Evaluate(double const* const* parameters, double* residuals,
                double** jacobians) const override {
    const double t_d = parameters[0][0];
    const double t_eval = t_bar_ - t_d;
    const Eigen::Vector3d p_W =
        traj_.position_wb(t_eval) + traj_.rotation_wb(t_eval) * L_B_to_body_;
    const Eigen::Vector3d v_W = WorldPointVelocity(traj_, t_eval, L_B_to_body_);
    const Eigen::Map<const Eigen::Quaterniond> q_LW(parameters[1]);
    const Eigen::Map<const Eigen::Vector3d> t_LW(parameters[2]);
    const SE3d T_LW(SO3d(q_LW), t_LW);
    const Eigen::Vector3d c_hat = T_LW * p_W;
    const Eigen::Vector3d r = c_L_ - c_hat;
    Eigen::Map<Eigen::Vector3d> e(residuals);
    e = sqrt_info_ * r;

    if (!jacobians) {
      return true;
    }
    const Eigen::Matrix3d R_LW = T_LW.so3().matrix();
    if (jacobians[0]) {
      Eigen::Map<Eigen::Matrix<double, 3, 1>> J_td(jacobians[0]);
      J_td = sqrt_info_ * (R_LW * v_W);
    }
    if (jacobians[1]) {
      Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> J_q(jacobians[1]);
      J_q.setZero();
      J_q.block<3, 3>(0, 0) = sqrt_info_ * (R_LW * SO3d::hat(p_W));
    }
    if (jacobians[2]) {
      Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> J_t(jacobians[2]);
      J_t = sqrt_info_ * (-Eigen::Matrix3d::Identity());
    }
    return true;
  }

 private:
  const BodyTrajectory& traj_;
  double t_bar_;
  Eigen::Vector3d c_L_;
  Eigen::Vector3d L_B_to_body_;
  Eigen::Matrix3d sqrt_info_;
};

}  // namespace clic_calib
