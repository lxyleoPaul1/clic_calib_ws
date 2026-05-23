#pragma once

#include <clic_calib/factor/fixed_traj_sphere_factor.h>
#include <clic_calib/utils/camera_projection.h>
#include <clic_calib/utils/sophus_utils.hpp>

#include <ceres/ceres.h>
#include <Eigen/Core>

namespace clic_calib {

/** Stage-2 AprilTag corner; trajectory fixed, t_d shifts evaluation time. */
class FixedTrajAprilTagFactor : public ceres::SizedCostFunction<2, 1, 4, 3> {
 public:
  FixedTrajAprilTagFactor(const BodyTrajectory& traj, double t_bar,
                          const Eigen::Vector2d& u_obs,
                          const Eigen::Vector3d& L_B_to_G,
                          const Eigen::Vector3d& L_G_to_M,
                          const PinholeIntrinsics& K,
                          const RadtanDistortion& dist, double inv_sigma_pix)
      : traj_(traj),
        t_bar_(t_bar),
        u_obs_(u_obs),
        L_B_to_G_(L_B_to_G),
        L_G_to_M_(L_G_to_M),
        K_(K),
        dist_(dist),
        inv_sigma_pix_(inv_sigma_pix) {}

  bool Evaluate(double const* const* parameters, double* residuals,
                double** jacobians) const override {
    const double t_d = parameters[0][0];
    const double t_eval = t_bar_ - t_d;
    const Eigen::Vector3d p_M_W =
        traj_.marker_center_w(t_eval, L_B_to_G_, L_G_to_M_);
    const Eigen::Vector3d dp_MW_dt =
        -WorldPointVelocity(traj_, t_eval, L_B_to_G_ + L_G_to_M_);
    const Eigen::Map<const Eigen::Quaterniond> q_CW(parameters[1]);
    const Eigen::Map<const Eigen::Vector3d> t_CW(parameters[2]);
    const SE3d T_CW(SO3d(q_CW), t_CW);
    const Eigen::Vector3d p_M_C = T_CW * p_M_W;
    Eigen::Matrix<double, 2, 3> J_proj;
    const Eigen::Vector2d u_pred =
        ProjectRadtan(p_M_C, K_, dist_, jacobians ? &J_proj : nullptr);
    Eigen::Map<Eigen::Vector2d> r(residuals);
    r = inv_sigma_pix_ * (u_obs_ - u_pred);
    if (!jacobians) {
      return true;
    }
    const Eigen::Matrix3d R_CW = T_CW.so3().matrix();
    const Eigen::Matrix3d R_hat = R_CW * SO3d::hat(p_M_W);
    if (jacobians[0]) {
      Eigen::Map<Eigen::Matrix<double, 2, 1>> J_td(jacobians[0]);
      J_td = inv_sigma_pix_ * J_proj * R_CW * dp_MW_dt;
    }
    if (jacobians[1]) {
      Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> J_q(jacobians[1]);
      J_q.setZero();
      J_q.block<2, 3>(0, 0) = inv_sigma_pix_ * J_proj * (-R_hat);
    }
    if (jacobians[2]) {
      Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> J_t(jacobians[2]);
      J_t = inv_sigma_pix_ * J_proj * R_CW;
    }
    return true;
  }

 private:
  const BodyTrajectory& traj_;
  double t_bar_;
  Eigen::Vector2d u_obs_;
  Eigen::Vector3d L_B_to_G_;
  Eigen::Vector3d L_G_to_M_;
  PinholeIntrinsics K_;
  RadtanDistortion dist_;
  double inv_sigma_pix_;
};

}  // namespace clic_calib
