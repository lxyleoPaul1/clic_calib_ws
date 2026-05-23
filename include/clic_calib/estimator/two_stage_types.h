#pragma once

#include <clic_calib/utils/sophus_utils.hpp>

#include <Eigen/Core>

namespace clic_calib {

struct CoarseExtrinsicInit {
  SE3d T_LW;
  SE3d T_CW;
  double t_d_L_s = 0.0;
  double t_d_C_s = 0.0;
};

/** Mutable extrinsic blocks for Stage-2 Ceres (quaternion + translation + t_d). */
struct ExtrinsicOptimizeState {
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  Eigen::Vector3d t = Eigen::Vector3d::Zero();
  double t_d = 0.0;

  SE3d AsSE3() const { return SE3d(q, t); }

  void SetFromSE3(const SE3d& T) {
    q = T.unit_quaternion();
    t = T.translation();
  }

  void SetFromInit(const CoarseExtrinsicInit& init, bool is_lidar) {
    if (is_lidar) {
      SetFromSE3(init.T_LW);
      t_d = init.t_d_L_s;
    } else {
      SetFromSE3(init.T_CW);
      t_d = init.t_d_C_s;
    }
  }
};

struct UmeyamaRigid {
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t = Eigen::Vector3d::Zero();
  double rms_m = 0.0;
};

struct GeometricInitReport {
  CoarseExtrinsicInit init;
  UmeyamaRigid lw_umeyama;
  int lw_pairs = 0;
  int pnp_frames_ok = 0;
  int pnp_frames_total = 0;
  double pnp_best_frame_reproj_px = 0.0;
  double pnp_reproj_px_rms = 0.0;
};

}  // namespace clic_calib
