#pragma once

#include <Eigen/Core>
#include <string>

namespace clic_calib {

/** @brief Pinhole intrinsics as fx, fy, cx, cy. */
struct PinholeIntrinsics {
  double fx = 1.0;
  double fy = 1.0;
  double cx = 0.0;
  double cy = 0.0;
};

/** @brief Radial-tangential distortion (OpenCV k1,k2,p1,p2). */
struct RadtanDistortion {
  double k1 = 0.0;
  double k2 = 0.0;
  double p1 = 0.0;
  double p2 = 0.0;
};

inline Eigen::Vector2d UndistortRadtan(const Eigen::Vector2d& uv_dist,
                                       const RadtanDistortion& d) {
  Eigen::Vector2d x = uv_dist;
  for (int iter = 0; iter < 5; ++iter) {
    const double r2 = x.squaredNorm();
    const double r4 = r2 * r2;
    const double radial = 1.0 + d.k1 * r2 + d.k2 * r4;
    const double x0 = (uv_dist.x() - 2 * d.p1 * x.x() * x.y() -
                       d.p2 * (r2 + 2 * x.x() * x.x())) /
                      radial;
    const double y0 = (uv_dist.y() - d.p1 * (r2 + 2 * x.y() * x.y()) -
                       2 * d.p2 * x.x() * x.y()) /
                      radial;
    x << x0, y0;
  }
  return x;
}

/** @brief Project 3D point in camera frame to distorted pixels + 2x3 Jacobian w.r.t. p_C. */
inline Eigen::Vector2d ProjectRadtan(
    const Eigen::Vector3d& p_C, const PinholeIntrinsics& K,
    const RadtanDistortion& d, Eigen::Matrix<double, 2, 3>* J_p_C = nullptr) {
  const double inv_z = 1.0 / p_C.z();
  const double x = p_C.x() * inv_z;
  const double y = p_C.y() * inv_z;

  const double r2 = x * x + y * y;
  const double r4 = r2 * r2;
  const double radial = 1.0 + d.k1 * r2 + d.k2 * r4;
  const double xd = x * radial + 2.0 * d.p1 * x * y + d.p2 * (r2 + 2.0 * x * x);
  const double yd = y * radial + d.p1 * (r2 + 2.0 * y * y) + 2.0 * d.p2 * x * y;

  const Eigen::Vector2d uv(K.fx * xd + K.cx, K.fy * yd + K.cy);

  if (J_p_C) {
    const double dx_dX = inv_z;
    const double dx_dZ = -p_C.x() * inv_z * inv_z;
    const double dy_dY = inv_z;
    const double dy_dZ = -p_C.y() * inv_z * inv_z;

    const double dradial_dx = 2.0 * x * (d.k1 + 2.0 * d.k2 * r2);
    const double dradial_dy = 2.0 * y * (d.k1 + 2.0 * d.k2 * r2);

    const double dxd_dx = radial + x * dradial_dx + 2.0 * d.p1 * y + 6.0 * d.p2 * x;
    const double dxd_dy = x * dradial_dy + 2.0 * d.p1 * x + 2.0 * d.p2 * y;
    const double dyd_dx = y * dradial_dx + 2.0 * d.p1 * x + 2.0 * d.p2 * y;
    const double dyd_dy = radial + y * dradial_dy + 2.0 * d.p1 * y + 6.0 * d.p2 * y;

    Eigen::Matrix<double, 2, 2> J_uv_xy;
    J_uv_xy << K.fx * dxd_dx, K.fx * dxd_dy, K.fy * dyd_dx, K.fy * dyd_dy;

    Eigen::Matrix<double, 2, 3> J_xy_p;
    J_xy_p << dx_dX, 0.0, dx_dZ, 0.0, dy_dY, dy_dZ;

    *J_p_C = J_uv_xy * J_xy_p;
  }

  return uv;
}

}  // namespace clic_calib
