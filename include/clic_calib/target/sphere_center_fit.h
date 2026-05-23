#pragma once

/**
 * Single-scan sphere center from LiDAR inlier points (LiDAR frame).
 * Used by ExtrinsicInitializer (Umeyama) on points from SphereExtractor.
 */

#include <Eigen/Core>

#include <cmath>
#include <vector>

namespace clic_calib {
namespace sphere_center {

inline bool FitCircleCenterInBestPlane(const std::vector<Eigen::Vector3d>& pts,
                                       Eigen::Vector3d* center) {
  if (!center || pts.size() < 3) {
    return false;
  }
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (const auto& p : pts) {
    mean += p;
  }
  mean /= static_cast<double>(pts.size());

  Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
  for (const auto& p : pts) {
    const Eigen::Vector3d d = p - mean;
    cov += d * d.transpose();
  }
  cov /= static_cast<double>(pts.size());
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
  const Eigen::Vector3d u = es.eigenvectors().col(2);
  const Eigen::Vector3d v = es.eigenvectors().col(1);

  double su = 0.0, sv = 0.0, suu = 0.0, svv = 0.0, suv = 0.0;
  double suuu = 0.0, svvv = 0.0, suuv = 0.0, suvv = 0.0;
  for (const auto& p : pts) {
    const Eigen::Vector3d d = p - mean;
    const double x = u.dot(d);
    const double y = v.dot(d);
    su += x;
    sv += y;
    suu += x * x;
    svv += y * y;
    suv += x * y;
    suuu += x * x * x;
    svvv += y * y * y;
    suuv += x * x * y;
    suvv += x * y * y;
  }
  Eigen::Matrix2d A;
  A << suu, suv, suv, svv;
  const Eigen::Vector2d b(0.5 * suuu, 0.5 * svvv);
  const Eigen::Vector2d uv = A.ldlt().solve(b);
  *center = mean + uv(0) * u + uv(1) * v;
  return center->allFinite();
}

/** Stable center for partial sphere arcs with known radius (probe / init path). */
inline bool FitSphereCenterKnownRadius(const std::vector<Eigen::Vector3d>& pts,
                                       double radius_m, Eigen::Vector3d* center,
                                       double* fit_rms_m = nullptr) {
  if (!center || pts.size() < 4) {
    return false;
  }
  if (!FitCircleCenterInBestPlane(pts, center)) {
    return false;
  }
  if (fit_rms_m) {
    double sq = 0.0;
    for (const auto& p : pts) {
      const double ri = (*center - p).norm();
      sq += (ri - radius_m) * (ri - radius_m);
    }
    *fit_rms_m = std::sqrt(sq / static_cast<double>(pts.size()));
  }
  return center->allFinite();
}

}  // namespace sphere_center
}  // namespace clic_calib
