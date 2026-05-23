#pragma once

/**
 * Closed-form Stage-2 extrinsic initialization (probe only).
 * T_LW: per-scan sphere center in L + known p_G^W(t) → Umeyama/Kabsch.
 * T_CW: per-frame AprilTag PnP (tag-local IPPE/EPnP) → aggregate T_CW.
 */

#include "diagnostic/two_stage_probe_common.hpp"

#include <clic_calib/sensor_data/apriltag_observation.h>
#include <clic_calib/sensor_data/lidar_target_observation.h>
#include <clic_calib/spline/trajectory.h>
#include <clic_calib/utils/camera_projection.h>
#include <clic_calib/utils/noise_model.h>

#include <opencv2/calib3d.hpp>

#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace clic_calib {
namespace two_stage_probe {

struct UmeyamaRigid {
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t = Eigen::Vector3d::Zero();
  double rms_m = 0.0;
};

/** Align Q ≈ R*P + t columnwise (3×N). */
inline UmeyamaRigid UmeyamaAlign(const Eigen::MatrixXd& P,
                                 const Eigen::MatrixXd& Q) {
  UmeyamaRigid out;
  const int n = static_cast<int>(P.cols());
  if (n < 3) {
    return out;
  }
  const Eigen::Vector3d p_mean = P.rowwise().mean();
  const Eigen::Vector3d q_mean = Q.rowwise().mean();
  const Eigen::MatrixXd Pc = P.colwise() - p_mean;
  const Eigen::MatrixXd Qc = Q.colwise() - q_mean;
  const Eigen::Matrix3d H = Pc * Qc.transpose();
  const Eigen::JacobiSVD<Eigen::Matrix3d> svd(
      H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d R = svd.matrixV() * svd.matrixU().transpose();
  if (R.determinant() < 0.0) {
    Eigen::Matrix3d V = svd.matrixV();
    V.col(2) *= -1.0;
    R = V * svd.matrixU().transpose();
  }
  out.R = R;
  out.t = q_mean - R * p_mean;
  double sq = 0.0;
  for (int i = 0; i < n; ++i) {
    sq += (Q.col(i) - (out.R * P.col(i) + out.t)).squaredNorm();
  }
  out.rms_m = std::sqrt(sq / static_cast<double>(n));
  return out;
}

/** Circle center in the best-fit plane (stable for coplanar sphere arcs). */
inline bool FitCircleCenterInBestPlane(
    const std::vector<Eigen::Vector3d>& pts, Eigen::Vector3d* center) {
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

/** Sphere center from a scan cluster with known radius. */
inline bool FitSphereCenterKnownRadius(const std::vector<Eigen::Vector3d>& pts,
                                       double radius_m,
                                       Eigen::Vector3d* center,
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

/** Linear least-squares sphere center (known radius, for diagnostics). */
inline bool FitSphereCenterLinear(const std::vector<Eigen::Vector3d>& pts,
                                  double radius_m,
                                  Eigen::Vector3d* center,
                                  double* fit_rms_m = nullptr) {
  if (!center || pts.size() < 4) {
    return false;
  }
  Eigen::MatrixXd A(static_cast<int>(pts.size()), 4);
  Eigen::VectorXd b(static_cast<int>(pts.size()));
  for (size_t i = 0; i < pts.size(); ++i) {
    A.row(static_cast<int>(i)) << 2.0 * pts[i].x(), 2.0 * pts[i].y(),
        2.0 * pts[i].z(), 1.0;
    b(static_cast<int>(i)) = pts[i].squaredNorm();
  }
  const Eigen::Vector4d x =
      A.colPivHouseholderQr().solve(b);
  *center = x.head<3>();
  const double B = x(3);
  const double c2 = center->squaredNorm();
  if (fit_rms_m) {
    double sq = 0.0;
    for (const auto& p : pts) {
      const double ri = (*center - p).norm();
      sq += (ri - radius_m) * (ri - radius_m);
    }
    *fit_rms_m = std::sqrt(sq / static_cast<double>(pts.size()));
  }
  (void)B;
  (void)c2;
  return center->allFinite();
}

inline Eigen::Vector3d TagCornerBody(int corner_idx,
                                     const Eigen::Vector3d& L_B_to_G,
                                     const Eigen::Vector3d& L_G_to_M) {
  static const Eigen::Vector3d kCorners[4] = {
      Eigen::Vector3d(-0.025, -0.025, 0.0),
      Eigen::Vector3d(0.025, -0.025, 0.0),
      Eigen::Vector3d(0.025, 0.025, 0.0),
      Eigen::Vector3d(-0.025, 0.025, 0.0)};
  return L_B_to_G + L_G_to_M + kCorners[corner_idx % 4];
}

struct GeometricInitReport {
  CoarseExtrinsicInit init;
  UmeyamaRigid lw_umeyama;
  int lw_pairs = 0;
  int pnp_frames_ok = 0;
  int pnp_frames_total = 0;
  double pnp_best_frame_reproj_px = 0.0;
  double pnp_reproj_px_rms = 0.0;
};

inline SE3d SE3FromCvRvecTvec(const cv::Mat& rvec, const cv::Mat& tvec) {
  cv::Mat Rmat;
  cv::Rodrigues(rvec, Rmat);
  Eigen::Matrix3d R;
  Eigen::Vector3d t;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      R(r, c) = Rmat.at<double>(r, c);
    }
    t(r) = tvec.at<double>(r);
  }
  return SE3d(SO3d(R), t);
}

inline const std::array<cv::Point3f, 4>& TagLocalCornersCv() {
  static const std::array<cv::Point3f, 4> k = {{
      cv::Point3f(-0.025f, -0.025f, 0.f), cv::Point3f(0.025f, -0.025f, 0.f),
      cv::Point3f(0.025f, 0.025f, 0.f), cv::Point3f(-0.025f, 0.025f, 0.f)}};
  return k;
}

inline double FrameReprojRmsPx(
    const SE3d& T_CW, const BodyTrajectory& traj, double t_world,
    const Eigen::Vector3d& L_B_to_G, const Eigen::Vector3d& L_G_to_M,
    const AprilTagObservation& det, const PinholeIntrinsics& K,
    const RadtanDistortion& dist) {
  const SE3d T_WB = traj.pose_wb(t_world);
  double sq = 0.0;
  for (int c = 0; c < 4; ++c) {
    const Eigen::Vector3d p_W = T_WB * TagCornerBody(c, L_B_to_G, L_G_to_M);
    const Eigen::Vector2d uv_pred =
        ProjectRadtan(T_CW * p_W, K, dist, nullptr);
    sq += (uv_pred - det.corners_pixel_[c]).squaredNorm();
  }
  return std::sqrt(sq / 4.0);
}

/** World pose of tag origin (marker center) at body time t_world. */
inline SE3d MarkerWorldPose(const BodyTrajectory& traj, double t_world,
                            const Eigen::Vector3d& L_B_to_G,
                            const Eigen::Vector3d& L_G_to_M) {
  return traj.pose_wb(t_world) * SE3d(SO3d(), L_B_to_G + L_G_to_M);
}

inline double GlobalTagReprojRmsPx(
    const SE3d& T_CW, const BodyTrajectory& traj, double t_d_C_pair_s,
    const Eigen::Vector3d& L_B_to_G, const Eigen::Vector3d& L_G_to_M,
    const std::vector<AprilTagObservation>& tags, const PinholeIntrinsics& K,
    const RadtanDistortion& dist) {
  if (tags.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  double sq = 0.0;
  for (const auto& det : tags) {
    const double t_world = det.t_sensor_ - t_d_C_pair_s;
    const double r =
        FrameReprojRmsPx(T_CW, traj, t_world, L_B_to_G, L_G_to_M, det, K, dist);
    sq += r * r;
  }
  return std::sqrt(sq / static_cast<double>(tags.size()));
}

/** Quaternion + translation mean on SE(3) (probe aggregation). */
inline SE3d AverageSE3(const std::vector<SE3d>& transforms) {
  if (transforms.empty()) {
    return SE3d();
  }
  Eigen::Vector3d t_sum = Eigen::Vector3d::Zero();
  Eigen::Matrix4d Q_acc = Eigen::Matrix4d::Zero();
  for (const SE3d& T : transforms) {
    t_sum += T.translation();
    Eigen::Quaterniond q(T.so3().unit_quaternion());
    if (q.w() < 0.0) {
      q.coeffs() *= -1.0;
    }
    Q_acc += q.coeffs() * q.coeffs().transpose();
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> es(Q_acc);
  Eigen::Quaterniond q_avg(es.eigenvectors().col(3));
  if (q_avg.w() < 0.0) {
    q_avg.coeffs() *= -1.0;
  }
  q_avg.normalize();
  return SE3d(SO3d(q_avg), t_sum / static_cast<double>(transforms.size()));
}

/** Batch PnP on all world corners (used as aggregate candidate). */
inline bool InitTCWFromAprilTagPnPBatch(
    const BodyTrajectory& traj,
    const std::vector<AprilTagObservation>& tags,
    const Eigen::Vector3d& L_B_to_G, const Eigen::Vector3d& L_G_to_M,
    double t_d_C_s, const PinholeIntrinsics& K, const RadtanDistortion& dist,
    SE3d* T_CW, int* num_points, double* reproj_rms_px);

/** Per-frame tag-local PnP (IPPE/EPnP) → T_CW; aggregate by global reproj. */
inline bool InitTCWFromAprilTagPnPPerFrame(
    const BodyTrajectory& traj,
    const std::vector<AprilTagObservation>& tags,
    const Eigen::Vector3d& L_B_to_G, const Eigen::Vector3d& L_G_to_M,
    double t_d_C_pair_s, const PinholeIntrinsics& K, const RadtanDistortion& dist,
    SE3d* T_CW, int* frames_ok = nullptr, int* frames_total = nullptr,
    double* best_frame_reproj_px = nullptr, double* all_frames_reproj_px = nullptr) {
  if (!T_CW || tags.empty()) {
    return false;
  }
  const cv::Mat Kmat = (cv::Mat_<double>(3, 3) << K.fx, 0, K.cx, 0, K.fy, K.cy,
                        0, 0, 1);
  const cv::Mat dist_coeffs =
      (cv::Mat_<double>(1, 4) << dist.k1, dist.k2, dist.p1, dist.p2);
  const auto& tag_local_corners = TagLocalCornersCv();
  const std::vector<cv::Point3f> tag_local(tag_local_corners.begin(),
                                           tag_local_corners.end());

  struct Candidate {
    SE3d T_CW;
    double self_reproj_px = 0.0;
  };
  std::vector<Candidate> candidates;
  std::vector<SE3d> per_frame_solutions;
  candidates.reserve(tags.size() + 2);
  per_frame_solutions.reserve(tags.size());

  for (const auto& det : tags) {
    const double t_world = det.t_sensor_ - t_d_C_pair_s;
    const SE3d T_WM = MarkerWorldPose(traj, t_world, L_B_to_G, L_G_to_M);

    std::vector<cv::Point2f> img;
    img.reserve(4);
    for (int c = 0; c < 4; ++c) {
      img.emplace_back(static_cast<float>(det.corners_pixel_[c].x()),
                       static_cast<float>(det.corners_pixel_[c].y()));
    }

    cv::Mat rvec, tvec;
    bool ok = cv::solvePnP(tag_local, img, Kmat, dist_coeffs, rvec, tvec, false,
                           cv::SOLVEPNP_IPPE_SQUARE);
    if (!ok) {
      ok = cv::solvePnP(tag_local, img, Kmat, dist_coeffs, rvec, tvec, false,
                        cv::SOLVEPNP_EPNP);
      if (ok) {
        cv::solvePnP(tag_local, img, Kmat, dist_coeffs, rvec, tvec, true,
                     cv::SOLVEPNP_ITERATIVE);
      }
    }
    if (!ok) {
      continue;
    }
    const SE3d T_CTag = SE3FromCvRvecTvec(rvec, tvec);
    const SE3d T_CW_frame = T_CTag * T_WM.inverse();
    const double self_reproj =
        FrameReprojRmsPx(T_CW_frame, traj, t_world, L_B_to_G, L_G_to_M, det, K,
                         dist);
    candidates.push_back({T_CW_frame, self_reproj});
    per_frame_solutions.push_back(T_CW_frame);
  }

  if (per_frame_solutions.size() >= 3) {
    const SE3d T_mean = AverageSE3(per_frame_solutions);
    const double mean_reproj = GlobalTagReprojRmsPx(
        T_mean, traj, t_d_C_pair_s, L_B_to_G, L_G_to_M, tags, K, dist);
    candidates.push_back({T_mean, mean_reproj});
  }

  SE3d T_batch;
  double batch_reproj = 0.0;
  bool batch_added = false;
  if (InitTCWFromAprilTagPnPBatch(traj, tags, L_B_to_G, L_G_to_M, t_d_C_pair_s,
                                  K, dist, &T_batch, nullptr, &batch_reproj)) {
    candidates.push_back({T_batch, batch_reproj});
    batch_added = true;
  }

  if (frames_total) {
    *frames_total = static_cast<int>(tags.size());
  }
  if (frames_ok) {
    *frames_ok = static_cast<int>(per_frame_solutions.size());
  }
  if (candidates.empty()) {
    return false;
  }

  const Candidate* best = &candidates[0];
  double best_global_reproj = std::numeric_limits<double>::infinity();
  for (const auto& cand : candidates) {
    const double global_reproj = GlobalTagReprojRmsPx(
        cand.T_CW, traj, t_d_C_pair_s, L_B_to_G, L_G_to_M, tags, K, dist);
    if (global_reproj < best_global_reproj) {
      best_global_reproj = global_reproj;
      best = &cand;
    }
  }
  *T_CW = best->T_CW;

  if (best_frame_reproj_px) {
    *best_frame_reproj_px = best->self_reproj_px;
  }
  if (all_frames_reproj_px) {
    *all_frames_reproj_px = best_global_reproj;
  }
  (void)batch_added;
  return true;
}

/** T_LW from {p_G^L(t)} ↔ {p_G^W(t)} via Umeyama; p_G^W from fixed trajectory. */
inline bool InitTLWFromLidarSphereCenters(
    const BodyTrajectory& traj,
    const std::vector<LiDARTargetObservation>& lidar,
    const Eigen::Vector3d& L_B_to_G, double t_d_L_s, double sphere_radius_m,
    SE3d* T_LW, UmeyamaRigid* umeyama = nullptr, int* num_pairs = nullptr) {
  if (!T_LW) {
    return false;
  }
  std::vector<Eigen::Vector3d> p_L_list;
  std::vector<Eigen::Vector3d> p_W_list;
  p_L_list.reserve(lidar.size());
  p_W_list.reserve(lidar.size());
  for (const auto& scan : lidar) {
    if (scan.points_L_.size() < 8) {
      continue;
    }
    Eigen::Vector3d c_L;
    double fit_rms = 0.0;
    if (!FitSphereCenterKnownRadius(scan.points_L_, sphere_radius_m, &c_L,
                                    &fit_rms)) {
      continue;
    }
    if (fit_rms > 0.04) {
      continue;
    }
    const double t_world = scan.t_sensor_ - t_d_L_s;
    p_L_list.push_back(c_L);
    p_W_list.push_back(traj.sphere_center_w(t_world, L_B_to_G));
  }
  if (p_L_list.size() < 3) {
    return false;
  }
  Eigen::MatrixXd P(3, static_cast<int>(p_L_list.size()));
  Eigen::MatrixXd Q(3, static_cast<int>(p_W_list.size()));
  for (size_t i = 0; i < p_L_list.size(); ++i) {
    P.col(static_cast<int>(i)) = p_L_list[i];
    Q.col(static_cast<int>(i)) = p_W_list[i];
  }
  const UmeyamaRigid u = UmeyamaAlign(P, Q);
  if (umeyama) {
    *umeyama = u;
  }
  if (num_pairs) {
    *num_pairs = static_cast<int>(p_L_list.size());
  }
  const SE3d T_WL(u.R, u.t);
  *T_LW = T_WL.inverse();
  return true;
}

/** Batch PnP on all world corners (diagnostic fallback). */
inline bool InitTCWFromAprilTagPnPBatch(
    const BodyTrajectory& traj,
    const std::vector<AprilTagObservation>& tags,
    const Eigen::Vector3d& L_B_to_G, const Eigen::Vector3d& L_G_to_M,
    double t_d_C_s, const PinholeIntrinsics& K, const RadtanDistortion& dist,
    SE3d* T_CW, int* num_points = nullptr, double* reproj_rms_px = nullptr) {
  if (!T_CW) {
    return false;
  }
  std::vector<cv::Point3f> obj;
  std::vector<cv::Point2f> img;
  obj.reserve(tags.size() * 4);
  img.reserve(tags.size() * 4);
  for (const auto& det : tags) {
    const double t_world = det.t_sensor_ - t_d_C_s;
    const SE3d T_WB = traj.pose_wb(t_world);
    for (int c = 0; c < 4; ++c) {
      const Eigen::Vector3d p_W =
          T_WB * TagCornerBody(c, L_B_to_G, L_G_to_M);
      obj.emplace_back(static_cast<float>(p_W.x()),
                       static_cast<float>(p_W.y()),
                       static_cast<float>(p_W.z()));
      const Eigen::Vector2d uv = det.corners_pixel_[c];
      img.emplace_back(static_cast<float>(uv.x()), static_cast<float>(uv.y()));
    }
  }
  if (obj.size() < 4) {
    return false;
  }
  cv::Mat Kmat = (cv::Mat_<double>(3, 3) << K.fx, 0, K.cx, 0, K.fy, K.cy, 0, 0,
                  1);
  cv::Mat dist_coeffs =
      (cv::Mat_<double>(1, 4) << dist.k1, dist.k2, dist.p1, dist.p2);
  cv::Mat rvec, tvec;
  const bool ok = cv::solvePnP(obj, img, Kmat, dist_coeffs, rvec, tvec, false,
                               cv::SOLVEPNP_ITERATIVE);
  if (!ok) {
    return false;
  }
  cv::Mat Rmat;
  cv::Rodrigues(rvec, Rmat);
  Eigen::Matrix3d R;
  Eigen::Vector3d t;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      R(r, c) = Rmat.at<double>(r, c);
    }
    t(r) = tvec.at<double>(r);
  }
  *T_CW = SE3d(SO3d(R), t);
  if (num_points) {
    *num_points = static_cast<int>(obj.size());
  }
  if (reproj_rms_px) {
    double sq = 0.0;
    for (size_t i = 0; i < obj.size(); ++i) {
      const Eigen::Vector3d p_W(obj[i].x, obj[i].y, obj[i].z);
      const Eigen::Vector3d p_C = (*T_CW) * p_W;
      const Eigen::Vector2d uv_pred =
          ProjectRadtan(p_C, K, dist, nullptr);
      const Eigen::Vector2d uv_obs(img[i].x, img[i].y);
      sq += (uv_pred - uv_obs).squaredNorm();
    }
    *reproj_rms_px = std::sqrt(sq / static_cast<double>(obj.size()));
  }
  return true;
}

inline GeometricInitReport MakeGeometricExtrinsicInit(
    const BodyTrajectory& traj,
    const std::vector<LiDARTargetObservation>& lidar,
    const std::vector<AprilTagObservation>& tags,
    const LeverArmConfig& levers, double sphere_radius_m,
    const CoarseExtrinsicInit& t_d_prior) {
  GeometricInitReport rep;
  const PinholeIntrinsics K{600.0, 600.0, 320.0, 240.0};
  const RadtanDistortion dist;
  const Eigen::Vector3d L_G_to_M = levers.L_G_to_M.at(0);

  SE3d T_LW;
  // World-time pairing: nominal rig delay from yaml (not GT-optimized t_d).
  // Stage-2 init state t_d^L := 0 (unknown; refined in joint solve).
  if (!InitTLWFromLidarSphereCenters(traj, lidar, levers.L_B_to_G,
                                     t_d_prior.t_d_L_s, sphere_radius_m, &T_LW,
                                     &rep.lw_umeyama, &rep.lw_pairs)) {
    throw std::runtime_error(
        "MakeGeometricExtrinsicInit: LiDAR Umeyama failed");
  }
  rep.init.t_d_L_s = 0.0;
  rep.init.T_LW = T_LW;

  SE3d T_CW;
  if (!InitTCWFromAprilTagPnPPerFrame(traj, tags, levers.L_B_to_G, L_G_to_M,
                                      t_d_prior.t_d_C_s, K, dist, &T_CW,
                                      &rep.pnp_frames_ok, &rep.pnp_frames_total,
                                      &rep.pnp_best_frame_reproj_px,
                                      &rep.pnp_reproj_px_rms)) {
    throw std::runtime_error("MakeGeometricExtrinsicInit: PnP failed");
  }
  rep.init.t_d_C_s = 0.0;
  rep.init.T_CW = T_CW;
  return rep;
}

/** Probe-only: reproject tag pixels through a fixed trajectory + GT extrinsic. */
inline std::vector<AprilTagObservation> SynthesizeTagObsForTrajectory(
    const std::vector<AprilTagObservation>& template_obs,
    const BodyTrajectory& traj, const SE3d& T_CW,
    const Eigen::Vector3d& L_B_to_G, const Eigen::Vector3d& L_G_to_M,
    double t_d_C_s, const PinholeIntrinsics& K, const RadtanDistortion& dist,
    const NoiseModel& noise, uint32_t seed) {
  std::vector<AprilTagObservation> out = template_obs;
  std::mt19937 rng(seed + 4242u);
  for (auto& det : out) {
    const double t_world = det.t_sensor_ - t_d_C_s;
    const SE3d T_WB = traj.pose_wb(t_world);
    for (int c = 0; c < 4; ++c) {
      const Eigen::Vector3d p_W = T_WB * TagCornerBody(c, L_B_to_G, L_G_to_M);
      const Eigen::Vector3d p_C = T_CW * p_W;
      det.corners_pixel_[c] =
          ProjectRadtan(p_C, K, dist, nullptr) + noise.SamplePixelNoise(rng);
    }
  }
  return out;
}

}  // namespace two_stage_probe
}  // namespace clic_calib
