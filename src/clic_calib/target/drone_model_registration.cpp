#include <clic_calib/target/drone_model_registration.h>

#include <ceres/ceres.h>

#include <cmath>

namespace clic_calib {
namespace {

double PointToAabbSurfaceDistance(const Eigen::Vector3d& p,
                                  const Eigen::Vector3d& c,
                                  const Eigen::Vector3d& half) {
  const Eigen::Vector3d d = (p - c).cwiseAbs() - half;
  if (d.maxCoeff() <= 0.0) {
    return 0.0;
  }
  const Eigen::Vector3d q = d.cwiseMax(0.0);
  return q.norm();
}

struct BoxCenterCost {
  BoxCenterCost(const Eigen::Vector3d& p_B, const Eigen::Vector3d& half)
      : p_B_(p_B), half_(half) {}

  template <typename T>
  bool operator()(const T* const c, T* residual) const {
    Eigen::Matrix<T, 3, 1> p = p_B_.cast<T>();
    Eigen::Matrix<T, 3, 1> h = half_.cast<T>();
    Eigen::Matrix<T, 3, 1> center(c[0], c[1], c[2]);
    Eigen::Matrix<T, 3, 1> d = (p - center).cwiseAbs() - h;
    if (d[0] <= T(0) && d[1] <= T(0) && d[2] <= T(0)) {
      residual[0] = T(0);
      return true;
    }
    Eigen::Matrix<T, 3, 1> q;
    for (int i = 0; i < 3; ++i) {
      q[i] = d[i] > T(0) ? d[i] : T(0);
    }
    residual[0] = q.norm();
    return true;
  }

  Eigen::Vector3d p_B_;
  Eigen::Vector3d half_;
};

std::vector<Eigen::Vector3d> PointsBodyFrame(
    const std::vector<Eigen::Vector3d>& points_L, const SE3d& T_LW,
    const SE3d& T_WB) {
  const SE3d T_WL = T_LW.inverse();
  std::vector<Eigen::Vector3d> out;
  out.reserve(points_L.size());
  for (const auto& p_L : points_L) {
    out.push_back(T_WB.so3().inverse() * (T_WL * p_L - T_WB.translation()));
  }
  return out;
}

int DominantFaceAxis(const Eigen::Vector3d& p_B, const Eigen::Vector3d& center,
                     const Eigen::Vector3d& half, double* sign_out) {
  const Eigen::Vector3d q = p_B - center;
  double best = -1.0;
  int best_axis = 0;
  double best_sign = 1.0;
  for (int axis = 0; axis < 3; ++axis) {
    for (double sign : {-1.0, 1.0}) {
      const double coord = sign * q[axis];
      const double dist = std::abs(coord) - half[axis];
      if (dist > best) {
        best = dist;
        best_axis = axis;
        best_sign = sign;
      }
    }
  }
  if (sign_out) {
    *sign_out = best_sign;
  }
  return best_axis;
}

bool IsFaceVisible(int axis, double sign, const Eigen::Vector3d& view_b) {
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
  normal[axis] = sign;
  return normal.dot(view_b) > 0.05;
}

DroneModelRegistrationResult OptimizeBoxCenter(
    const std::vector<Eigen::Vector3d>& pts_B, const SE3d& T_LW,
    const SE3d& T_WB, const BodyModelConfig& model) {
  DroneModelRegistrationResult out;
  if (pts_B.size() < 4) {
    return out;
  }
  Eigen::Vector3d center = model.centroid_lever_arm_B;
  if (center.norm() < 1e-9) {
    center = Eigen::Vector3d(0.0, 0.0, -0.12);
  }

  ceres::Problem problem;
  for (const auto& p_B : pts_B) {
    auto* cost = new ceres::AutoDiffCostFunction<BoxCenterCost, 1, 3>(
        new BoxCenterCost(p_B, model.half_extent_B));
    problem.AddResidualBlock(cost, nullptr, center.data());
  }

  ceres::Solver::Options opts;
  opts.max_num_iterations = 50;
  opts.minimizer_progress_to_stdout = false;
  ceres::Solver::Summary summary;
  ceres::Solve(opts, &problem, &summary);

  double sq = 0.0;
  for (const auto& p_B : pts_B) {
    const double d =
        PointToAabbSurfaceDistance(p_B, center, model.half_extent_B);
    sq += d * d;
  }

  out.fitted_center_B = center;
  out.centroid_L = T_LW * (T_WB * center);
  out.surface_rmse_m = std::sqrt(sq / static_cast<double>(pts_B.size()));
  out.ok = summary.IsSolutionUsable();
  return out;
}

}  // namespace

DroneModelRegistrationResult DroneModelRegistration::Register(
    const std::vector<Eigen::Vector3d>& points_L, const SE3d& T_LW,
    const SE3d& T_WB, const BodyModelConfig& model) {
  DroneModelRegistrationResult out;
  if (points_L.size() < 8) {
    return out;
  }

  const std::vector<Eigen::Vector3d> pts_B =
      PointsBodyFrame(points_L, T_LW, T_WB);
  return OptimizeBoxCenter(pts_B, T_LW, T_WB, model);
}

DroneModelRegistrationResult DroneModelRegistration::RegisterVisibilityAware(
    const std::vector<Eigen::Vector3d>& points_L, const SE3d& T_LW,
    const SE3d& T_WB, const BodyModelConfig& model) {
  if (points_L.size() < 8) {
    return {};
  }
  const std::vector<Eigen::Vector3d> pts_B =
      PointsBodyFrame(points_L, T_LW, T_WB);
  Eigen::Vector3d center0 = model.centroid_lever_arm_B;
  if (center0.norm() < 1e-9) {
    center0 = Eigen::Vector3d(0.0, 0.0, -0.12);
  }
  const SE3d T_WL = T_LW.inverse();
  const Eigen::Vector3d lidar_w = T_WL * Eigen::Vector3d::Zero();
  const Eigen::Vector3d view_w =
      (T_WB * center0 - lidar_w).normalized();
  const Eigen::Vector3d view_b = T_WB.so3().inverse() * view_w;

  std::vector<Eigen::Vector3d> visible_pts_B;
  visible_pts_B.reserve(pts_B.size());
  for (const auto& p_B : pts_B) {
    double sign = 1.0;
    const int axis = DominantFaceAxis(p_B, center0, model.half_extent_B, &sign);
    if (IsFaceVisible(axis, sign, view_b)) {
      visible_pts_B.push_back(p_B);
    }
  }
  if (visible_pts_B.size() < 4) {
    return Register(points_L, T_LW, T_WB, model);
  }
  return OptimizeBoxCenter(visible_pts_B, T_LW, T_WB, model);
}

std::vector<BodyClusterObservation> DroneModelRegistration::ApplyToObservations(
    const std::vector<BodyClusterObservation>& observations,
    const BodyTrajectory& traj, const SE3d& T_LW, double t_d_L_s,
    const BodyModelConfig& model) {
  std::vector<BodyClusterObservation> out;
  out.reserve(observations.size());
  for (const auto& obs : observations) {
    BodyClusterObservation reg = obs;
    const double t_world = obs.t_sensor_ - t_d_L_s;
    const SE3d T_WB = traj.pose_wb(t_world);
    const std::vector<Eigen::Vector3d>& pts =
        obs.raw_points_L_.empty()
            ? std::vector<Eigen::Vector3d>{obs.centroid_L_}
            : obs.raw_points_L_;
    const DroneModelRegistrationResult r =
        Register(pts, T_LW, T_WB, model);
    if (r.ok) {
      reg.centroid_L_ = r.centroid_L;
    }
    out.push_back(reg);
  }
  return out;
}

std::vector<BodyClusterObservation>
DroneModelRegistration::ApplyVisibilityAwareToObservations(
    const std::vector<BodyClusterObservation>& observations,
    const BodyTrajectory& traj, const SE3d& T_LW, double t_d_L_s,
    const BodyModelConfig& model) {
  std::vector<BodyClusterObservation> out;
  out.reserve(observations.size());
  for (const auto& obs : observations) {
    BodyClusterObservation reg = obs;
    const double t_world = obs.t_sensor_ - t_d_L_s;
    const SE3d T_WB = traj.pose_wb(t_world);
    const std::vector<Eigen::Vector3d>& pts =
        obs.raw_points_L_.empty()
            ? std::vector<Eigen::Vector3d>{obs.centroid_L_}
            : obs.raw_points_L_;
    const DroneModelRegistrationResult r =
        RegisterVisibilityAware(pts, T_LW, T_WB, model);
    if (r.ok) {
      reg.centroid_L_ = r.centroid_L;
    }
    out.push_back(reg);
  }
  return out;
}

}  // namespace clic_calib
