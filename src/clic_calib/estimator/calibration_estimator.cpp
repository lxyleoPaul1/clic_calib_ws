#include <clic_calib/estimator/calibration_estimator.h>

#include <clic_calib/factor/apriltag_reproj_factor.h>
#include <clic_calib/factor/ceres_local_param.h>
#include <clic_calib/factor/prior_factor.h>
#include <clic_calib/factor/rtk_position_factor.h>
#include <clic_calib/factor/sphere_implicit_factor.h>
#include <clic_calib/factor/trajectory_smoothness_factor.h>
#include <clic_calib/utils/camera_projection.h>
#include <clic_calib/utils/lever_arm.h>
#include <clic_calib/utils/noise_model.h>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <utility>

namespace clic_calib {
namespace {

constexpr double kDegToRad = M_PI / 180.0;

struct SplineConfig {
  double knot_interval_s = 0.05;
  double alpha_p = 0.01;
  double alpha_R = 0.01;
  double lidar_cauchy_scale = 1.0;
  double camera_huber_delta_px = 2.0;
  /** Hard clamp on |t_d| [s]; intersected with spline-support bounds. */
  double t_d_max_abs_s = 0.1;
};

struct TargetGeometryConfig {
  double sphere_radius_m = 0.10;
  std::map<int, std::array<Eigen::Vector3d, 4>> corners_in_marker_frame;
};

struct LidarRigConfig {
  int id = 0;
  SE3d initial_T_LW;
  double initial_t_d_s = 0.0;
  double prior_rot_std_deg = 5.0;
  double prior_trans_std_m = 0.5;
};

struct CameraRigConfig {
  int id = 0;
  PinholeIntrinsics K;
  RadtanDistortion dist;
  SE3d initial_T_CW;
  double initial_t_d_s = 0.0;
  double prior_rot_std_deg = 5.0;
  double prior_trans_std_m = 0.5;
};

SE3d SE3FromXyzRpy(const std::vector<double>& v) {
  if (v.size() != 6) {
    throw std::runtime_error("initial_T_* expects 6 values [x,y,z,roll,pitch,yaw]");
  }
  const Eigen::Vector3d t(v[0], v[1], v[2]);
  const SO3d R = SO3d::rotZ(v[5]) * SO3d::rotY(v[4]) * SO3d::rotX(v[3]);
  return SE3d(R, t);
}

Eigen::Matrix<double, 6, 1> PriorSqrtInfo(double rot_std_deg, double trans_std_m) {
  Eigen::Matrix<double, 6, 1> info;
  const double rot_std = rot_std_deg * kDegToRad;
  info << 1.0 / rot_std, 1.0 / rot_std, 1.0 / rot_std, 1.0 / trans_std_m,
      1.0 / trans_std_m, 1.0 / trans_std_m;
  return info;
}

SplineConfig LoadSplineConfig(const std::string& path) {
  const YAML::Node node = YAML::LoadFile(path);
  SplineConfig cfg;
  if (node["knot_interval_s"]) {
    cfg.knot_interval_s = node["knot_interval_s"].as<double>();
  }
  if (node["alpha_p"]) {
    cfg.alpha_p = node["alpha_p"].as<double>();
  }
  if (node["alpha_R"]) {
    cfg.alpha_R = node["alpha_R"].as<double>();
  }
  if (node["lidar_cauchy_scale"]) {
    cfg.lidar_cauchy_scale = node["lidar_cauchy_scale"].as<double>();
  }
  if (node["camera_huber_delta_px"]) {
    cfg.camera_huber_delta_px = node["camera_huber_delta_px"].as<double>();
  }
  if (node["t_d_max_abs_s"]) {
    cfg.t_d_max_abs_s = node["t_d_max_abs_s"].as<double>();
  }
  return cfg;
}

TargetGeometryConfig LoadTargetGeometry(const std::string& path) {
  const YAML::Node node = YAML::LoadFile(path);
  TargetGeometryConfig cfg;
  if (node["sphere_radius_m"]) {
    cfg.sphere_radius_m = node["sphere_radius_m"].as<double>();
  }
  if (node["apriltag_corners_in_marker_frame"]) {
    for (const auto& item : node["apriltag_corners_in_marker_frame"]) {
      const int tag_id = item.first.as<int>();
      std::array<Eigen::Vector3d, 4> corners;
      int c = 0;
      for (const auto& corner : item.second) {
        if (c >= 4) {
          break;
        }
        corners[c++] = Eigen::Vector3d(corner[0].as<double>(), corner[1].as<double>(),
                                       corner[2].as<double>());
      }
      cfg.corners_in_marker_frame[tag_id] = corners;
    }
  }
  return cfg;
}

void LoadSensorRig(const std::string& path, std::map<int, LidarRigConfig>* lidars,
                   std::map<int, CameraRigConfig>* cameras) {
  const YAML::Node node = YAML::LoadFile(path);
  if (node["lidars"]) {
    for (const auto& L : node["lidars"]) {
      LidarRigConfig cfg;
      cfg.id = L["id"].as<int>();
      if (L["initial_T_LW"]) {
        cfg.initial_T_LW = SE3FromXyzRpy(L["initial_T_LW"].as<std::vector<double>>());
      }
      if (L["t_d_L_s"]) {
        cfg.initial_t_d_s = L["t_d_L_s"].as<double>();
      }
      if (L["prior_rot_std_deg"]) {
        cfg.prior_rot_std_deg = L["prior_rot_std_deg"].as<double>();
      }
      if (L["prior_trans_std_m"]) {
        cfg.prior_trans_std_m = L["prior_trans_std_m"].as<double>();
      }
      (*lidars)[cfg.id] = cfg;
    }
  }
  if (node["cameras"]) {
    for (const auto& C : node["cameras"]) {
      CameraRigConfig cfg;
      cfg.id = C["id"].as<int>();
      if (C["K"]) {
        const auto K = C["K"].as<std::vector<double>>();
        if (K.size() == 4) {
          cfg.K = PinholeIntrinsics{K[0], K[1], K[2], K[3]};
        }
      }
      if (C["distortion_coeffs"]) {
        const auto d = C["distortion_coeffs"].as<std::vector<double>>();
        if (d.size() >= 4) {
          cfg.dist = RadtanDistortion{d[0], d[1], d[2], d[3]};
        }
      }
      if (C["initial_T_CW"]) {
        cfg.initial_T_CW = SE3FromXyzRpy(C["initial_T_CW"].as<std::vector<double>>());
      }
      if (C["t_d_C_s"]) {
        cfg.initial_t_d_s = C["t_d_C_s"].as<double>();
      }
      if (C["prior_rot_std_deg"]) {
        cfg.prior_rot_std_deg = C["prior_rot_std_deg"].as<double>();
      }
      if (C["prior_trans_std_m"]) {
        cfg.prior_trans_std_m = C["prior_trans_std_m"].as<double>();
      }
      (*cameras)[cfg.id] = cfg;
    }
  }
}

struct ExtrinsicState {
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  Eigen::Vector3d t = Eigen::Vector3d::Zero();
  double t_d = 0.0;
  SE3d prior;
  Eigen::Matrix<double, 6, 1> prior_sqrt_info =
      PriorSqrtInfo(5.0, 0.5);

  SE3d AsSE3() const { return SE3d(q, t); }

  void SetFromSE3(const SE3d& T) {
    q = T.unit_quaternion();
    t = T.translation();
  }
};

Eigen::Vector3d InterpolateRtkPosition(const std::vector<RTKMeasurement>& rtk,
                                       double t_s) {
  if (rtk.empty()) {
    return Eigen::Vector3d::Zero();
  }
  if (t_s <= rtk.front().t_world_) {
    return rtk.front().p_A_W_observed_;
  }
  if (t_s >= rtk.back().t_world_) {
    return rtk.back().p_A_W_observed_;
  }
  for (size_t i = 1; i < rtk.size(); ++i) {
    if (t_s <= rtk[i].t_world_) {
      const double t0 = rtk[i - 1].t_world_;
      const double t1 = rtk[i].t_world_;
      const double u = (t_s - t0) / (t1 - t0);
      return (1.0 - u) * rtk[i - 1].p_A_W_observed_ +
             u * rtk[i].p_A_W_observed_;
    }
  }
  return rtk.back().p_A_W_observed_;
}

bool GetActiveKnotPointers(const BodyTrajectory& traj, int64_t t_ns,
                           std::array<double*, SplineOrder>* rot_knots,
                           std::array<double*, SplineOrder>* pos_knots) {
  if (traj.numKnots() < static_cast<size_t>(SplineOrder)) {
    return false;
  }
  SplineSegmentMeta<SplineOrder> meta(traj.minTimeNs(), traj.getDtNs(),
                                      traj.numKnots());
  if (t_ns < meta.MinTimeNs() || t_ns >= meta.MaxTimeNs()) {
    return false;
  }
  const auto ui = meta.computeTIndexNs(t_ns);
  const size_t s = ui.second;
  if (s + SplineOrder > traj.numKnots()) {
    return false;
  }
  for (int i = 0; i < SplineOrder; ++i) {
    (*rot_knots)[i] = const_cast<double*>(
        traj.getKnotSO3(static_cast<int>(s + i)).data());
    (*pos_knots)[i] = const_cast<double*>(
        traj.getKnotPos(static_cast<int>(s + i)).data());
  }
  return true;
}

void ExtendTrajectoryToCover(BodyTrajectory* traj, double t_start_s,
                             double t_end_s) {
  constexpr double kTimeOffsetMarginS = 1.0;
  const double t_hi = t_end_s + kTimeOffsetMarginS + 2.0 * traj->getDt();

  const int64_t t0_ns = static_cast<int64_t>(t_start_s * BodyTrajectory::kSToNs);
  const int64_t t1_ns = static_cast<int64_t>(t_hi * BodyTrajectory::kSToNs);
  if (traj->numKnots() == 0) {
    traj->extendKnotsTo(t0_ns, SO3d(Eigen::Quaterniond::Identity()),
                        Eigen::Vector3d::Zero());
  }
  traj->extendKnotsTo(t1_ns, traj->getKnotSO3(traj->numKnots() - 1),
                      traj->getKnotPos(traj->numKnots() - 1));
}

Eigen::Vector3d LeverArmToMarkerCorner(const LeverArmConfig& levers,
                                       const TargetGeometryConfig& target,
                                       int tag_id, int corner_idx) {
  Eigen::Vector3d L = levers.L_B_to_G;
  const auto it = levers.L_G_to_M.find(tag_id);
  if (it != levers.L_G_to_M.end()) {
    L += it->second;
  }
  const auto cit = target.corners_in_marker_frame.find(tag_id);
  if (cit != target.corners_in_marker_frame.end() && corner_idx >= 0 &&
      corner_idx < 4) {
    L += cit->second[static_cast<size_t>(corner_idx)];
  }
  return L;
}

std::pair<double, double> TimeOffsetBoundsForBarTimestamps(
    const std::vector<double>& bar_t_s, const BodyTrajectory& traj,
    double t_d_max_abs_s) {
  if (bar_t_s.empty()) {
    return {-t_d_max_abs_s, t_d_max_abs_s};
  }
  const double min_bar =
      *std::min_element(bar_t_s.begin(), bar_t_s.end());
  const double max_bar =
      *std::max_element(bar_t_s.begin(), bar_t_s.end());
  const double t_min = traj.minTimeNs() * NS_TO_S + 1e-3;
  const double t_max = traj.maxTimeNs() * NS_TO_S - 1e-3;
  double td_lower = max_bar - t_max;
  double td_upper = min_bar - t_min;
  td_lower = std::max(td_lower, -t_d_max_abs_s);
  td_upper = std::min(td_upper, t_d_max_abs_s);
  if (td_lower >= td_upper) {
    const double mid = 0.5 * (min_bar + max_bar) - 0.5 * (t_min + t_max);
    return {mid - 0.05, mid + 0.05};
  }
  return {td_lower, td_upper};
}

}  // namespace

struct CalibrationEstimator::Impl {
  std::string config_dir;
  LeverArmConfig levers;
  NoiseModel noise_model;
  SplineConfig spline;
  TargetGeometryConfig target;
  std::map<int, LidarRigConfig> lidar_cfg;
  std::map<int, CameraRigConfig> camera_cfg;

  std::map<int, ExtrinsicState> lidar_state;
  std::map<int, ExtrinsicState> camera_state;

  bool enable_extrinsic_prior_factors = true;

  std::vector<RTKMeasurement> rtk;
  std::map<int, std::vector<LiDARTargetObservation>> lidar_obs;
  std::map<int, std::vector<AprilTagObservation>> apriltag_obs;

  BodyTrajectory::Ptr trajectory;
  bool trajectory_rtk_seeded = false;
  bool rtk_warm_start_done = false;

  std::unique_ptr<ceres::Problem> problem;
  std::vector<std::unique_ptr<ceres::CostFunction>> owned_costs;
  std::vector<std::unique_ptr<ceres::LossFunction>> owned_losses;
  std::vector<std::unique_ptr<ceres::LocalParameterization>> owned_local_params;

  ceres::LocalParameterization* so3_local_param = nullptr;

  void ClearProblem() {
    if (problem) {
      problem.reset();
    }
    for (auto& cost : owned_costs) {
      cost.release();
    }
    owned_costs.clear();
    for (auto& loss : owned_losses) {
      loss.release();
    }
    owned_losses.clear();
    for (auto& local_param : owned_local_params) {
      local_param.release();
    }
    owned_local_params.clear();
    so3_local_param = nullptr;
  }

  SplineSegmentMeta<SplineOrder> TrajectoryMeta() const {
    return SplineSegmentMeta<SplineOrder>(trajectory->minTimeNs(),
                                          trajectory->getDtNs(),
                                          trajectory->numKnots());
  }

  void EnsureSo3LocalParam() {
    if (!so3_local_param) {
      owned_local_params.push_back(
          std::make_unique<LieLocalParameterization<SO3d>>());
      so3_local_param = owned_local_params.back().get();
    }
  }

  void SetLocalParamSO3(ceres::Problem* prob, double* q) {
    EnsureSo3LocalParam();
    if (prob->HasParameterBlock(q)) {
      prob->SetParameterization(q, so3_local_param);
    }
  }

  void AddRtkFactors(ceres::Problem* prob) {
    const SplineSegmentMeta<SplineOrder> meta = TrajectoryMeta();
    for (const auto& m : rtk) {
      if (m.fix_status_ != RTKMeasurement::FixStatus::FIXED) {
        continue;
      }
      const int64_t t_ns = static_cast<int64_t>(m.t_world_ * S_TO_NS);
      std::array<double*, SplineOrder> rot_knots{};
      std::array<double*, SplineOrder> pos_knots{};
      if (!GetActiveKnotPointers(*trajectory, t_ns, &rot_knots, &pos_knots)) {
        continue;
      }
      owned_costs.push_back(std::make_unique<analytic_derivative::RTKPositionFactor>(
          t_ns, m.p_A_W_observed_, m.covariance_, levers.L_B_to_A, meta));
      auto* factor = owned_costs.back().get();
      std::vector<double*> blocks;
      blocks.insert(blocks.end(), rot_knots.begin(), rot_knots.end());
      blocks.insert(blocks.end(), pos_knots.begin(), pos_knots.end());
      prob->AddResidualBlock(factor, nullptr, blocks);
      for (double* q : rot_knots) {
        SetLocalParamSO3(prob, q);
      }
    }
  }

  void AddSmoothnessFactors(ceres::Problem* prob) {
    const SplineSegmentMeta<SplineOrder> meta = TrajectoryMeta();
    const double dt_s = trajectory->getDt();
    if (trajectory->numKnots() < static_cast<size_t>(SplineOrder + 1)) {
      return;
    }
    for (size_t seg = 0; seg + SplineOrder <= trajectory->numKnots(); ++seg) {
      const int64_t t_mid_ns =
          meta.t0_ns + static_cast<int64_t>((static_cast<double>(seg) + 0.5) *
                                            meta.dt_ns);
      if (t_mid_ns < meta.MinTimeNs() || t_mid_ns >= meta.MaxTimeNs()) {
        continue;
      }
      std::array<double*, SplineOrder> rot_knots{};
      std::array<double*, SplineOrder> pos_knots{};
      if (!GetActiveKnotPointers(*trajectory, t_mid_ns, &rot_knots, &pos_knots)) {
        continue;
      }
      owned_costs.push_back(
          std::make_unique<analytic_derivative::TrajectorySmoothnessFactor>(
              t_mid_ns, spline.alpha_p, spline.alpha_R, dt_s, meta));
      auto* factor = owned_costs.back().get();
      std::vector<double*> blocks;
      blocks.insert(blocks.end(), rot_knots.begin(), rot_knots.end());
      blocks.insert(blocks.end(), pos_knots.begin(), pos_knots.end());
      prob->AddResidualBlock(factor, nullptr, blocks);
      for (double* q : rot_knots) {
        SetLocalParamSO3(prob, q);
      }
    }
  }

  void AddLidarFactors(ceres::Problem* prob) {
    const SplineSegmentMeta<SplineOrder> meta = TrajectoryMeta();
    owned_losses.push_back(
        std::make_unique<ceres::CauchyLoss>(spline.lidar_cauchy_scale));
    ceres::LossFunction* cauchy = owned_losses.back().get();

    for (const auto& kv : lidar_obs) {
      const int sensor_id = kv.first;
      auto sit = lidar_state.find(sensor_id);
      if (sit == lidar_state.end()) {
        continue;
      }
      ExtrinsicState& state = sit->second;
      const double sigma_r = noise_model.lidar_ranging_sigma_m;

      std::vector<double> bar_times;
      bar_times.reserve(kv.second.size());
      for (const auto& scan : kv.second) {
        bar_times.push_back(scan.t_sensor_);
      }

      for (const auto& scan : kv.second) {
        const int64_t bar_t_ns =
            static_cast<int64_t>(scan.t_sensor_ * S_TO_NS);
        for (const auto& q_L : scan.points_L_) {
          const int64_t t_eval_ns =
              bar_t_ns - static_cast<int64_t>(state.t_d * S_TO_NS);
          std::array<double*, SplineOrder> rot_knots{};
          std::array<double*, SplineOrder> pos_knots{};
          if (!GetActiveKnotPointers(*trajectory, t_eval_ns, &rot_knots,
                                     &pos_knots)) {
            continue;
          }
          owned_costs.push_back(
              std::make_unique<analytic_derivative::SphereImplicitFactor>(
                  bar_t_ns, q_L, levers.L_B_to_G, target.sphere_radius_m,
                  sigma_r, meta));
          auto* factor = owned_costs.back().get();
          std::vector<double*> blocks = {&state.t_d, state.q.coeffs().data(),
                                         state.t.data()};
          blocks.insert(blocks.end(), rot_knots.begin(), rot_knots.end());
          blocks.insert(blocks.end(), pos_knots.begin(), pos_knots.end());
          prob->AddResidualBlock(factor, cauchy, blocks);
          SetLocalParamSO3(prob, state.q.coeffs().data());
          for (double* q : rot_knots) {
            SetLocalParamSO3(prob, q);
          }
        }
      }

      const auto td_bounds = TimeOffsetBoundsForBarTimestamps(
          bar_times, *trajectory, spline.t_d_max_abs_s);
      if (prob->HasParameterBlock(&state.t_d)) {
        prob->SetParameterLowerBound(&state.t_d, 0, td_bounds.first);
        prob->SetParameterUpperBound(&state.t_d, 0, td_bounds.second);
      }

      owned_costs.push_back(
          std::make_unique<analytic_derivative::ExtrinsicPriorFactor>(
              state.prior, state.prior_sqrt_info));
      if (enable_extrinsic_prior_factors) {
        prob->AddResidualBlock(owned_costs.back().get(), nullptr,
                               state.q.coeffs().data(), state.t.data());
      }
      SetLocalParamSO3(prob, state.q.coeffs().data());
    }
  }

  void AddCameraFactors(ceres::Problem* prob) {
    const SplineSegmentMeta<SplineOrder> meta = TrajectoryMeta();
    owned_losses.push_back(
        std::make_unique<ceres::HuberLoss>(spline.camera_huber_delta_px));
    ceres::LossFunction* huber = owned_losses.back().get();

    for (const auto& kv : apriltag_obs) {
      const int sensor_id = kv.first;
      auto sit = camera_state.find(sensor_id);
      if (sit == camera_state.end()) {
        continue;
      }
      ExtrinsicState& state = sit->second;
      const CameraRigConfig& cam_cfg = camera_cfg.at(sensor_id);

      std::vector<double> bar_times;
      bar_times.reserve(kv.second.size());
      for (const auto& det : kv.second) {
        bar_times.push_back(det.t_sensor_);
      }

      for (const auto& det : kv.second) {
        for (int corner = 0; corner < 4; ++corner) {
          const Eigen::Vector3d L_B_to_G_M = LeverArmToMarkerCorner(
              levers, target, det.tag_id_, corner);
          const int64_t bar_t_ns =
              static_cast<int64_t>(det.t_sensor_ * S_TO_NS);
          const int64_t t_eval_ns =
              bar_t_ns - static_cast<int64_t>(state.t_d * S_TO_NS);
          std::array<double*, SplineOrder> rot_knots{};
          std::array<double*, SplineOrder> pos_knots{};
          if (!GetActiveKnotPointers(*trajectory, t_eval_ns, &rot_knots,
                                     &pos_knots)) {
            continue;
          }
          owned_costs.push_back(
              std::make_unique<analytic_derivative::AprilTagReprojFactor>(
                  bar_t_ns, det.corners_pixel_[corner], L_B_to_G_M, cam_cfg.K,
                  cam_cfg.dist, noise_model.camera_pixel_sigma, meta));
          auto* factor = owned_costs.back().get();
          std::vector<double*> blocks = {&state.t_d, state.q.coeffs().data(),
                                         state.t.data()};
          blocks.insert(blocks.end(), rot_knots.begin(), rot_knots.end());
          blocks.insert(blocks.end(), pos_knots.begin(), pos_knots.end());
          prob->AddResidualBlock(factor, huber, blocks);
          SetLocalParamSO3(prob, state.q.coeffs().data());
          for (double* q : rot_knots) {
            SetLocalParamSO3(prob, q);
          }
        }
      }

      const auto td_bounds = TimeOffsetBoundsForBarTimestamps(
          bar_times, *trajectory, spline.t_d_max_abs_s);
      if (prob->HasParameterBlock(&state.t_d)) {
        prob->SetParameterLowerBound(&state.t_d, 0, td_bounds.first);
        prob->SetParameterUpperBound(&state.t_d, 0, td_bounds.second);
      }

      owned_costs.push_back(
          std::make_unique<analytic_derivative::ExtrinsicPriorFactor>(
              state.prior, state.prior_sqrt_info));
      if (enable_extrinsic_prior_factors) {
        prob->AddResidualBlock(owned_costs.back().get(), nullptr,
                               state.q.coeffs().data(), state.t.data());
      }
      SetLocalParamSO3(prob, state.q.coeffs().data());
    }
  }

  void BuildProblem(bool rtk_only) {
    ClearProblem();
    problem = std::make_unique<ceres::Problem>();
    AddRtkFactors(problem.get());
    AddSmoothnessFactors(problem.get());
    if (!rtk_only) {
      AddLidarFactors(problem.get());
      AddCameraFactors(problem.get());
    }
  }

  ceres::Solver::Summary RunSolver(int max_iters) {
    ceres::Solver::Options opts;
    opts.max_num_iterations = max_iters;
    opts.minimizer_type = ceres::TRUST_REGION;
    opts.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    opts.function_tolerance = 1e-8;
    opts.gradient_tolerance = 1e-10;
    opts.minimizer_progress_to_stdout = false;

    ceres::Solver::Summary summary;
    if (problem && problem->NumResidualBlocks() > 0) {
      ceres::Solve(opts, problem.get(), &summary);
    }
    return summary;
  }

  void SeedTrajectoryKnotsFromRtk() {
    if (rtk.empty()) {
      throw std::runtime_error("SeedTrajectoryKnotsFromRtk: no RTK data");
    }

    const double t0 = rtk.front().t_world_;
    const double t1 = rtk.back().t_world_;
    trajectory = std::make_shared<BodyTrajectory>(spline.knot_interval_s, t0);
    ExtendTrajectoryToCover(trajectory.get(), t0, t1);

    for (size_t i = 0; i < trajectory->numKnots(); ++i) {
      const double t_k =
          trajectory->minTimeNs() * NS_TO_S +
          static_cast<double>(i) * trajectory->getDt();
      const Eigen::Vector3d p_A = InterpolateRtkPosition(rtk, t_k);
      const Eigen::Vector3d p_WB = p_A - levers.L_B_to_A;
      trajectory->setKnotPos(p_WB, static_cast<int>(i));
      trajectory->setKnotSO3(SO3d(Eigen::Quaterniond::Identity()),
                             static_cast<int>(i));
    }
    trajectory_rtk_seeded = true;
  }

  void RunRtkWarmStart() {
    if (rtk_warm_start_done) {
      return;
    }
    if (!trajectory_rtk_seeded) {
      SeedTrajectoryKnotsFromRtk();
    }
    BuildProblem(true);
    const ceres::Solver::Summary summary = RunSolver(100);
    if (!summary.IsSolutionUsable()) {
      std::cerr << "[CalibrationEstimator] RTK warm-start failed:\n"
                << summary.FullReport() << std::endl;
    }
    rtk_warm_start_done = true;
  }

  ~Impl() { ClearProblem(); }
};

CalibrationEstimator::CalibrationEstimator(const std::string& config_path)
    : impl_(std::make_unique<Impl>()) {
  impl_->config_dir = config_path;
  if (!impl_->config_dir.empty() && impl_->config_dir.back() == '/') {
    impl_->config_dir.pop_back();
  }

  const std::string prefix = impl_->config_dir + "/";
  impl_->levers = LeverArmConfig::from_yaml(prefix + "lever_arms.yaml");
  impl_->noise_model = NoiseModel::FromConfigDir(impl_->config_dir);
  impl_->noise_model.Log();
  impl_->spline = LoadSplineConfig(prefix + "spline.yaml");
  impl_->target = LoadTargetGeometry(prefix + "target_geometry.yaml");
  LoadSensorRig(prefix + "sensor_rig.yaml", &impl_->lidar_cfg, &impl_->camera_cfg);

  impl_->trajectory = std::make_shared<BodyTrajectory>(impl_->spline.knot_interval_s,
                                                       0.0);

  for (const auto& kv : impl_->lidar_cfg) {
    ExtrinsicState state;
    state.SetFromSE3(kv.second.initial_T_LW);
    state.t_d = kv.second.initial_t_d_s;
    state.prior = kv.second.initial_T_LW;
    state.prior_sqrt_info =
        PriorSqrtInfo(kv.second.prior_rot_std_deg, kv.second.prior_trans_std_m);
    impl_->lidar_state[kv.first] = state;
  }
  for (const auto& kv : impl_->camera_cfg) {
    ExtrinsicState state;
    state.SetFromSE3(kv.second.initial_T_CW);
    state.t_d = kv.second.initial_t_d_s;
    state.prior = kv.second.initial_T_CW;
    state.prior_sqrt_info =
        PriorSqrtInfo(kv.second.prior_rot_std_deg, kv.second.prior_trans_std_m);
    impl_->camera_state[kv.first] = state;
  }
}

CalibrationEstimator::~CalibrationEstimator() = default;

void CalibrationEstimator::add_rtk_measurements(
    const std::vector<RTKMeasurement>& rtk) {
  impl_->rtk.insert(impl_->rtk.end(), rtk.begin(), rtk.end());
  std::sort(impl_->rtk.begin(), impl_->rtk.end(),
            [](const RTKMeasurement& a, const RTKMeasurement& b) {
              return a.t_world_ < b.t_world_;
            });
}

void CalibrationEstimator::add_lidar_target_observations(
    int sensor_id, const std::vector<LiDARTargetObservation>& obs) {
  auto& dst = impl_->lidar_obs[sensor_id];
  dst.insert(dst.end(), obs.begin(), obs.end());
}

void CalibrationEstimator::add_apriltag_observations(
    int sensor_id, const std::vector<AprilTagObservation>& obs) {
  auto& dst = impl_->apriltag_obs[sensor_id];
  dst.insert(dst.end(), obs.begin(), obs.end());
}

void CalibrationEstimator::set_initial_extrinsic_T_LW(int sensor_id,
                                                        const SE3d& T_LW) {
  impl_->lidar_state[sensor_id].SetFromSE3(T_LW);
}

void CalibrationEstimator::set_initial_extrinsic_T_CW(int sensor_id,
                                                        const SE3d& T_CW) {
  impl_->camera_state[sensor_id].SetFromSE3(T_CW);
}

void CalibrationEstimator::set_extrinsic_prior_T_LW(int sensor_id,
                                                      const SE3d& T_LW_prior) {
  impl_->lidar_state[sensor_id].prior = T_LW_prior;
}

void CalibrationEstimator::set_extrinsic_prior_T_CW(int sensor_id,
                                                      const SE3d& T_CW_prior) {
  impl_->camera_state[sensor_id].prior = T_CW_prior;
}

void CalibrationEstimator::set_extrinsic_prior_std(double rot_std_deg,
                                                     double trans_std_m) {
  const Eigen::Matrix<double, 6, 1> info =
      PriorSqrtInfo(rot_std_deg, trans_std_m);
  for (auto& kv : impl_->lidar_state) {
    kv.second.prior_sqrt_info = info;
  }
  for (auto& kv : impl_->camera_state) {
    kv.second.prior_sqrt_info = info;
  }
}

void CalibrationEstimator::set_extrinsic_prior_enabled(bool enabled) {
  impl_->enable_extrinsic_prior_factors = enabled;
}

void CalibrationEstimator::initialize_trajectory_from_rtk() {
  impl_->RunRtkWarmStart();
}

ceres::Solver::Summary CalibrationEstimator::solve(int max_iters) {
  if (impl_->rtk.empty()) {
    throw std::runtime_error("solve: no RTK measurements");
  }
  impl_->RunRtkWarmStart();

  const double t0 = impl_->rtk.front().t_world_;
  double t_end = impl_->rtk.back().t_world_;
  for (const auto& kv : impl_->lidar_obs) {
    for (const auto& scan : kv.second) {
      t_end = std::max(t_end, scan.t_sensor_);
    }
  }
  for (const auto& kv : impl_->apriltag_obs) {
    for (const auto& det : kv.second) {
      t_end = std::max(t_end, det.t_sensor_);
    }
  }
  ExtendTrajectoryToCover(impl_->trajectory.get(), t0, t_end);

  impl_->BuildProblem(false);
  ceres::Solver::Summary summary = impl_->RunSolver(max_iters);
  if (!summary.IsSolutionUsable()) {
    std::cerr << "[CalibrationEstimator] solve failed:\n"
              << summary.FullReport() << std::endl;
  }
  return summary;
}

SE3d CalibrationEstimator::get_T_LW(int sensor_id) const {
  return impl_->lidar_state.at(sensor_id).AsSE3();
}

SE3d CalibrationEstimator::get_T_CW(int sensor_id) const {
  return impl_->camera_state.at(sensor_id).AsSE3();
}

double CalibrationEstimator::get_t_d_lidar(int sensor_id) const {
  return impl_->lidar_state.at(sensor_id).t_d;
}

double CalibrationEstimator::get_t_d_camera(int sensor_id) const {
  return impl_->camera_state.at(sensor_id).t_d;
}

std::shared_ptr<Trajectory> CalibrationEstimator::get_trajectory() const {
  return impl_->trajectory;
}

void CalibrationEstimator::build_problem_for_analysis() {
  if (impl_->rtk.empty()) {
    throw std::runtime_error("build_problem_for_analysis: no RTK measurements");
  }
  impl_->RunRtkWarmStart();

  const double t0 = impl_->rtk.front().t_world_;
  double t_end = impl_->rtk.back().t_world_;
  for (const auto& kv : impl_->lidar_obs) {
    for (const auto& scan : kv.second) {
      t_end = std::max(t_end, scan.t_sensor_);
    }
  }
  for (const auto& kv : impl_->apriltag_obs) {
    for (const auto& det : kv.second) {
      t_end = std::max(t_end, det.t_sensor_);
    }
  }
  ExtendTrajectoryToCover(impl_->trajectory.get(), t0, t_end);
  impl_->BuildProblem(false);
}

const ceres::Problem& CalibrationEstimator::problem() const {
  if (!impl_->problem) {
    throw std::runtime_error(
        "problem() called before solve() or build_problem_for_analysis()");
  }
  return *impl_->problem;
}

ceres::Problem& CalibrationEstimator::problem() {
  if (!impl_->problem) {
    throw std::runtime_error(
        "problem() called before solve() or build_problem_for_analysis()");
  }
  return *impl_->problem;
}

AnalysisParameterLayout CalibrationEstimator::analysis_parameter_layout() const {
  if (!impl_->problem) {
    throw std::runtime_error(
        "analysis_parameter_layout: problem not built");
  }
  const ceres::Problem& prob = *impl_->problem;

  auto is_extrinsic_ptr = [&](const double* ptr_const) -> bool {
    double* ptr = const_cast<double*>(ptr_const);
    for (const auto& kv : impl_->lidar_state) {
      if (ptr == kv.second.q.coeffs().data() || ptr == kv.second.t.data()) {
        return true;
      }
    }
    for (const auto& kv : impl_->camera_state) {
      if (ptr == kv.second.q.coeffs().data() || ptr == kv.second.t.data()) {
        return true;
      }
    }
    return false;
  };

  std::vector<double*> blocks;
  prob.GetParameterBlocks(&blocks);

  AnalysisParameterLayout layout;
  int cursor = 0;
  int fext_idx = 0;
  for (size_t bi = 0; bi < blocks.size(); ++bi) {
    double* ptr = blocks[bi];
    if (ptr == nullptr) {
      throw std::runtime_error("analysis_parameter_layout: null parameter block");
    }
    const int local_size = prob.ParameterBlockLocalSize(ptr);
    const bool extrinsic = is_extrinsic_ptr(ptr);
    bool is_lidar_q = false;
    bool is_lidar_t = false;
    if (!impl_->lidar_state.empty()) {
      const auto& lidar0 = impl_->lidar_state.begin()->second;
      is_lidar_q = (ptr == lidar0.q.coeffs().data());
      is_lidar_t = (ptr == lidar0.t.data());
    }
    for (int k = 0; k < local_size; ++k) {
      if (extrinsic) {
        layout.extrinsic_local_indices.push_back(cursor + k);
        if (is_lidar_q) {
          layout.lidar_rot_local_indices.push_back(cursor + k);
          layout.lidar_rot_fext_indices.push_back(fext_idx++);
        } else if (is_lidar_t) {
          layout.lidar_trans_local_indices.push_back(cursor + k);
          layout.lidar_trans_fext_indices.push_back(fext_idx++);
        } else {
          ++fext_idx;
        }
      } else {
        layout.rest_local_indices.push_back(cursor + k);
      }
    }
    cursor += local_size;
  }
  layout.num_local_parameters = cursor;
  return layout;
}

}  // namespace clic_calib
