#pragma once

#include <clic_calib/factor/apriltag_reproj_factor.h>
#include <clic_calib/factor/ceres_local_param.h>
#include <clic_calib/sensor_data/attitude_observation.h>
#include <clic_calib/factor/attitude_factor.h>
#include <clic_calib/factor/rtk_position_factor.h>
#include <clic_calib/factor/sphere_implicit_factor.h>
#include <clic_calib/factor/trajectory_smoothness_factor.h>
#include <clic_calib/sensor_data/apriltag_observation.h>
#include <clic_calib/sensor_data/lidar_target_observation.h>
#include <clic_calib/sensor_data/rtk_measurement.h>
#include <clic_calib/spline/trajectory.h>
#include <clic_calib/utils/camera_projection.h>
#include <clic_calib/utils/lever_arm.h>
#include <clic_calib/utils/noise_model.h>
#include <clic_calib/utils/sophus_utils.hpp>

#include <ceres/ceres.h>
#include <ceres/crs_matrix.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <random>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace clic_calib {
namespace two_stage_probe {

inline SE3d SE3FromXyzRpy(const std::vector<double>& v) {
  if (v.size() != 6) {
    throw std::runtime_error("initial_T_* expects 6 values [x,y,z,r,p,y]");
  }
  const Eigen::Vector3d t(v[0], v[1], v[2]);
  const SO3d R = SO3d::rotZ(v[5]) * SO3d::rotY(v[4]) * SO3d::rotX(v[3]);
  return SE3d(R, t);
}

struct SplineConfig {
  double knot_interval_s = 0.05;
  double alpha_p = 0.01;
  double alpha_R = 0.01;
  double t_d_max_abs_s = 0.1;
};

inline SplineConfig LoadSplineConfig(const std::string& path) {
  SplineConfig cfg;
  const YAML::Node node = YAML::LoadFile(path);
  if (node["knot_interval_s"]) {
    cfg.knot_interval_s = node["knot_interval_s"].as<double>();
  }
  if (node["alpha_p"]) {
    cfg.alpha_p = node["alpha_p"].as<double>();
  }
  if (node["alpha_R"]) {
    cfg.alpha_R = node["alpha_R"].as<double>();
  }
  if (node["t_d_max_abs_s"]) {
    cfg.t_d_max_abs_s = node["t_d_max_abs_s"].as<double>();
  }
  return cfg;
}

struct CoarseExtrinsicInit {
  SE3d T_LW;
  SE3d T_CW;
  double t_d_L_s = 0.03;
  double t_d_C_s = -0.015;
};

inline CoarseExtrinsicInit LoadCoarseExtrinsicsFromYaml(
    const std::string& config_dir) {
  CoarseExtrinsicInit out;
  const YAML::Node node = YAML::LoadFile(config_dir + "/sensor_rig.yaml");
  if (node["lidars"] && node["lidars"].IsSequence() &&
      node["lidars"].size() > 0) {
    const YAML::Node L = node["lidars"][0];
    if (L["initial_T_LW"]) {
      out.T_LW = SE3FromXyzRpy(L["initial_T_LW"].as<std::vector<double>>());
    }
    if (L["t_d_L_s"]) {
      out.t_d_L_s = L["t_d_L_s"].as<double>();
    }
  }
  if (node["cameras"] && node["cameras"].IsSequence() &&
      node["cameras"].size() > 0) {
    const YAML::Node C = node["cameras"][0];
    if (C["initial_T_CW"]) {
      out.T_CW = SE3FromXyzRpy(C["initial_T_CW"].as<std::vector<double>>());
    }
    if (C["t_d_C_s"]) {
      out.t_d_C_s = C["t_d_C_s"].as<double>();
    }
  }
  return out;
}

/** Near-field probe: coarse init = GT perturbed ~1 m / ~10° (not legacy yaml). */
inline CoarseExtrinsicInit MakeCoarseExtrinsicInitPerturbed(
    const SE3d& T_LW_gt, const SE3d& T_CW_gt, double t_d_L_gt, double t_d_C_gt,
    uint32_t seed) {
  CoarseExtrinsicInit out;
  std::mt19937 rng(seed + 9001u);
  std::uniform_real_distribution<double> u01(0.0, 1.0);
  auto sample_signed = [&](double mag) {
    return (u01(rng) * 2.0 - 1.0) * mag;
  };
  const Eigen::Vector3d omega_L(sample_signed(10.0 * M_PI / 180.0),
                                sample_signed(10.0 * M_PI / 180.0),
                                sample_signed(10.0 * M_PI / 180.0));
  const Eigen::Vector3d omega_C(sample_signed(10.0 * M_PI / 180.0),
                                sample_signed(10.0 * M_PI / 180.0),
                                sample_signed(10.0 * M_PI / 180.0));
  const Eigen::Vector3d dt_L(sample_signed(1.0), sample_signed(1.0),
                             sample_signed(1.0));
  const Eigen::Vector3d dt_C(sample_signed(1.0), sample_signed(1.0),
                             sample_signed(1.0));
  out.T_LW = SE3d(T_LW_gt.so3() * SO3d::exp(omega_L), T_LW_gt.translation() + dt_L);
  out.T_CW = SE3d(T_CW_gt.so3() * SO3d::exp(omega_C), T_CW_gt.translation() + dt_C);
  out.t_d_L_s = t_d_L_gt + sample_signed(0.005);
  out.t_d_C_s = t_d_C_gt + sample_signed(0.005);
  return out;
}

inline double MinSpacingInSortedTimes(const std::vector<double>& times) {
  if (times.size() < 2) {
    return 0.0;
  }
  std::vector<double> sorted = times;
  std::sort(sorted.begin(), sorted.end());
  sorted.erase(std::unique(sorted.begin(), sorted.end(),
                           [](double a, double b) {
                             return std::abs(a - b) < 1e-9;
                           }),
              sorted.end());
  double min_dt = std::numeric_limits<double>::infinity();
  for (size_t i = 1; i < sorted.size(); ++i) {
    const double dt = sorted[i] - sorted[i - 1];
    if (dt > 1e-9) {
      min_dt = std::min(min_dt, dt);
    }
  }
  return std::isfinite(min_dt) ? min_dt : 0.0;
}

inline std::shared_ptr<BodyTrajectory> TrimTrajectoryToObservedSupport(
    const BodyTrajectory& reference, double t_obs_lo, double t_obs_hi,
    double knot_dt) {
  const double ref_t0 = reference.minTimeNs() * NS_TO_S;
  const double ref_t1 = reference.maxTimeNs() * NS_TO_S;
  const double margin = static_cast<double>(SplineOrder - 1) * knot_dt;
  const double t_lo = std::max(ref_t0, t_obs_lo - margin);
  const double t_hi = std::min(ref_t1, t_obs_hi + margin);
  if (t_hi <= t_lo + knot_dt) {
    throw std::runtime_error("TrimTrajectoryToObservedSupport: invalid span");
  }
  const int num_knots =
      static_cast<int>(std::ceil((t_hi - t_lo) / knot_dt)) + (SplineOrder - 1);
  auto traj = std::make_shared<BodyTrajectory>(knot_dt, t_lo);
  traj->setKnots(reference.pose_wb(t_lo), num_knots);
  for (int i = 0; i < num_knots; ++i) {
    const double t = t_lo + static_cast<double>(i) * knot_dt;
    const double t_sample = std::min(std::max(t, ref_t0), ref_t1 - 1e-9);
    traj->setKnot(reference.pose_wb(t_sample), i);
  }
  return traj;
}

inline bool GetActiveKnotPointers(const BodyTrajectory& traj, int64_t t_ns,
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

/** Ceres scope matching CalibrationEstimator: shared SO3 local param, no ownership transfer. */
struct CeresSo3ProblemScope {
  ceres::Problem::Options opts;
  std::unique_ptr<ceres::Problem> problem;
  std::vector<std::unique_ptr<ceres::CostFunction>> owned_costs;
  std::unique_ptr<LieLocalParameterization<SO3d>> so3_local;

  CeresSo3ProblemScope() {
    opts.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    opts.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    opts.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    problem = std::make_unique<ceres::Problem>(opts);
    so3_local = std::make_unique<LieLocalParameterization<SO3d>>();
  }

  void SetLocalParamSO3(double* q) {
    if (problem->HasParameterBlock(q)) {
      problem->SetParameterization(q, so3_local.get());
    }
  }
};

inline Eigen::Vector3d InterpolateRtkPosition(
    const std::vector<RTKMeasurement>& rtk, double t_s) {
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

inline void ReseedKnotPositionsFromRtkLeverArm(
    BodyTrajectory* traj, const std::vector<RTKMeasurement>& rtk,
    const Eigen::Vector3d& L_B_to_A) {
  if (!traj || rtk.empty()) {
    return;
  }
  for (size_t i = 0; i < traj->numKnots(); ++i) {
    const double t_k =
        traj->minTimeNs() * NS_TO_S + static_cast<double>(i) * traj->getDt();
    const Eigen::Vector3d p_A = InterpolateRtkPosition(rtk, t_k);
    const SO3d R_wb = traj->getKnotSO3(static_cast<int>(i));
    traj->setKnotPos(p_A - R_wb * L_B_to_A, static_cast<int>(i));
  }
}

inline double RtkAntennaRmsMm(const BodyTrajectory& traj,
                                const std::vector<RTKMeasurement>& rtk,
                                const Eigen::Vector3d& L_B_to_A) {
  double sq = 0.0;
  int n = 0;
  for (const auto& m : rtk) {
    if (m.fix_status_ != RTKMeasurement::FixStatus::FIXED) {
      continue;
    }
    const Eigen::Vector3d p_pred =
        traj.antenna_position_w(m.t_world_, L_B_to_A);
    sq += (p_pred - m.p_A_W_observed_).squaredNorm();
    ++n;
  }
  return std::sqrt(sq / std::max(n, 1)) * 1e3;
}

inline Eigen::Vector3d AntennaVelocityFromRtk(
    const std::vector<RTKMeasurement>& rtk, double t_s, double dt_s) {
  const double t0 = rtk.front().t_world_;
  const double t1 = rtk.back().t_world_;
  if (t_s - dt_s < t0) {
    return (InterpolateRtkPosition(rtk, std::min(t_s + dt_s, t1)) -
            InterpolateRtkPosition(rtk, t0)) /
           std::max(std::min(t_s + dt_s, t1) - t0, 1e-6);
  }
  if (t_s + dt_s > t1) {
    return (InterpolateRtkPosition(rtk, t1) -
            InterpolateRtkPosition(rtk, std::max(t_s - dt_s, t0))) /
           std::max(t1 - std::max(t_s - dt_s, t0), 1e-6);
  }
  return (InterpolateRtkPosition(rtk, t_s + dt_s) -
          InterpolateRtkPosition(rtk, t_s - dt_s)) /
         (2.0 * dt_s);
}

/** FRD body in ENU: x ∥ horizontal velocity, z down. Coarse yaw, not GT. */
inline SO3d RotationFrdFromWorldVelocity(const Eigen::Vector3d& v_W) {
  Eigen::Vector3d x = v_W;
  x.z() = 0.0;
  if (x.norm() < 1e-4) {
    return SO3d(Eigen::Quaterniond::Identity());
  }
  x.normalize();
  const Eigen::Vector3d z_body_in_W(0.0, 0.0, -1.0);
  Eigen::Vector3d y = z_body_in_W.cross(x);
  if (y.norm() < 1e-6) {
    return SO3d(Eigen::Quaterniond::Identity());
  }
  y.normalize();
  x = y.cross(z_body_in_W).normalized();
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  R.col(0) = x;
  R.col(1) = y;
  R.col(2) = z_body_in_W;
  return SO3d(R);
}

/** Interpolate noisy PSDK attitude (not GT) onto knot times. */
inline SO3d InterpolateAttitudeObservation(
    const std::vector<AttitudeObservation>& attitude, double t) {
  if (attitude.empty()) {
    return SO3d(Eigen::Quaterniond::Identity());
  }
  if (t <= attitude.front().t_world_) {
    return attitude.front().R_WB_observed_;
  }
  if (t >= attitude.back().t_world_) {
    return attitude.back().R_WB_observed_;
  }
  for (size_t i = 1; i < attitude.size(); ++i) {
    if (t <= attitude[i].t_world_) {
      const double t0 = attitude[i - 1].t_world_;
      const double t1 = attitude[i].t_world_;
      const double u = (t - t0) / std::max(t1 - t0, 1e-9);
      const SO3d dR = attitude[i - 1].R_WB_observed_.inverse() *
                      attitude[i].R_WB_observed_;
      return attitude[i - 1].R_WB_observed_ * SO3d::exp(u * dR.log());
    }
  }
  return attitude.back().R_WB_observed_;
}

enum class AttitudeInitMode {
  kIdentity,
  kRtkVelocityFrd,
  kAttitudeStream,
};

inline std::shared_ptr<BodyTrajectory> InitStage1TrajectoryNoGtAttitude(
    double t_obs_lo, double t_obs_hi, double knot_dt,
    const std::vector<RTKMeasurement>& rtk,
    const Eigen::Vector3d& L_B_to_A,
    AttitudeInitMode att_init = AttitudeInitMode::kAttitudeStream,
    const std::vector<AttitudeObservation>* attitude = nullptr) {
  const double margin = static_cast<double>(SplineOrder - 1) * knot_dt;
  const double t_lo = std::max(0.0, t_obs_lo - margin);
  const double t_hi = t_obs_hi + margin;
  if (t_hi <= t_lo + knot_dt) {
    throw std::runtime_error("InitStage1TrajectoryNoGtAttitude: invalid span");
  }
  const int num_knots =
      static_cast<int>(std::ceil((t_hi - t_lo) / knot_dt)) + (SplineOrder - 1);
  auto traj = std::make_shared<BodyTrajectory>(knot_dt, t_lo);
  const SE3d k0(SO3d(Eigen::Quaterniond::Identity()), Eigen::Vector3d::Zero());
  traj->setKnots(k0, num_knots);
  const double vel_dt = std::max(0.5 * knot_dt, 0.02);
  for (int i = 0; i < num_knots; ++i) {
    const double t_k = t_lo + static_cast<double>(i) * knot_dt;
    const Eigen::Vector3d p_A = InterpolateRtkPosition(rtk, t_k);
    SO3d R_init = SO3d(Eigen::Quaterniond::Identity());
    switch (att_init) {
      case AttitudeInitMode::kIdentity:
        break;
      case AttitudeInitMode::kRtkVelocityFrd:
        if (!rtk.empty()) {
          R_init = RotationFrdFromWorldVelocity(
              AntennaVelocityFromRtk(rtk, t_k, vel_dt));
        }
        break;
      case AttitudeInitMode::kAttitudeStream:
        if (attitude != nullptr && !attitude->empty()) {
          R_init = InterpolateAttitudeObservation(*attitude, t_k);
        }
        break;
    }
    traj->setKnot(SE3d(R_init, p_A - R_init * L_B_to_A), i);
  }
  return traj;
}

inline double Stage1KnotDt(const std::vector<RTKMeasurement>& rtk,
                             const std::vector<AttitudeObservation>& attitude,
                             double knot_interval_s) {
  std::vector<double> times;
  times.reserve(rtk.size() + attitude.size());
  for (const auto& m : rtk) {
    times.push_back(m.t_world_);
  }
  for (const auto& a : attitude) {
    times.push_back(a.t_world_);
  }
  const double min_dt = MinSpacingInSortedTimes(times);
  return std::max(knot_interval_s, min_dt);
}

struct EigenReport {
  double lambda_min = 0.0;
  double lambda_max = 0.0;
  double cond = std::numeric_limits<double>::infinity();
  int rank = 0;
  int dim = 0;
  bool is_pd = false;
};

inline EigenReport AnalyzeSymmetricF(const Eigen::MatrixXd& F) {
  EigenReport r;
  r.dim = static_cast<int>(F.rows());
  if (F.rows() == 0) {
    return r;
  }
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(F);
  const Eigen::VectorXd evals = es.eigenvalues();
  r.lambda_min = evals.minCoeff();
  r.lambda_max = evals.maxCoeff();
  r.is_pd = r.lambda_min > 0.0;
  if (r.is_pd) {
    r.cond = r.lambda_max / r.lambda_min;
  }
  const double tol =
      std::max(1e-12, 1e-10 * std::max(std::abs(r.lambda_max), 1.0));
  for (int i = 0; i < evals.size(); ++i) {
    if (evals(i) > tol) {
      ++r.rank;
    }
  }
  return r;
}

inline Eigen::MatrixXd InformationFromCRS(const ceres::CRSMatrix& crs) {
  const int n = std::max(0, crs.num_cols);
  Eigen::MatrixXd F = Eigen::MatrixXd::Zero(n, n);
  for (int row = 0; row < crs.num_rows; ++row) {
    const int begin = crs.rows[row];
    const int end = crs.rows[row + 1];
    for (int k = begin; k < end; ++k) {
      const int col_k = crs.cols[k];
      const double val_k = crs.values[k];
      for (int l = k; l < end; ++l) {
        const int col_l = crs.cols[l];
        const double val_l = crs.values[l];
        F(col_k, col_l) += val_k * val_l;
        if (col_k != col_l) {
          F(col_l, col_k) += val_k * val_l;
        }
      }
    }
  }
  return F;
}

struct ExtrinsicState {
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  Eigen::Vector3d t = Eigen::Vector3d::Zero();
  double t_d = 0.0;
};

inline SE3d StateToSE3(const ExtrinsicState& s) {
  return SE3d(SO3d(s.q), s.t);
}

inline Eigen::Vector3d WorldPointVelocity(const BodyTrajectory& traj,
                                          double t_s,
                                          const Eigen::Vector3d& lever_b) {
  const double eps = 1e-3;
  const Eigen::Vector3d p0 = traj.position_wb(t_s - eps) +
                           traj.rotation_wb(t_s - eps) * lever_b;
  const Eigen::Vector3d p1 = traj.position_wb(t_s + eps) +
                           traj.rotation_wb(t_s + eps) * lever_b;
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

}  // namespace two_stage_probe
}  // namespace clic_calib
