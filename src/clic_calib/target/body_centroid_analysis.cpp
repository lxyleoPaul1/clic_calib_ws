#include <clic_calib/target/body_centroid_analysis.h>

#include <cmath>

namespace clic_calib {
namespace {

Eigen::Vector3d EmpiricalCentroidBody(const SE3d& T_WB, const SE3d& T_LW,
                                      const BodyClusterObservation& obs) {
  if (obs.raw_points_L_.empty()) {
    const SE3d T_WL = T_LW.inverse();
    return T_WB.so3().inverse() *
           (T_WL * obs.centroid_L_ - T_WB.translation());
  }
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  const SE3d T_WL = T_LW.inverse();
  for (const auto& p_L : obs.raw_points_L_) {
    sum += T_WB.so3().inverse() * (T_WL * p_L - T_WB.translation());
  }
  return sum / static_cast<double>(obs.raw_points_L_.size());
}

}  // namespace

Eigen::Vector3d CentroidBackprojectToBody(const SE3d& T_WB, const SE3d& T_LW,
                                          const Eigen::Vector3d& c_L) {
  const SE3d T_WL = T_LW.inverse();
  return T_WB.so3().inverse() * (T_WL * c_L - T_WB.translation());
}

Eigen::Vector3d ComputeBConstFromCentroidBackproject(
    const BodyTrajectory& traj, const SE3d& T_LW, double t_d_L_s,
    const Eigen::Vector3d& L_B_nominal,
    const std::vector<BodyClusterObservation>& observations) {
  if (observations.empty()) {
    return Eigen::Vector3d::Zero();
  }
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  for (const auto& obs : observations) {
    const double t_world = obs.t_sensor_ - t_d_L_s;
    const SE3d T_WB = traj.pose_wb(t_world);
    sum += CentroidBackprojectToBody(T_WB, T_LW, obs.centroid_L_);
  }
  const Eigen::Vector3d mean_centroid_B =
      sum / static_cast<double>(observations.size());
  return mean_centroid_B - L_B_nominal;
}

BodyCentroidBiasDecomposition DecomposeBodyCentroidBias(
    const BodyTrajectory& traj, const SE3d& T_LW, double t_d_L_s,
    const Eigen::Vector3d& L_B_nominal,
    const std::vector<BodyClusterObservation>& observations) {
  BodyCentroidBiasDecomposition out;
  if (observations.empty()) {
    return out;
  }

  std::vector<Eigen::Vector3d> biases;
  biases.reserve(observations.size());
  for (const auto& obs : observations) {
    const double t_world = obs.t_sensor_ - t_d_L_s;
    const SE3d T_WB = traj.pose_wb(t_world);
    const Eigen::Vector3d c_emp_B = EmpiricalCentroidBody(T_WB, T_LW, obs);
    biases.push_back(c_emp_B - L_B_nominal);
  }

  out.num_frames = static_cast<int>(biases.size());
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  for (const auto& b : biases) {
    sum += b;
  }
  out.constant_component_B = sum / static_cast<double>(biases.size());
  out.constant_norm_mm = out.constant_component_B.norm() * 1e3;

  double sq = 0.0;
  double max_v = 0.0;
  for (const auto& b : biases) {
    const Eigen::Vector3d v = b - out.constant_component_B;
    const double n = v.norm();
    sq += n * n;
    max_v = std::max(max_v, n);
  }
  out.varying_rms_mm =
      std::sqrt(sq / static_cast<double>(biases.size())) * 1e3;
  out.max_varying_mm = max_v * 1e3;
  return out;
}

Eigen::Vector3d EstimateObservedMeanBodyLever(
    const BodyTrajectory& traj, const SE3d& T_LW, double t_d_L_s,
    const std::vector<BodyClusterObservation>& observations) {
  if (observations.empty()) {
    return Eigen::Vector3d::Zero();
  }
  const SE3d T_WL = T_LW.inverse();
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  int count = 0;
  for (const auto& obs : observations) {
    const double t_world = obs.t_sensor_ - t_d_L_s;
    const SE3d T_WB = traj.pose_wb(t_world);
    const Eigen::Vector3d L_obs =
        T_WB.so3().inverse() * (T_WL * obs.centroid_L_ - T_WB.translation());
    sum += L_obs;
    ++count;
  }
  return sum / static_cast<double>(count);
}

namespace {

double YawFromRwb(const SO3d& R) {
  const Eigen::Matrix3d M = R.matrix();
  return std::atan2(M(1, 0), M(0, 0)) * 180.0 / M_PI;
}

double PitchFromRwb(const SO3d& R) {
  const Eigen::Matrix3d M = R.matrix();
  return std::atan2(-M(2, 0),
                    std::sqrt(M(2, 1) * M(2, 1) + M(2, 2) * M(2, 2))) *
         180.0 / M_PI;
}

}  // namespace

TrajectoryAttitudeSpread ComputeAttitudeSpreadAtObservations(
    const BodyTrajectory& traj, double t_d_L_s,
    const std::vector<BodyClusterObservation>& observations) {
  TrajectoryAttitudeSpread out;
  if (observations.empty()) {
    return out;
  }
  double yaw_min = 1e9;
  double yaw_max = -1e9;
  double pitch_min = 1e9;
  double pitch_max = -1e9;
  double sum_cos = 0.0;
  double sum_sin = 0.0;
  double pitch_sum = 0.0;
  double pitch_sq_sum = 0.0;
  for (const auto& obs : observations) {
    const double t_world = obs.t_sensor_ - t_d_L_s;
    const SO3d R = traj.rotation_wb(t_world);
    const double yaw = YawFromRwb(R);
    const double pitch = PitchFromRwb(R);
    const double yaw_rad = yaw * M_PI / 180.0;
    sum_cos += std::cos(yaw_rad);
    sum_sin += std::sin(yaw_rad);
    pitch_sum += pitch;
    pitch_sq_sum += pitch * pitch;
    yaw_min = std::min(yaw_min, yaw);
    yaw_max = std::max(yaw_max, yaw);
    pitch_min = std::min(pitch_min, pitch);
    pitch_max = std::max(pitch_max, pitch);
  }
  const double n = static_cast<double>(observations.size());
  out.num_frames = static_cast<int>(n);
  out.yaw_min_deg = yaw_min;
  out.yaw_max_deg = yaw_max;
  out.yaw_span_deg = yaw_max - yaw_min;
  out.pitch_min_deg = pitch_min;
  out.pitch_max_deg = pitch_max;
  out.pitch_span_deg = pitch_max - pitch_min;
  const double R_len = std::sqrt(sum_cos * sum_cos + sum_sin * sum_sin);
  out.yaw_circular_variance = 1.0 - R_len / n;
  const double pitch_mean = pitch_sum / n;
  out.pitch_std_deg = std::sqrt(std::max(0.0, pitch_sq_sum / n - pitch_mean * pitch_mean));
  return out;
}

WorldBiasDirectionStats ComputeWorldBiasDirectionStats(
    const BodyTrajectory& traj, const SE3d& T_LW, double t_d_L_s,
    const Eigen::Vector3d& b_const_B,
    const std::vector<BodyClusterObservation>& observations) {
  WorldBiasDirectionStats out;
  if (observations.empty()) {
    return out;
  }
  const Eigen::Matrix3d R_LW = T_LW.so3().matrix();
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  double h_sq = 0.0;
  double v_sq = 0.0;
  for (const auto& obs : observations) {
    const double t_world = obs.t_sensor_ - t_d_L_s;
    const Eigen::Matrix3d R_WB = traj.rotation_wb(t_world).matrix();
    const Eigen::Vector3d v_W = R_LW * R_WB * b_const_B;
    sum += v_W;
    const Eigen::Vector3d v_h(v_W.x(), v_W.y(), 0.0);
    h_sq += v_h.squaredNorm();
    v_sq += v_W.z() * v_W.z();
  }
  const double n = static_cast<double>(observations.size());
  out.num_frames = static_cast<int>(n);
  out.mean_W = sum / n;
  out.mean_horizontal_norm_mm = Eigen::Vector3d(out.mean_W.x(), out.mean_W.y(), 0.0).norm() * 1e3;
  out.mean_abs_vertical_mm = std::abs(out.mean_W.z()) * 1e3;
  out.horizontal_rms_mm = std::sqrt(h_sq / n) * 1e3;
  out.vertical_rms_mm = std::sqrt(v_sq / n) * 1e3;
  return out;
}

namespace {

double PearsonCorrelation(const std::vector<double>& x,
                          const std::vector<double>& y) {
  if (x.size() != y.size() || x.size() < 2) {
    return 0.0;
  }
  const double n = static_cast<double>(x.size());
  double mx = 0.0;
  double my = 0.0;
  for (size_t i = 0; i < x.size(); ++i) {
    mx += x[i];
    my += y[i];
  }
  mx /= n;
  my /= n;
  double sxx = 0.0;
  double syy = 0.0;
  double sxy = 0.0;
  for (size_t i = 0; i < x.size(); ++i) {
    const double dx = x[i] - mx;
    const double dy = y[i] - my;
    sxx += dx * dx;
    syy += dy * dy;
    sxy += dx * dy;
  }
  if (sxx < 1e-18 || syy < 1e-18) {
    return 0.0;
  }
  return sxy / std::sqrt(sxx * syy);
}

double LinearSlope(const std::vector<double>& x, const std::vector<double>& y) {
  if (x.size() != y.size() || x.size() < 2) {
    return 0.0;
  }
  const double n = static_cast<double>(x.size());
  double mx = 0.0;
  double my = 0.0;
  for (size_t i = 0; i < x.size(); ++i) {
    mx += x[i];
    my += y[i];
  }
  mx /= n;
  my /= n;
  double sxx = 0.0;
  double sxy = 0.0;
  for (size_t i = 0; i < x.size(); ++i) {
    const double dx = x[i] - mx;
    sxx += dx * dx;
    sxy += dx * (y[i] - my);
  }
  if (sxx < 1e-18) {
    return 0.0;
  }
  return sxy / sxx;
}

Eigen::Vector3d ObservedLeverBody(const BodyTrajectory& traj,
                                  const SE3d& T_LW, double t_d_L_s,
                                  const BodyClusterObservation& obs) {
  const double t_world = obs.t_sensor_ - t_d_L_s;
  const SE3d T_WB = traj.pose_wb(t_world);
  const SE3d T_WL = T_LW.inverse();
  return T_WB.so3().inverse() *
         (T_WL * obs.centroid_L_ - T_WB.translation());
}

}  // namespace

RangeBiasScatterReport BuildRangeBiasScatterReport(
    const BodyTrajectory& traj, const SE3d& T_LW, double t_d_L_s,
    const Eigen::Vector3d& L_B_nominal,
    const std::vector<BodyClusterObservation>& observations,
    const std::vector<int>& visible_faces_per_frame) {
  RangeBiasScatterReport out;
  if (observations.empty()) {
    return out;
  }
  std::vector<double> ranges;
  std::vector<double> bias_norms_mm;
  std::vector<double> point_counts;
  std::vector<double> visible_faces;
  double sum_bias_1 = 0.0;
  double sum_bias_2 = 0.0;
  double sum_bias_3 = 0.0;
  int n1 = 0;
  int n2 = 0;
  int n3 = 0;

  for (size_t i = 0; i < observations.size(); ++i) {
    const auto& obs = observations[i];
    const Eigen::Vector3d L_obs = ObservedLeverBody(traj, T_LW, t_d_L_s, obs);
    const Eigen::Vector3d bias_B = L_obs - L_B_nominal;
    RangeBiasScatterSample s;
    s.range_m = obs.mean_range_m_;
    s.bias_B = bias_B;
    s.bias_norm_mm = bias_B.norm() * 1e3;
    s.point_count = obs.point_count_;
    s.visible_faces =
        (i < visible_faces_per_frame.size()) ? visible_faces_per_frame[i] : 0;
    out.samples.push_back(s);

    ranges.push_back(s.range_m);
    bias_norms_mm.push_back(s.bias_norm_mm);
    point_counts.push_back(static_cast<double>(s.point_count));
    visible_faces.push_back(static_cast<double>(s.visible_faces));

    if (s.visible_faces <= 1) {
      sum_bias_1 += s.bias_norm_mm;
      ++n1;
      ++out.count_faces_1;
    } else if (s.visible_faces == 2) {
      sum_bias_2 += s.bias_norm_mm;
      ++n2;
      ++out.count_faces_2;
    } else {
      sum_bias_3 += s.bias_norm_mm;
      ++n3;
      ++out.count_faces_3plus;
    }
  }

  out.corr_range_bias_norm = PearsonCorrelation(ranges, bias_norms_mm);
  out.bias_norm_slope_mm_per_m = LinearSlope(ranges, bias_norms_mm);
  out.corr_range_point_count = PearsonCorrelation(ranges, point_counts);
  out.corr_range_visible_faces = PearsonCorrelation(ranges, visible_faces);
  out.mean_bias_norm_faces_1_mm = n1 > 0 ? sum_bias_1 / n1 : 0.0;
  out.mean_bias_norm_faces_2_mm = n2 > 0 ? sum_bias_2 / n2 : 0.0;
  out.mean_bias_norm_faces_3plus_mm = n3 > 0 ? sum_bias_3 / n3 : 0.0;
  out.mechanism = ClassifyRangeBiasMechanism(out);
  return out;
}

RangeBiasMechanism ClassifyRangeBiasMechanism(
    const RangeBiasScatterReport& report) {
  const double abs_r_bias = std::abs(report.corr_range_bias_norm);
  const double abs_r_pts = std::abs(report.corr_range_point_count);
  const double abs_r_faces = std::abs(report.corr_range_visible_faces);

  const double face_jump =
      std::abs(report.mean_bias_norm_faces_2_mm -
               report.mean_bias_norm_faces_1_mm);
  const bool face_transition_dominant =
      report.count_faces_1 > 0 && report.count_faces_2 > 0 &&
      abs_r_faces > 0.35 && face_jump > 15.0;

  if (face_transition_dominant) {
    return RangeBiasMechanism::kRangeWindow;
  }
  if (abs_r_bias >= 0.40 &&
      abs_r_bias >= abs_r_pts * 0.85) {
    return RangeBiasMechanism::kLinearPB;
  }
  if (abs_r_pts >= 0.40 && abs_r_pts > abs_r_bias) {
    return RangeBiasMechanism::kRangeWeighting;
  }
  if (abs_r_bias >= 0.25) {
    return RangeBiasMechanism::kLinearPB;
  }
  return RangeBiasMechanism::kRangeWeighting;
}

LinearRangeLeverModel FitLinearRangeBodyLever(
    const BodyTrajectory& traj, const SE3d& T_LW, double t_d_L_s,
    const std::vector<BodyClusterObservation>& observations) {
  LinearRangeLeverModel out;
  if (observations.size() < 3) {
    return out;
  }
  std::vector<double> ranges;
  std::vector<Eigen::Vector3d> levers;
  ranges.reserve(observations.size());
  levers.reserve(observations.size());
  for (const auto& obs : observations) {
    ranges.push_back(obs.mean_range_m_);
    levers.push_back(ObservedLeverBody(traj, T_LW, t_d_L_s, obs));
  }
  for (int axis = 0; axis < 3; ++axis) {
    std::vector<double> y;
    y.reserve(levers.size());
    for (const auto& L : levers) {
      y.push_back(L[axis]);
    }
    const double slope = LinearSlope(ranges, y);
    out.L1[axis] = slope;
    double mean_r = 0.0;
    double mean_y = 0.0;
    for (size_t i = 0; i < ranges.size(); ++i) {
      mean_r += ranges[i];
      mean_y += y[i];
    }
    mean_r /= static_cast<double>(ranges.size());
    mean_y /= static_cast<double>(ranges.size());
    out.L0[axis] = mean_y - slope * mean_r;
  }
  return out;
}

std::vector<Eigen::Vector3d> EvaluateLinearRangeLever(
    const LinearRangeLeverModel& model,
    const std::vector<BodyClusterObservation>& observations) {
  std::vector<Eigen::Vector3d> out;
  out.reserve(observations.size());
  for (const auto& obs : observations) {
    out.push_back(model.L0 + model.L1 * obs.mean_range_m_);
  }
  return out;
}

Eigen::Vector3d LidarDirectionInBody(const SE3d& T_WB,
                                     const Eigen::Vector3d& p_wb_W,
                                     const Eigen::Vector3d& lidar_post_W) {
  Eigen::Vector3d u_W = lidar_post_W - p_wb_W;
  const double n = u_W.norm();
  if (n < 1e-9) {
    return Eigen::Vector3d::UnitX();
  }
  u_W /= n;
  return T_WB.so3().inverse() * u_W;
}

AspectBiasScatterReport BuildAspectBiasScatterReport(
    const BodyTrajectory& traj, const SE3d& T_LW, double t_d_L_s,
    const Eigen::Vector3d& L_B_nominal,
    const Eigen::Vector3d& lidar_post_W,
    const std::vector<BodyClusterObservation>& observations) {
  AspectBiasScatterReport out;
  if (observations.empty()) {
    return out;
  }

  std::vector<double> u_az_deg;
  std::vector<double> bx;
  std::vector<double> by;
  std::vector<double> bz;
  std::vector<Eigen::Vector3d> biases;
  u_az_deg.reserve(observations.size());
  bx.reserve(observations.size());
  by.reserve(observations.size());
  bz.reserve(observations.size());
  biases.reserve(observations.size());

  for (const auto& obs : observations) {
    const double t_world = obs.t_sensor_ - t_d_L_s;
    const SE3d T_WB = traj.pose_wb(t_world);
    const Eigen::Vector3d u_B = LidarDirectionInBody(
        T_WB, T_WB.translation(), lidar_post_W);
    const double u_az = std::atan2(u_B.y(), u_B.x()) * 180.0 / M_PI;
    const Eigen::Vector3d bias_B =
        ObservedLeverBody(traj, T_LW, t_d_L_s, obs) - L_B_nominal;

    AspectBiasScatterSample s;
    s.u_B_azimuth_deg = u_az;
    s.bias_B = bias_B;
    out.samples.push_back(s);

    u_az_deg.push_back(u_az);
    bx.push_back(bias_B.x());
    by.push_back(bias_B.y());
    bz.push_back(bias_B.z());
    biases.push_back(bias_B);
  }

  out.corr_u_az_bias_x = PearsonCorrelation(u_az_deg, bx);
  out.corr_u_az_bias_y = PearsonCorrelation(u_az_deg, by);
  out.corr_u_az_bias_z = PearsonCorrelation(u_az_deg, bz);

  double mean_az = 0.0;
  for (double a : u_az_deg) {
    mean_az += a;
  }
  mean_az /= static_cast<double>(u_az_deg.size());
  double az_sq = 0.0;
  for (double a : u_az_deg) {
    const double d = a - mean_az;
    az_sq += d * d;
  }
  out.u_B_azimuth_std_deg =
      std::sqrt(az_sq / static_cast<double>(u_az_deg.size()));

  const auto decomp = DecomposeBodyCentroidBias(traj, T_LW, t_d_L_s, L_B_nominal,
                                                observations);
  out.bias_varying_rms_mm = decomp.varying_rms_mm;
  return out;
}

TidalLockAudit AuditOrbitYawTidalLock(
    double t0, double t1, double dt,
    const std::function<SE3d(double)>& pose_at_t) {
  TidalLockAudit out;
  std::vector<double> orbit_az;
  std::vector<double> body_yaw;
  std::vector<double> yaw_minus_orbit_deg;
  double orbit_min = 1e9;
  double orbit_max = -1e9;
  double yaw_min = 1e9;
  double yaw_max = -1e9;
  double sum_cos = 0.0;
  double sum_sin = 0.0;
  for (double t = t0; t <= t1 + 1e-9; t += dt) {
    const SE3d T_WB = pose_at_t(t);
    const Eigen::Vector3d p = T_WB.translation();
    const double az_rad = std::atan2(p.y(), p.x());
    const double az = az_rad * 180.0 / M_PI;
    const Eigen::Matrix3d M = T_WB.so3().matrix();
    const double yaw_rad = std::atan2(M(1, 0), M(0, 0));
    const double yaw = yaw_rad * 180.0 / M_PI;
    orbit_az.push_back(az);
    body_yaw.push_back(yaw);
    const double d_rad = yaw_rad - az_rad;
    sum_cos += std::cos(d_rad);
    sum_sin += std::sin(d_rad);
    yaw_minus_orbit_deg.push_back(d_rad * 180.0 / M_PI);
    orbit_min = std::min(orbit_min, az);
    orbit_max = std::max(orbit_max, az);
    yaw_min = std::min(yaw_min, yaw);
    yaw_max = std::max(yaw_max, yaw);
  }
  out.corr_orbit_azimuth_body_yaw =
      PearsonCorrelation(orbit_az, body_yaw);
  out.orbit_azimuth_span_deg = orbit_max - orbit_min;
  out.body_yaw_span_deg = yaw_max - yaw_min;
  const double n = static_cast<double>(yaw_minus_orbit_deg.size());
  const double R = std::sqrt(sum_cos * sum_cos + sum_sin * sum_sin);
  const double circ_var = 1.0 - R / std::max(n, 1.0);
  out.yaw_minus_orbit_std_deg = std::sqrt(std::max(0.0, circ_var)) * 180.0;
  out.tidal_locked = out.yaw_minus_orbit_std_deg < 15.0;
  return out;
}

}  // namespace clic_calib
