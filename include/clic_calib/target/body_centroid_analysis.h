#pragma once

#include <clic_calib/sensor_data/body_cluster_observation.h>
#include <clic_calib/spline/trajectory.h>
#include <clic_calib/utils/sophus_utils.hpp>

#include <Eigen/Core>

#include <functional>
#include <vector>

namespace clic_calib {

/**
 * @brief Per-frame body-centroid bias decomposition (body frame B).
 *
 * bias_B_i = empirical_centroid_B_i - L_B_nominal, where empirical centroid is
 * the mean of visible cluster points transformed into B (not L-frame mean).
 */
struct BodyCentroidBiasDecomposition {
  Eigen::Vector3d constant_component_B = Eigen::Vector3d::Zero();
  double constant_norm_mm = 0.0;
  double varying_rms_mm = 0.0;
  double max_varying_mm = 0.0;
  int num_frames = 0;
};

/**
 * @brief Decompose view-dependent centroid bias into body-fixed constant and
 *        per-frame varying parts.
 */
BodyCentroidBiasDecomposition DecomposeBodyCentroidBias(
    const BodyTrajectory& traj, const SE3d& T_LW, double t_d_L_s,
    const Eigen::Vector3d& L_B_nominal,
    const std::vector<BodyClusterObservation>& observations);

/** @brief c_L → body frame using GT/explicit T_WB, T_LW (centroid back-projection). */
Eigen::Vector3d CentroidBackprojectToBody(const SE3d& T_WB, const SE3d& T_LW,
                                          const Eigen::Vector3d& c_L);

/**
 * @brief b_const = mean_i (backproject(c_L_i) - L_B_nominal) at GT trajectory.
 */
Eigen::Vector3d ComputeBConstFromCentroidBackproject(
    const BodyTrajectory& traj, const SE3d& T_LW, double t_d_L_s,
    const Eigen::Vector3d& L_B_nominal,
    const std::vector<BodyClusterObservation>& observations);

/**
 * @brief Estimate p_B as the mean observed lever arm in B:
 *   L̄_B = mean_i R_WB_i^T (T_WL c_L_i - p_WB_i)
 */
Eigen::Vector3d EstimateObservedMeanBodyLever(
    const BodyTrajectory& traj, const SE3d& T_LW, double t_d_L_s,
    const std::vector<BodyClusterObservation>& observations,
    const std::vector<double>* temporal_weights = nullptr);

/** u_B jitter × b_const lever projection onto horizontal T_LW translation. */
struct AspectLeverTranslationProjection {
  Eigen::Vector3d b_const_B = Eigen::Vector3d::Zero();
  double u_B_azimuth_std_deg = 0.0;
  double bias_varying_rms_mm = 0.0;
  double lever_horizontal_mm = 0.0;
  /** lever_h × u_B_std[rad] × (varying_RMS / 1000) — first-order jitter budget. */
  double projected_jitter_trans_mm = 0.0;
};

AspectLeverTranslationProjection ComputeAspectLeverTranslationProjection(
    const BodyTrajectory& traj, const SE3d& T_LW, double t_d_L_s,
    const Eigen::Vector3d& L_B_nominal,
    const Eigen::Vector3d& lidar_post_W,
    const std::vector<BodyClusterObservation>& observations);

/** R_WB attitude spread at body-cluster observation times. */
struct TrajectoryAttitudeSpread {
  double yaw_min_deg = 0.0;
  double yaw_max_deg = 0.0;
  double yaw_span_deg = 0.0;
  double pitch_min_deg = 0.0;
  double pitch_max_deg = 0.0;
  double pitch_span_deg = 0.0;
  /** Circular variance of yaw: 1 − R/n, R = ||Σ(cos,sin)|| (0 = tight, 1 = uniform). */
  double yaw_circular_variance = 0.0;
  /** Linear std of pitch [deg] (pitch is not wrapped on this trajectory). */
  double pitch_std_deg = 0.0;
  int num_frames = 0;
};

TrajectoryAttitudeSpread ComputeAttitudeSpreadAtObservations(
    const BodyTrajectory& traj, double t_d_L_s,
    const std::vector<BodyClusterObservation>& observations);

/**
 * Per-frame world bias v_i = R_LW * R_WB(t_i) * b_const_B; reports mean and
 * horizontal/vertical decomposition (mechanism diagnostic).
 */
struct WorldBiasDirectionStats {
  Eigen::Vector3d mean_W = Eigen::Vector3d::Zero();
  double mean_horizontal_norm_mm = 0.0;
  double mean_abs_vertical_mm = 0.0;
  double horizontal_rms_mm = 0.0;
  double vertical_rms_mm = 0.0;
  int num_frames = 0;
};

WorldBiasDirectionStats ComputeWorldBiasDirectionStats(
    const BodyTrajectory& traj, const SE3d& T_LW, double t_d_L_s,
    const Eigen::Vector3d& b_const_B,
    const std::vector<BodyClusterObservation>& observations);

/** Mechanism suggested by range–bias scatter (b_const(range) hypothesis). */
enum class RangeBiasMechanism {
  kLinearPB,
  kRangeWeighting,
  kRangeWindow,
};

struct RangeBiasScatterSample {
  double range_m = 0.0;
  Eigen::Vector3d bias_B = Eigen::Vector3d::Zero();
  double bias_norm_mm = 0.0;
  int point_count = 0;
  int visible_faces = 0;
};

struct RangeBiasScatterReport {
  std::vector<RangeBiasScatterSample> samples;
  double corr_range_bias_norm = 0.0;
  double bias_norm_slope_mm_per_m = 0.0;
  double corr_range_point_count = 0.0;
  double corr_range_visible_faces = 0.0;
  double mean_bias_norm_faces_1_mm = 0.0;
  double mean_bias_norm_faces_2_mm = 0.0;
  double mean_bias_norm_faces_3plus_mm = 0.0;
  int count_faces_1 = 0;
  int count_faces_2 = 0;
  int count_faces_3plus = 0;
  RangeBiasMechanism mechanism = RangeBiasMechanism::kLinearPB;
};

/** Per-frame GT back-project bias vs range (+ optional visible-face counts). */
RangeBiasScatterReport BuildRangeBiasScatterReport(
    const BodyTrajectory& traj, const SE3d& T_LW, double t_d_L_s,
    const Eigen::Vector3d& L_B_nominal,
    const std::vector<BodyClusterObservation>& observations,
    const std::vector<int>& visible_faces_per_frame);

/** Classify (a) p_B(r), (b) far-range down-weight, or (c) multi-face window. */
RangeBiasMechanism ClassifyRangeBiasMechanism(
    const RangeBiasScatterReport& report);

/** L_B(r) = L0 + L1 * r fitted from observed levers at @p T_LW. */
struct LinearRangeLeverModel {
  Eigen::Vector3d L0 = Eigen::Vector3d::Zero();
  Eigen::Vector3d L1 = Eigen::Vector3d::Zero();
};

LinearRangeLeverModel FitLinearRangeBodyLever(
    const BodyTrajectory& traj, const SE3d& T_LW, double t_d_L_s,
    const std::vector<BodyClusterObservation>& observations);

std::vector<Eigen::Vector3d> EvaluateLinearRangeLever(
    const LinearRangeLeverModel& model,
    const std::vector<BodyClusterObservation>& observations);

/** Unit vector from body origin toward LiDAR post, expressed in body frame B. */
Eigen::Vector3d LidarDirectionInBody(const SE3d& T_WB,
                                     const Eigen::Vector3d& p_wb_W,
                                     const Eigen::Vector3d& lidar_post_W);

struct AspectBiasScatterSample {
  double u_B_azimuth_deg = 0.0;
  Eigen::Vector3d bias_B = Eigen::Vector3d::Zero();
};

struct AspectBiasScatterReport {
  std::vector<AspectBiasScatterSample> samples;
  double corr_u_az_bias_x = 0.0;
  double corr_u_az_bias_y = 0.0;
  double corr_u_az_bias_z = 0.0;
  double u_B_azimuth_std_deg = 0.0;
  double bias_varying_rms_mm = 0.0;
};

/** Per-frame GT bias_B vs u_B azimuth (LiDAR look direction in body). */
AspectBiasScatterReport BuildAspectBiasScatterReport(
    const BodyTrajectory& traj, const SE3d& T_LW, double t_d_L_s,
    const Eigen::Vector3d& L_B_nominal,
    const Eigen::Vector3d& lidar_post_W,
    const std::vector<BodyClusterObservation>& observations);

struct TidalLockAudit {
  double corr_orbit_azimuth_body_yaw = 0.0;
  double orbit_azimuth_span_deg = 0.0;
  double body_yaw_span_deg = 0.0;
  /** Circular std of (body_yaw − orbit_azimuth) [deg]; low ⇒ locked. */
  double yaw_minus_orbit_std_deg = 0.0;
  bool tidal_locked = false;
};

/**
 * Orbit azimuth atan2(y,x) vs body yaw on a pose law; |r|>0.95 ⇒ tidal lock.
 * @p pose_at_t  returns T_WB at world time t.
 */
TidalLockAudit AuditOrbitYawTidalLock(
    double t0, double t1, double dt,
    const std::function<SE3d(double)>& pose_at_t);

}  // namespace clic_calib
