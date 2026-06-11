#include <clic_calib/estimator/body_gated_calibration.h>

#include <clic_calib/target/body_centroid_analysis.h>

namespace clic_calib {
namespace {

BodyExtrinsicMetrics MetricsFromSolve(const SE3d& T_est, const SE3d& T_gt,
                                      bool ok) {
  BodyExtrinsicMetrics m;
  m.stage2_ok = ok;
  const SO3d R_err = T_gt.so3().inverse() * T_est.so3();
  m.rot_deg = R_err.log().norm() * 180.0 / M_PI;
  m.trans_mm = (T_est.translation() - T_gt.translation()).norm() * 1e3;
  return m;
}

bool FitStage1AndGeometricInit(
    const BodyGatedCalibrationInput& in, const LeverArmConfig& levers,
    const TwoStagePipelineConfig& cfg, Stage1TrajectoryResult* s1_out,
    GeometricInitReport* geo_out) {
  const Stage1TrajectoryInput s1_in =
      Stage1TrajectoryInput::FromRtkAttitudeStreams(in.rtk, in.attitude);
  const Stage1TrajectoryResult s1 =
      Stage1TrajectoryFitter::Fit(s1_in, levers, cfg.stage1);
  if (!s1.summary.IsSolutionUsable() || !s1.trajectory) {
    return false;
  }
  ExtrinsicInitializerConfig init_cfg = cfg.init;
  init_cfg.lidar_target_mode = LidarTargetMode::kBodyCluster;
  try {
    *geo_out = ExtrinsicInitializer::FromGeometric(
        *s1.trajectory, {}, in.body_obs, in.tags, levers, init_cfg);
  } catch (const std::exception&) {
    return false;
  }
  *s1_out = s1;
  return true;
}

struct BodyPathOutcome {
  BodyExtrinsicMetrics metrics;
  SE3d T_LW_est;
  double refine_cost = 0.0;
};

BodyPathOutcome RunBodyCentroidPath(
    const BodyGatedCalibrationInput& in, const LeverArmConfig& levers,
    const NoiseModel& noise, const TwoStagePipelineConfig& base_cfg,
    BodyLeverArmMode lever_mode,
    const std::vector<double>* temporal_sqrt_info_scales) {
  BodyPathOutcome out;
  TwoStagePipelineConfig cfg = base_cfg;
  cfg.init.lidar_target_mode = LidarTargetMode::kBodyCluster;
  cfg.refine.lidar_target_mode = LidarTargetMode::kBodyCluster;
  cfg.refine.body_lever_arm_mode = lever_mode;
  if (temporal_sqrt_info_scales != nullptr) {
    cfg.refine.body_temporal_sqrt_info_scales = *temporal_sqrt_info_scales;
  }

  Stage1TrajectoryResult s1;
  GeometricInitReport geo;
  if (!FitStage1AndGeometricInit(in, levers, cfg, &s1, &geo)) {
    return out;
  }

  LeverArmConfig levers_refine = levers;
  ExtrinsicInitializerConfig init_cfg = cfg.init;
  init_cfg.lidar_target_mode = LidarTargetMode::kBodyCluster;

  if (lever_mode == BodyLeverArmMode::kObservedMean) {
    levers_refine.L_B_to_body_centroid = EstimateObservedMeanBodyLever(
        *s1.trajectory, geo.init.T_LW, in.nominal_t_d_L_s, in.body_obs,
        temporal_sqrt_info_scales);
    try {
      geo = ExtrinsicInitializer::FromGeometric(
          *s1.trajectory, {}, in.body_obs, in.tags, levers_refine, init_cfg);
    } catch (const std::exception&) {
      return out;
    }
    cfg.refine.body_lever_arm_mode = BodyLeverArmMode::kNominalYaml;
  }

  const ExtrinsicRefinerResult s2 = ExtrinsicRefiner::Refine(
      *s1.trajectory, {}, in.body_obs, in.tags, levers_refine, noise, geo.init,
      cfg.refine);
  out.T_LW_est = SE3d(s2.lidar.q, s2.lidar.t);
  out.refine_cost = s2.summary.final_cost;
  out.metrics.stage2_ok = s2.converged;
  return out;
}

}  // namespace

IterativeObservedMeanResult RunBodyPathIterativeObservedMean(
    const BodyGatedCalibrationInput& in, const LeverArmConfig& levers,
    const NoiseModel& noise, const TwoStagePipelineConfig& base_cfg,
    int num_iters, const std::vector<double>* temporal_sqrt_info_scales) {
  IterativeObservedMeanResult out;
  if (num_iters < 1) {
    return out;
  }
  TwoStagePipelineConfig cfg = base_cfg;
  cfg.init.lidar_target_mode = LidarTargetMode::kBodyCluster;
  cfg.refine.lidar_target_mode = LidarTargetMode::kBodyCluster;
  cfg.refine.body_lever_arm_mode = BodyLeverArmMode::kNominalYaml;
  if (temporal_sqrt_info_scales != nullptr) {
    cfg.refine.body_temporal_sqrt_info_scales = *temporal_sqrt_info_scales;
  }

  Stage1TrajectoryResult s1;
  GeometricInitReport geo;
  if (!FitStage1AndGeometricInit(in, levers, cfg, &s1, &geo)) {
    return out;
  }

  LeverArmConfig levers_refine = levers;
  ExtrinsicInitializerConfig init_cfg = cfg.init;
  init_cfg.lidar_target_mode = LidarTargetMode::kBodyCluster;

  double best_cost = std::numeric_limits<double>::infinity();
  for (int k = 0; k < num_iters; ++k) {
    levers_refine.L_B_to_body_centroid = EstimateObservedMeanBodyLever(
        *s1.trajectory, geo.init.T_LW, in.nominal_t_d_L_s, in.body_obs,
        temporal_sqrt_info_scales);
    try {
      geo = ExtrinsicInitializer::FromGeometric(
          *s1.trajectory, {}, in.body_obs, in.tags, levers_refine, init_cfg);
    } catch (const std::exception&) {
      break;
    }
    const ExtrinsicRefinerResult s2 = ExtrinsicRefiner::Refine(
        *s1.trajectory, {}, in.body_obs, in.tags, levers_refine, noise,
        geo.init, cfg.refine);
    geo.init.T_LW = SE3d(s2.lidar.q, s2.lidar.t);
    const double step_cost = s2.summary.final_cost;
    out.trans_mm_per_iter.push_back(0.0);
    if (s2.converged && step_cost + 1e-9 < best_cost) {
      best_cost = step_cost;
      out.final_metrics.stage2_ok = true;
      out.final_T_LW = geo.init.T_LW;
      out.final_refine_cost = step_cost;
      if (in.gt_T_LW) {
        out.final_metrics =
            MetricsFromSolve(out.final_T_LW, *in.gt_T_LW, true);
      }
    } else {
      break;
    }
  }
  return out;
}

GatedBodyCalibrationResult CalibrateBodyGatedObservedMean(
    const BodyGatedCalibrationInput& in, const LeverArmConfig& levers,
    const NoiseModel& noise, const TwoStagePipelineConfig& base_cfg,
    int observed_mean_iters, const ObservedMeanAspectGate& gate,
    double u_B_azimuth_std_deg,
    const std::vector<double>* temporal_sqrt_info_scales) {
  GatedBodyCalibrationResult out;
  Stage1TrajectoryResult s1_spread;
  GeometricInitReport geo_spread;
  if (FitStage1AndGeometricInit(in, levers, base_cfg, &s1_spread, &geo_spread) &&
      s1_spread.trajectory) {
    out.aspect = ComputeAttitudeSpreadAtObservations(
        *s1_spread.trajectory, in.nominal_t_d_L_s, in.body_obs);
  }

  const auto cent = RunBodyCentroidPath(in, levers, noise, base_cfg,
                                        BodyLeverArmMode::kNominalYaml,
                                        temporal_sqrt_info_scales);
  out.T_LW_centroid = cent.T_LW_est;
  out.centroid_refine_cost = cent.refine_cost;
  out.centroid_only.stage2_ok = cent.metrics.stage2_ok;
  if (in.gt_T_LW) {
    out.centroid_only =
        MetricsFromSolve(cent.T_LW_est, *in.gt_T_LW, cent.metrics.stage2_ok);
  }
  out.observed_mean = out.centroid_only;
  out.T_LW_observed_mean = out.T_LW_centroid;

  if (!ShouldApplyObservedMeanPB(out.aspect, gate, u_B_azimuth_std_deg)) {
    return out;
  }

  const auto iter = RunBodyPathIterativeObservedMean(
      in, levers, noise, base_cfg, observed_mean_iters,
      temporal_sqrt_info_scales);
  bool trans_ok = iter.final_metrics.stage2_ok;
  if (in.gt_T_LW) {
    const BodyExtrinsicMetrics m_iter =
        MetricsFromSolve(iter.final_T_LW, *in.gt_T_LW, iter.final_metrics.stage2_ok);
    trans_ok = iter.final_metrics.stage2_ok &&
               m_iter.trans_mm <= out.centroid_only.trans_mm + 1e-3;
  }
  const bool cost_ok = iter.final_metrics.stage2_ok &&
                       iter.final_refine_cost <= cent.refine_cost + 1e-6;
  if (trans_ok && cost_ok) {
    if (in.gt_T_LW) {
      out.observed_mean =
          MetricsFromSolve(iter.final_T_LW, *in.gt_T_LW, iter.final_metrics.stage2_ok);
    } else {
      out.observed_mean = iter.final_metrics;
    }
    out.T_LW_observed_mean = iter.final_T_LW;
    out.observed_refine_cost = iter.final_refine_cost;
    out.observed_mean_applied = true;
  }
  return out;
}

}  // namespace clic_calib
