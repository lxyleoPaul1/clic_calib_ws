#pragma once

#include "diagnostic/attitude_scenario_common.hpp"
#include "diagnostic/closed_form_init_common.hpp"
#include "experiments/body_sampling_common.hpp"
#include "experiments/noise_regime_common.hpp"

#include <clic_calib/config/body_model_config.h>
#include <clic_calib/estimator/body_gated_calibration.h>
#include <clic_calib/estimator/extrinsic_initializer.h>
#include <clic_calib/estimator/extrinsic_refiner.h>
#include <clic_calib/estimator/observed_mean_gate.h>
#include <clic_calib/estimator/stage1_trajectory_fitter.h>
#include <clic_calib/estimator/two_stage_pipeline.h>
#include <clic_calib/target/body_centroid_analysis.h>
#include <clic_calib/target/drone_model_registration.h>

#include <cmath>
#include <limits>
#include <string>

namespace clic_calib {
namespace experiments {

using clic_calib::ComputeUBAzimuthStdDeg;
using clic_calib::ObservedMeanAspectGate;
using clic_calib::ShouldApplyObservedMeanPB;

struct Phase15Scenario {
  SyntheticScenarioBundle sc;
  std::vector<BodyClusterObservation> body_cluster;
  SyntheticFlightGeometry geom;
};

struct Phase15TlwMetrics {
  double rot_deg = 0.0;
  double trans_mm = 0.0;
  bool stage2_ok = false;
};

struct Phase15BodyCalibOutcome {
  Phase15TlwMetrics metrics;
  SE3d T_LW_est;
  double refine_cost = 0.0;
};

inline bool PreparePhase15ScenarioTags(
    Phase15Scenario* ps, const LeverArmConfig& levers,
    const NoiseModel& noise,
    const two_stage_probe::CoarseExtrinsicInit& t_d,
    const two_stage_probe::SplineConfig& spline_cfg, uint32_t tag_seed,
    double stage1_alpha_p = 0.01, double stage1_alpha_R = 0.01) {
  const PinholeIntrinsics K{600.0, 600.0, 320.0, 240.0};
  const RadtanDistortion dist;
  const Eigen::Vector3d L_G_to_M = levers.L_G_to_M.at(0);
  Stage1TrajectoryConfig s1_cfg;
  s1_cfg.knot_interval_s = spline_cfg.knot_interval_s;
  s1_cfg.alpha_p = stage1_alpha_p;
  s1_cfg.alpha_R = stage1_alpha_R;
  s1_cfg.attitude_stride = 25;
  s1_cfg.trim_to_observation_support = true;
  const Stage1TrajectoryResult s1 = Stage1TrajectoryFitter::Fit(
      Stage1TrajectoryInput::FromRtkAttitudeStreams(ps->sc.rtk,
                                                    ps->sc.attitude_obs),
      levers, s1_cfg);
  if (!s1.summary.IsSolutionUsable() || !s1.trajectory) {
    return false;
  }
  ps->sc.tag_obs = two_stage_probe::SynthesizeTagObsForTrajectory(
      ps->sc.tag_obs, *s1.trajectory, ps->sc.gt.T_CW, levers.L_B_to_G, L_G_to_M,
      t_d.t_d_C_s, K, dist, noise, tag_seed);
  return true;
}

inline Phase15TlwMetrics TlwMetrics(const SE3d& T_est, const SE3d& T_gt,
                                    bool ok) {
  Phase15TlwMetrics m;
  m.stage2_ok = ok;
  const SO3d R_err = T_gt.so3().inverse() * T_est.so3();
  m.rot_deg = R_err.log().norm() * 180.0 / M_PI;
  m.trans_mm = (T_est.translation() - T_gt.translation()).norm() * 1e3;
  return m;
}

inline Phase15Scenario BuildPhase15ScenarioWithGeom(
    uint32_t seed, const NoiseModel& noise,
    const SyntheticFlightGeometry& geom) {
  Phase15Scenario out;
  out.geom = geom;
  out.sc =
      BuildNoisyScenarioFromGeometry(seed, seed, noise, geom);
  std::mt19937 rng_att(seed + 7919u);
  AppendAttitudeObservations(&out.sc, geom, AttitudeNoiseSpec(), &rng_att);
  const auto levers =
      LeverArmConfig::from_yaml(ConfigDirFromExperiments() + "/lever_arms.yaml");
  out.body_cluster = BuildBodyClusterObsForGeometry(
      out.sc.gt_traj, out.sc.gt.T_LW, out.sc.gt.t_d_L_s, levers, noise, geom,
      seed + 17u, true);
  return out;
}

inline Phase15Scenario BuildPhase15Scenario(uint32_t seed,
                                            const NoiseModel& noise) {
  return BuildPhase15ScenarioWithGeom(seed, noise,
                                      NearFieldFimScenarioGeometry());
}

inline TwoStagePipelineResult RunSpherePath(
    const Phase15Scenario& ps, const LeverArmConfig& levers,
    const NoiseModel& noise, const TwoStagePipelineConfig& cfg) {
  return TwoStagePipeline::Run(ps.sc.rtk, ps.sc.attitude_obs, ps.sc.lidar_obs,
                               ps.sc.tag_obs, levers, noise, cfg);
}

inline bool FitStage1AndGeometricInit(
    const Phase15Scenario& ps, const LeverArmConfig& levers,
    const TwoStagePipelineConfig& cfg,
    const std::vector<BodyClusterObservation>& body_obs,
    Stage1TrajectoryResult* s1_out, GeometricInitReport* geo_out) {
  const Stage1TrajectoryInput s1_in =
      Stage1TrajectoryInput::FromRtkAttitudeStreams(ps.sc.rtk,
                                                    ps.sc.attitude_obs);
  const Stage1TrajectoryResult s1 =
      Stage1TrajectoryFitter::Fit(s1_in, levers, cfg.stage1);
  if (!s1.summary.IsSolutionUsable() || !s1.trajectory) {
    return false;
  }
  ExtrinsicInitializerConfig init_cfg = cfg.init;
  init_cfg.lidar_target_mode = LidarTargetMode::kBodyCluster;
  try {
    *geo_out = ExtrinsicInitializer::FromGeometric(
        *s1.trajectory, {}, body_obs, ps.sc.tag_obs, levers, init_cfg);
  } catch (const std::exception&) {
    return false;
  }
  *s1_out = s1;
  return true;
}

inline Phase15TlwMetrics RunBodyPathWithExplicitLever(
    const Phase15Scenario& ps, const LeverArmConfig& levers,
    const NoiseModel& noise, const TwoStagePipelineConfig& base_cfg,
    BodyLeverArmMode lever_mode,
    const std::vector<BodyClusterObservation>& body_obs,
    const Eigen::Vector3d& L_B_override, bool use_override,
    SE3d* T_LW_est_out = nullptr, double* refine_cost_out = nullptr) {
  TwoStagePipelineConfig cfg = base_cfg;
  cfg.init.lidar_target_mode = LidarTargetMode::kBodyCluster;
  cfg.refine.lidar_target_mode = LidarTargetMode::kBodyCluster;
  cfg.refine.body_lever_arm_mode = lever_mode;

  Stage1TrajectoryResult s1;
  GeometricInitReport geo;
  if (!FitStage1AndGeometricInit(ps, levers, cfg, body_obs, &s1, &geo)) {
    return {};
  }

  LeverArmConfig levers_refine = levers;
  ExtrinsicInitializerConfig init_cfg = cfg.init;
  init_cfg.lidar_target_mode = LidarTargetMode::kBodyCluster;

  if (use_override) {
    levers_refine.L_B_to_body_centroid = L_B_override;
    try {
      geo = ExtrinsicInitializer::FromGeometric(
          *s1.trajectory, {}, body_obs, ps.sc.tag_obs, levers_refine,
          init_cfg);
    } catch (const std::exception&) {
      return {};
    }
    cfg.refine.body_lever_arm_mode = BodyLeverArmMode::kNominalYaml;
  } else if (lever_mode == BodyLeverArmMode::kObservedMean) {
    levers_refine.L_B_to_body_centroid = EstimateObservedMeanBodyLever(
        *s1.trajectory, geo.init.T_LW, ps.sc.gt.t_d_L_s, body_obs);
    try {
      geo = ExtrinsicInitializer::FromGeometric(
          *s1.trajectory, {}, body_obs, ps.sc.tag_obs, levers_refine,
          init_cfg);
    } catch (const std::exception&) {
      return {};
    }
    cfg.refine.body_lever_arm_mode = BodyLeverArmMode::kNominalYaml;
  } else if (lever_mode == BodyLeverArmMode::kJointOptimize) {
    CoarseExtrinsicInit init_lever = geo.init;
    init_lever.t_d_L_s = ps.sc.gt.t_d_L_s;
    levers_refine.L_B_to_body_centroid =
        ExtrinsicRefiner::OptimizeJointBodyLeverAtFixedExtrinsic(
            *s1.trajectory, body_obs, init_lever, levers, noise, cfg.refine);
    try {
      geo = ExtrinsicInitializer::FromGeometric(
          *s1.trajectory, {}, body_obs, ps.sc.tag_obs, levers_refine,
          init_cfg);
    } catch (const std::exception&) {
      return {};
    }
    cfg.refine.body_lever_arm_mode = BodyLeverArmMode::kNominalYaml;
  }

  const ExtrinsicRefinerResult s2 = ExtrinsicRefiner::Refine(
      *s1.trajectory, {}, body_obs, ps.sc.tag_obs, levers_refine, noise,
      geo.init, cfg.refine);

  const SE3d T_LW(s2.lidar.q, s2.lidar.t);
  if (T_LW_est_out) {
    *T_LW_est_out = T_LW;
  }
  if (refine_cost_out) {
    *refine_cost_out = s2.summary.final_cost;
  }
  Phase15TlwMetrics m = TlwMetrics(T_LW, ps.sc.gt.T_LW, s2.converged);
  return m;
}

inline Phase15BodyCalibOutcome RunBodyPathOutcome(
    const Phase15Scenario& ps, const LeverArmConfig& levers,
    const NoiseModel& noise, const TwoStagePipelineConfig& base_cfg,
    BodyLeverArmMode lever_mode,
    const std::vector<BodyClusterObservation>& body_obs) {
  Phase15BodyCalibOutcome out;
  out.metrics = RunBodyPathWithExplicitLever(
      ps, levers, noise, base_cfg, lever_mode, body_obs,
      Eigen::Vector3d::Zero(), false, &out.T_LW_est, &out.refine_cost);
  return out;
}

inline Phase15TlwMetrics RunBodyPath(
    const Phase15Scenario& ps, const LeverArmConfig& levers,
    const NoiseModel& noise, const TwoStagePipelineConfig& base_cfg,
    BodyLeverArmMode lever_mode,
    const std::vector<BodyClusterObservation>& body_obs,
    SE3d* T_LW_est_out = nullptr) {
  return RunBodyPathWithExplicitLever(ps, levers, noise, base_cfg, lever_mode,
                                      body_obs, Eigen::Vector3d::Zero(),
                                      false, T_LW_est_out);
}

struct JointLeverDiagnostic {
  Eigen::Vector3d L_B_before = Eigen::Vector3d::Zero();
  Eigen::Vector3d L_B_after = Eigen::Vector3d::Zero();
  Eigen::Vector3d t_LW_before = Eigen::Vector3d::Zero();
  Eigen::Vector3d t_LW_after = Eigen::Vector3d::Zero();
  double L_B_delta_mm = 0.0;
  double t_LW_delta_mm = 0.0;
  double dist_to_L_nom_plus_bconst_mm = 0.0;
};

inline Phase15TlwMetrics RunModelRegPath(
    const Phase15Scenario& ps, const LeverArmConfig& levers,
    const NoiseModel& noise, const TwoStagePipelineConfig& base_cfg,
    const BodyModelConfig& body_model,
    const std::vector<BodyClusterObservation>& body_obs) {
  TwoStagePipelineConfig cfg = base_cfg;
  cfg.init.lidar_target_mode = LidarTargetMode::kBodyCluster;
  cfg.refine.lidar_target_mode = LidarTargetMode::kBodyCluster;

  const Stage1TrajectoryInput s1_in =
      Stage1TrajectoryInput::FromRtkAttitudeStreams(ps.sc.rtk,
                                                    ps.sc.attitude_obs);
  const Stage1TrajectoryResult s1 =
      Stage1TrajectoryFitter::Fit(s1_in, levers, cfg.stage1);
  if (!s1.summary.IsSolutionUsable() || !s1.trajectory) {
    return {};
  }
  ExtrinsicInitializerConfig init_cfg = cfg.init;
  init_cfg.lidar_target_mode = LidarTargetMode::kBodyCluster;
  GeometricInitReport geo;
  try {
    geo = ExtrinsicInitializer::FromGeometric(
        *s1.trajectory, {}, body_obs, ps.sc.tag_obs, levers, init_cfg);
  } catch (const std::exception&) {
    return {};
  }
  const std::vector<BodyClusterObservation> body_reg =
      DroneModelRegistration::ApplyVisibilityAwareToObservations(
          body_obs, *s1.trajectory, geo.init.T_LW, ps.sc.gt.t_d_L_s, body_model);
  return RunBodyPath(ps, levers, noise, base_cfg,
                     BodyLeverArmMode::kNominalYaml, body_reg);
}

struct IterativeObservedMeanResult {
  Phase15TlwMetrics final_metrics;
  SE3d final_T_LW;
  double final_refine_cost = std::numeric_limits<double>::infinity();
  std::vector<double> trans_mm_per_iter;
  std::vector<Eigen::Vector3d> L_B_per_iter;
};

inline IterativeObservedMeanResult RunBodyPathIterativeObservedMean(
    const Phase15Scenario& ps, const LeverArmConfig& levers,
    const NoiseModel& noise, const TwoStagePipelineConfig& base_cfg,
    const std::vector<BodyClusterObservation>& body_obs, int num_iters,
    const std::vector<double>* temporal_sqrt_info_scales = nullptr) {
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
  if (!FitStage1AndGeometricInit(ps, levers, cfg, body_obs, &s1, &geo)) {
    return out;
  }

  LeverArmConfig levers_refine = levers;
  ExtrinsicInitializerConfig init_cfg = cfg.init;
  init_cfg.lidar_target_mode = LidarTargetMode::kBodyCluster;

  double best_cost = std::numeric_limits<double>::infinity();
  for (int k = 0; k < num_iters; ++k) {
    levers_refine.L_B_to_body_centroid = EstimateObservedMeanBodyLever(
        *s1.trajectory, geo.init.T_LW, ps.sc.gt.t_d_L_s, body_obs,
        temporal_sqrt_info_scales);
    out.L_B_per_iter.push_back(levers_refine.L_B_to_body_centroid);
    try {
      geo = ExtrinsicInitializer::FromGeometric(
          *s1.trajectory, {}, body_obs, ps.sc.tag_obs, levers_refine,
          init_cfg);
    } catch (const std::exception&) {
      break;
    }
    const ExtrinsicRefinerResult s2 = ExtrinsicRefiner::Refine(
        *s1.trajectory, {}, body_obs, ps.sc.tag_obs, levers_refine, noise,
        geo.init, cfg.refine);
    geo.init.T_LW = SE3d(s2.lidar.q, s2.lidar.t);
    const Phase15TlwMetrics m =
        TlwMetrics(geo.init.T_LW, ps.sc.gt.T_LW, s2.converged);
    const double step_cost = s2.summary.final_cost;
    out.trans_mm_per_iter.push_back(m.trans_mm);
    if (m.stage2_ok && step_cost + 1e-9 < best_cost) {
      best_cost = step_cost;
      out.final_metrics = m;
      out.final_T_LW = geo.init.T_LW;
      out.final_refine_cost = step_cost;
    } else {
      break;
    }
  }
  return out;
}

/** Gate / fallback / p_B audit for observed-mean path debugging. */
struct ObservedMeanPathAudit {
  bool gate_on = false;
  bool applied = false;
  bool trans_ok = false;
  bool cost_ok = false;
  TrajectoryAttitudeSpread aspect{};
  Eigen::Vector3d p_B_nominal = Eigen::Vector3d::Zero();
  Eigen::Vector3d p_B_after_stage1 = Eigen::Vector3d::Zero();
  Eigen::Vector3d p_B_final_iter = Eigen::Vector3d::Zero();
  double cent_trans_mm = 0.0;
  double obs_trans_mm = 0.0;
  double cent_cost = 0.0;
  double obs_cost = 0.0;
};

inline ObservedMeanPathAudit AuditObservedMeanPath(
    const Phase15Scenario& ps, const LeverArmConfig& levers,
    const NoiseModel& noise, const TwoStagePipelineConfig& base_cfg,
    const std::vector<BodyClusterObservation>& body_obs,
    double u_B_azimuth_std_deg, int observed_mean_iters = 3,
    const std::vector<double>* temporal_sqrt_info_scales = nullptr) {
  ObservedMeanPathAudit audit;
  audit.p_B_nominal = levers.L_B_to_body_centroid;
  audit.aspect = ComputeAttitudeSpreadAtObservations(
      ps.sc.gt_traj, ps.sc.gt.t_d_L_s, body_obs);
  audit.gate_on = ShouldApplyObservedMeanPB(audit.aspect, {},
                                            u_B_azimuth_std_deg);

  TwoStagePipelineConfig cent_cfg = base_cfg;
  if (temporal_sqrt_info_scales != nullptr) {
    cent_cfg.refine.body_temporal_sqrt_info_scales =
        *temporal_sqrt_info_scales;
  }
  const auto cent = RunBodyPathOutcome(ps, levers, noise, cent_cfg,
                                       BodyLeverArmMode::kNominalYaml,
                                       body_obs);
  audit.cent_trans_mm = cent.metrics.trans_mm;
  audit.cent_cost = cent.refine_cost;

  Stage1TrajectoryResult s1;
  GeometricInitReport geo;
  if (!FitStage1AndGeometricInit(ps, levers, cent_cfg, body_obs, &s1, &geo)) {
    return audit;
  }
  audit.p_B_after_stage1 = EstimateObservedMeanBodyLever(
      *s1.trajectory, geo.init.T_LW, ps.sc.gt.t_d_L_s, body_obs,
      temporal_sqrt_info_scales);

  if (!audit.gate_on) {
    return audit;
  }
  const auto iter = RunBodyPathIterativeObservedMean(
      ps, levers, noise, base_cfg, body_obs, observed_mean_iters,
      temporal_sqrt_info_scales);
  audit.obs_trans_mm = iter.final_metrics.trans_mm;
  audit.obs_cost = iter.final_refine_cost;
  audit.trans_ok =
      iter.final_metrics.stage2_ok &&
      iter.final_metrics.trans_mm <= cent.metrics.trans_mm + 1e-3;
  audit.cost_ok = iter.final_metrics.stage2_ok &&
                  iter.final_refine_cost <= cent.refine_cost + 1e-6;
  audit.applied = audit.trans_ok && audit.cost_ok;
  if (!iter.L_B_per_iter.empty()) {
    audit.p_B_final_iter = iter.L_B_per_iter.back();
  }
  return audit;
}

struct GatedObservedMeanCalibResult {
  Phase15TlwMetrics centroid_only;
  Phase15TlwMetrics observed_mean;
  SE3d T_LW_centroid;
  SE3d T_LW_observed_mean;
  TrajectoryAttitudeSpread aspect;
  bool observed_mean_applied = false;
  double centroid_refine_cost = 0.0;
  double observed_refine_cost = 0.0;
};

inline BodyGatedCalibrationInput InputFromPhase15(
    const Phase15Scenario& ps,
    const std::vector<BodyClusterObservation>& body_obs) {
  BodyGatedCalibrationInput in;
  in.rtk = ps.sc.rtk;
  in.attitude = ps.sc.attitude_obs;
  in.tags = ps.sc.tag_obs;
  in.body_obs = body_obs;
  in.nominal_t_d_L_s = ps.sc.gt.t_d_L_s;
  in.gt_T_LW = &ps.sc.gt.T_LW;
  return in;
}

inline GatedObservedMeanCalibResult CalibrateBodyGatedObservedMean(
    const Phase15Scenario& ps, const LeverArmConfig& levers,
    const NoiseModel& noise, const TwoStagePipelineConfig& base_cfg,
    const std::vector<BodyClusterObservation>& body_obs,
    int observed_mean_iters = 3,
    const ObservedMeanAspectGate& gate = {},
    double u_B_azimuth_std_deg = -1.0,
    const std::vector<double>* temporal_sqrt_info_scales = nullptr) {
  const GatedBodyCalibrationResult prod = clic_calib::CalibrateBodyGatedObservedMean(
      InputFromPhase15(ps, body_obs), levers, noise, base_cfg,
      observed_mean_iters, gate, u_B_azimuth_std_deg,
      temporal_sqrt_info_scales);
  GatedObservedMeanCalibResult out;
  out.aspect = prod.aspect;
  out.centroid_only.rot_deg = prod.centroid_only.rot_deg;
  out.centroid_only.trans_mm = prod.centroid_only.trans_mm;
  out.centroid_only.stage2_ok = prod.centroid_only.stage2_ok;
  out.observed_mean.rot_deg = prod.observed_mean.rot_deg;
  out.observed_mean.trans_mm = prod.observed_mean.trans_mm;
  out.observed_mean.stage2_ok = prod.observed_mean.stage2_ok;
  out.T_LW_centroid = prod.T_LW_centroid;
  out.T_LW_observed_mean = prod.T_LW_observed_mean;
  out.observed_mean_applied = prod.observed_mean_applied;
  out.centroid_refine_cost = prod.centroid_refine_cost;
  out.observed_refine_cost = prod.observed_refine_cost;
  return out;
}

inline JointLeverDiagnostic RunBodyPathJointDiagnostic(
    const Phase15Scenario& ps, const LeverArmConfig& levers,
    const NoiseModel& noise, const TwoStagePipelineConfig& base_cfg,
    const std::vector<BodyClusterObservation>& body_obs,
    const Eigen::Vector3d& b_const_gt,
    Phase15TlwMetrics* metrics_out) {
  JointLeverDiagnostic diag;
  diag.L_B_before = levers.L_B_to_body_centroid;

  TwoStagePipelineConfig cfg = base_cfg;
  cfg.init.lidar_target_mode = LidarTargetMode::kBodyCluster;
  cfg.refine.lidar_target_mode = LidarTargetMode::kBodyCluster;

  Stage1TrajectoryResult s1;
  GeometricInitReport geo;
  if (!FitStage1AndGeometricInit(ps, levers, cfg, body_obs, &s1, &geo)) {
    return diag;
  }
  diag.t_LW_before = geo.init.T_LW.translation();

  CoarseExtrinsicInit init_lever = geo.init;
  init_lever.t_d_L_s = ps.sc.gt.t_d_L_s;
  diag.L_B_after = ExtrinsicRefiner::OptimizeJointBodyLeverAtFixedExtrinsic(
      *s1.trajectory, body_obs, init_lever, levers, noise, cfg.refine);

  LeverArmConfig levers_refine = levers;
  levers_refine.L_B_to_body_centroid = diag.L_B_after;
  ExtrinsicInitializerConfig init_cfg = cfg.init;
  init_cfg.lidar_target_mode = LidarTargetMode::kBodyCluster;
  try {
    geo = ExtrinsicInitializer::FromGeometric(
        *s1.trajectory, {}, body_obs, ps.sc.tag_obs, levers_refine, init_cfg);
  } catch (const std::exception&) {
    return diag;
  }

  cfg.refine.body_lever_arm_mode = BodyLeverArmMode::kNominalYaml;
  const ExtrinsicRefinerResult s2 = ExtrinsicRefiner::Refine(
      *s1.trajectory, {}, body_obs, ps.sc.tag_obs, levers_refine, noise,
      geo.init, cfg.refine);

  diag.t_LW_after = s2.lidar.t;
  diag.L_B_delta_mm = (diag.L_B_after - diag.L_B_before).norm() * 1e3;
  diag.t_LW_delta_mm = (diag.t_LW_after - diag.t_LW_before).norm() * 1e3;
  const Eigen::Vector3d L_target = levers.L_B_to_body_centroid + b_const_gt;
  diag.dist_to_L_nom_plus_bconst_mm =
      (diag.L_B_after - L_target).norm() * 1e3;

  if (metrics_out) {
    *metrics_out = TlwMetrics(SE3d(s2.lidar.q, s2.lidar.t), ps.sc.gt.T_LW,
                              s2.converged);
  }
  return diag;
}

}  // namespace experiments
}  // namespace clic_calib
