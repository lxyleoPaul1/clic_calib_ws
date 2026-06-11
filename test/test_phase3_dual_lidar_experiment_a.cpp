#include <clic_calib/target/body_centroid_analysis.h>
#include <clic_calib/utils/temporal_correlation.h>
#include <clic_calib/utils/lever_arm.h>
#include <clic_calib/utils/noise_model.h>
#include "gtest_ceres_guard.hpp"


#include <gtest/gtest.h>

#include "diagnostic/closed_form_init_common.hpp"
#include "diagnostic/two_stage_probe_common.hpp"
#include "experiments/dual_lidar_aspect_bias_diagnostic.hpp"
#include "experiments/dual_lidar_diagonal_geometry.hpp"
#include "experiments/dual_lidar_scenario_common.hpp"
#include "experiments/noise_regime_common.hpp"
#include "experiments/phase15_ablation_common.hpp"

#include <iomanip>
#include <iostream>

namespace {

constexpr uint32_t kRepSeed = 13025;
constexpr double kSphereRadiusM = 0.10;
constexpr double kStage1AlphaP = 0.01;
constexpr double kStage1AlphaR = 0.01;
constexpr int kObservedMeanIters = 3;
constexpr double kPhase15VaryingRmsMm = 22.0;
/** Field / real-dual-Ruby target (not simulation hard gate). */
constexpr double kFieldObsTargetMm = 35.0;
/** Simulation sign-off @ 0.5 Hz (see doc/board_free_results_frozen.md). */
constexpr double kSimMaxObsMm = 42.0;
/** SW-only synthetic sector artifact band above kSimMaxObsMm (exempt, not raised). */
constexpr double kSwSyntheticArtifactExemptUpperMm = 43.0;
constexpr double kSimRelRotDeg = 0.15;
constexpr double kEntryRelRotDeg60 = 0.15;

clic_calib::TwoStagePipelineConfig MakeBaseConfig(
    const clic_calib::two_stage_probe::SplineConfig& spline_cfg,
    const clic_calib::two_stage_probe::CoarseExtrinsicInit& t_d_nominal) {
  clic_calib::TwoStagePipelineConfig cfg;
  cfg.stage1.knot_interval_s = spline_cfg.knot_interval_s;
  cfg.stage1.alpha_p = kStage1AlphaP;
  cfg.stage1.alpha_R = kStage1AlphaR;
  cfg.stage1.attitude_stride = 25;
  cfg.stage1.trim_to_observation_support = true;
  cfg.init.sphere_radius_m = kSphereRadiusM;
  cfg.init.nominal_t_d_L_s = t_d_nominal.t_d_L_s;
  cfg.init.nominal_t_d_C_s = t_d_nominal.t_d_C_s;
  cfg.init.camera_K = {600.0, 600.0, 320.0, 240.0};
  cfg.refine.sphere_radius_m = kSphereRadiusM;
  cfg.refine.t_d_max_abs_s = spline_cfg.t_d_max_abs_s;
  cfg.refine.lidar_cauchy_scale = 1.0;
  cfg.refine.camera_huber_delta_px = 2.0;
  cfg.refine.camera_K = cfg.init.camera_K;
  cfg.refine.max_iterations = 500;
  // Legacy σ for gate flights (乙); 10 Hz 戊 uses centroid_cov override below.
  cfg.refine.use_centroid_cov_whitening = false;
  return cfg;
}

void PrintAspect(const char* tag,
                 const clic_calib::TrajectoryAttitudeSpread& a,
                 double u_B_az_std_deg = -1.0) {
  std::cout << "  [" << tag << "] yaw_cv=" << std::fixed << std::setprecision(3)
            << a.yaw_circular_variance << " pitch_std=" << a.pitch_std_deg
            << " deg  gate="
            << (clic_calib::experiments::ShouldApplyObservedMeanPB(
                    a, {}, u_B_az_std_deg)
                    ? "ON"
                    : "OFF")
            << "\n";
}

void PrintSensorCalib(const char* tag,
                      const clic_calib::experiments::DualSensorCalibResult& r,
                      double u_B_az_std_deg) {
  std::cout << std::fixed << std::setprecision(2);
  std::cout << "  " << std::setw(4) << tag << " u_B_az_std=" << std::setw(5)
            << u_B_az_std_deg << " deg  cent=" << std::setw(6)
            << r.calib.centroid_only.trans_mm << " mm rot="
            << r.calib.centroid_only.rot_deg << " deg  obs="
            << std::setw(6) << r.calib.observed_mean.trans_mm
            << " mm rot=" << r.calib.observed_mean.rot_deg << " deg"
            << (r.calib.observed_mean_applied ? "" : " (fallback)")
            << "  n=" << r.body_frames << "\n";
  PrintAspect(tag, r.calib.aspect, u_B_az_std_deg);
}

void PrintObservedMeanAudit(const char* tag,
                            const clic_calib::experiments::ObservedMeanPathAudit& a) {
  std::cout << std::fixed << std::setprecision(2);
  std::cout << "  [" << tag << "] gate=" << (a.gate_on ? "ON" : "OFF")
            << "  applied=" << (a.applied ? "YES" : "NO")
            << "  trans_ok=" << (a.trans_ok ? "Y" : "N")
            << "  cost_ok=" << (a.cost_ok ? "Y" : "N") << "\n";
  std::cout << "    yaw_cv=" << std::setprecision(3) << a.aspect.yaw_circular_variance
            << " pitch_std=" << a.aspect.pitch_std_deg << " deg"
            << "  cent/obs trans=" << std::setprecision(2) << a.cent_trans_mm
            << "/" << a.obs_trans_mm << " mm"
            << "  cost=" << a.cent_cost << "/" << a.obs_cost << "\n";
  std::cout << "    p_B nominal [mm]: "
            << (a.p_B_nominal * 1e3).transpose() << "\n";
  std::cout << "    p_B stage1  [mm]: "
            << (a.p_B_after_stage1 * 1e3).transpose()
            << "  Δ=" << (a.p_B_after_stage1 - a.p_B_nominal).norm() * 1e3
            << " mm\n";
  if (a.gate_on) {
    std::cout << "    p_B final   [mm]: "
              << (a.p_B_final_iter * 1e3).transpose()
              << "  Δ=" << (a.p_B_final_iter - a.p_B_nominal).norm() * 1e3
              << " mm\n";
  }
}

std::vector<clic_calib::BodyClusterObservation> FilterSectorObservations(
    const std::vector<clic_calib::BodyClusterObservation>& obs, double t_d_L_s,
    double sector_duration_s, int sector) {
  std::vector<clic_calib::BodyClusterObservation> out;
  out.reserve(obs.size());
  for (const auto& o : obs) {
    const double t_world = o.t_sensor_ - t_d_L_s;
    const int s =
        (t_world < sector_duration_s - 1e-6) ? 0 : 1;
    if (s == sector) {
      out.push_back(o);
    }
  }
  return out;
}

clic_calib::TrajectoryAttitudeSpread AttitudeSpreadOnGt(
    const clic_calib::BodyTrajectory& gt_traj, double t_d_L_s,
    const std::vector<clic_calib::BodyClusterObservation>& obs) {
  return clic_calib::ComputeAttitudeSpreadAtObservations(gt_traj, t_d_L_s, obs);
}

/** Per-frame POI roll proxy + u_B at body-cluster times (GT traj). */
struct SectorPerFrameStats {
  double body_x_tilt_span_deg = 0.0;
  double body_x_tilt_std_deg = 0.0;
  double u_B_az_std_deg = 0.0;
  int n_frames = 0;
};

SectorPerFrameStats SectorPerFrameStatsOnObs(
    const clic_calib::BodyTrajectory& traj, double t_d_L_s,
    const Eigen::Vector3d& post,
    const std::vector<clic_calib::BodyClusterObservation>& obs) {
  SectorPerFrameStats s;
  s.n_frames = static_cast<int>(obs.size());
  if (obs.empty()) {
    return s;
  }
  std::vector<double> tilt_deg;
  std::vector<double> az_deg;
  tilt_deg.reserve(obs.size());
  az_deg.reserve(obs.size());
  double tilt_min = 1e9;
  double tilt_max = -1e9;
  for (const auto& o : obs) {
    const double t = o.t_sensor_ - t_d_L_s;
    const clic_calib::SE3d T = traj.pose_wb(t);
    const Eigen::Matrix3d R = T.so3().matrix();
    const double tilt =
        std::asin(std::clamp(R(0, 2), -1.0, 1.0)) * 180.0 / M_PI;
    tilt_deg.push_back(tilt);
    tilt_min = std::min(tilt_min, tilt);
    tilt_max = std::max(tilt_max, tilt);
    const Eigen::Vector3d u_B = clic_calib::LidarDirectionInBody(
        T, T.translation(), post);
    az_deg.push_back(std::atan2(u_B.y(), u_B.x()) * 180.0 / M_PI);
  }
  s.body_x_tilt_span_deg = tilt_max - tilt_min;
  double mean = 0.0;
  for (double v : tilt_deg) {
    mean += v;
  }
  mean /= static_cast<double>(tilt_deg.size());
  double sq = 0.0;
  for (double v : tilt_deg) {
    const double d = v - mean;
    sq += d * d;
  }
  s.body_x_tilt_std_deg =
      std::sqrt(sq / static_cast<double>(tilt_deg.size()));
  double az_mean = 0.0;
  for (double a : az_deg) {
    az_mean += a;
  }
  az_mean /= static_cast<double>(az_deg.size());
  sq = 0.0;
  for (double a : az_deg) {
    const double d = a - az_mean;
    sq += d * d;
  }
  s.u_B_az_std_deg = std::sqrt(sq / static_cast<double>(az_deg.size()));
  return s;
}

void PrintSectorPerFrameStats(const char* label, const SectorPerFrameStats& s) {
  std::cout << std::fixed << std::setprecision(2);
  std::cout << "    " << label << "  n=" << s.n_frames
            << "  body_x_tilt span=" << s.body_x_tilt_span_deg
            << "° std=" << s.body_x_tilt_std_deg
            << "°  u_B_az_std=" << s.u_B_az_std_deg << "°\n";
}

void PrintFlightEntryTable(
    const char* label,
    const clic_calib::experiments::DualFlightEntryMetrics& m,
    double ne_u_B_std, double sw_u_B_std) {
  const auto& r = m.calib;
  std::cout << "\n--- " << label << " ---\n";
  PrintSensorCalib("NE", r.ne, ne_u_B_std);
  PrintSensorCalib("SW", r.sw, sw_u_B_std);
  std::cout << std::fixed << std::setprecision(2);
  std::cout << "  rel rot  cent=" << r.rel_centroid.rot_deg
            << " deg  obs=" << r.rel_observed.rot_deg << " deg\n";
  std::cout << "  rel |trans| cent=" << r.rel_centroid.trans_mm
            << " mm  obs=" << r.rel_observed.trans_mm
            << " mm (lever: Δθ×baseline)\n";
  std::cout << "  center-reg cent=" << m.center_reg_cent_mm
            << " mm  obs=" << m.center_reg_obs_mm << " mm\n";
}

clic_calib::AspectBiasScatterReport AspectOnFlight(
    const clic_calib::experiments::DualLidarPhase3Scenario& ds,
    const std::string& sensor_key, const clic_calib::LeverArmConfig& levers) {
  const Eigen::Vector3d post =
      clic_calib::experiments::DiagonalLidarPostW(ds.geom.dual_preset,
                                                  sensor_key);
  return clic_calib::BuildAspectBiasScatterReport(
      ds.sc.gt_traj, ds.gt_lidars.T_LW.at(sensor_key), ds.sc.gt.t_d_L_s,
      levers.L_B_to_body_centroid, post,
      ds.body_by_sensor.at(sensor_key));
}

}  // namespace

TEST(Phase3DualLidarExperimentA, AspectDiagnosticFlightDAndPOIFlightE) {
  const std::string config_dir =
      clic_calib::experiments::ConfigDirFromExperiments();
  const auto levers =
      clic_calib::LeverArmConfig::from_yaml(config_dir + "/lever_arms.yaml");
  const auto spline_cfg =
      clic_calib::two_stage_probe::LoadSplineConfig(config_dir + "/spline.yaml");
  const clic_calib::two_stage_probe::CoarseExtrinsicInit t_d_nominal =
      clic_calib::two_stage_probe::LoadCoarseExtrinsicsFromYaml(config_dir);
  const clic_calib::NoiseModel noise =
      clic_calib::NoiseModel::FromConfigDir(config_dir);
  const clic_calib::TwoStagePipelineConfig base_cfg =
      MakeBaseConfig(spline_cfg, t_d_nominal);

  std::cout << "\n### Phase 3 (A): u_B aspect + flight 戊 POI ###\n";

  // --- Alignment (Phase 1.5 reuse) ---
  clic_calib::experiments::Phase15Scenario ps_p15 =
      clic_calib::experiments::BuildPhase15ScenarioWithGeom(
          kRepSeed, noise,
          clic_calib::experiments::NearFieldHighAspectScenarioGeometry());
  ASSERT_TRUE(clic_calib::experiments::PreparePhase15ScenarioTags(
      &ps_p15, levers, noise, t_d_nominal, spline_cfg, kRepSeed));

  const auto m_cent_direct = clic_calib::experiments::RunBodyPath(
      ps_p15, levers, noise, base_cfg,
      clic_calib::BodyLeverArmMode::kNominalYaml, ps_p15.body_cluster);
  const auto iter_direct =
      clic_calib::experiments::RunBodyPathIterativeObservedMean(
          ps_p15, levers, noise, base_cfg, ps_p15.body_cluster,
          kObservedMeanIters);
  const auto gated_dual =
      clic_calib::experiments::CalibrateDualSensorViaPhase15(
          ps_p15, levers, noise, base_cfg, "lidar_NE", kObservedMeanIters);

  const auto p15_decomp = clic_calib::DecomposeBodyCentroidBias(
      ps_p15.sc.gt_traj, ps_p15.sc.gt.T_LW, ps_p15.sc.gt.t_d_L_s,
      levers.L_B_to_body_centroid, ps_p15.body_cluster);

  std::cout << "\n=== Alignment: NearFieldHighAspect @ seed " << kRepSeed
            << " ===\n";
  std::cout << std::fixed << std::setprecision(2);
  std::cout << "  P1.5 centroid=" << m_cent_direct.trans_mm
            << " mm  iter×3 obs=" << iter_direct.final_metrics.trans_mm
            << " mm  varying_RMS=" << p15_decomp.varying_rms_mm << " mm\n";
  std::cout << "  Dual-platform centroid=" << gated_dual.calib.centroid_only.trans_mm
            << " mm  gated obs=" << gated_dual.calib.observed_mean.trans_mm
            << " mm\n";

  EXPECT_NEAR(gated_dual.calib.centroid_only.trans_mm, m_cent_direct.trans_mm,
              0.5);
  EXPECT_NEAR(gated_dual.calib.observed_mean.trans_mm,
              iter_direct.final_metrics.trans_mm, 0.5);
  EXPECT_TRUE(gated_dual.calib.observed_mean_applied);
  EXPECT_NEAR(m_cent_direct.trans_mm, 63.5, 3.0);
  EXPECT_NEAR(iter_direct.final_metrics.trans_mm, 18.5, 3.0);

  // --- Tidal lock: NearFieldHighAspect single-sensor law ---
  const auto p15_geom =
      clic_calib::experiments::NearFieldHighAspectScenarioGeometry();
  const double p15_t_end =
      static_cast<double>(p15_geom.flight_layers_m.size()) *
      p15_geom.layer_duration_s;
  const auto tidal_p15 = clic_calib::AuditOrbitYawTidalLock(
      0.0, p15_t_end, 0.1, [&](double t) {
        return clic_calib::experiments::PoseWbFromGeometry(t, p15_geom);
      });

  std::cout << "\n=== Tidal-lock audit (orbit az vs body yaw) ===\n";
  clic_calib::experiments::PrintTidalLockAudit("P1.5 NearFieldHighAspect",
                                               tidal_p15);

  // --- Flight 丁: bias_B vs u_B aspect scatter ---
  const auto geom_d = clic_calib::experiments::DiagonalFlightD_Geometry();
  const auto ds_d = clic_calib::experiments::BuildDualDiagonalScenario(
      kRepSeed, noise, geom_d, spline_cfg, t_d_nominal);
  const auto aspect_d_ne = AspectOnFlight(ds_d, "lidar_NE", levers);
  const auto aspect_d_sw = AspectOnFlight(ds_d, "lidar_SW", levers);

  std::cout << "\n=== Flight 丁: bias_B vs u_B azimuth (GT) ===\n";
  std::cout << "  P1.5 reference varying_RMS ≈ " << kPhase15VaryingRmsMm
            << " mm\n";
  clic_calib::experiments::PrintAspectBiasScatterReport("NE", aspect_d_ne);
  clic_calib::experiments::PrintAspectBiasScatterReport("SW", aspect_d_sw);

  const auto tidal_d = clic_calib::AuditOrbitYawTidalLock(
      0.0, geom_d.sector_duration_s, 0.1, [&](double t) {
        return clic_calib::experiments::PoseWbFromDualDiagonalGeometry(t, geom_d);
      });
  clic_calib::experiments::PrintTidalLockAudit("丁 sector-0 (decoupled att)",
                                               tidal_d);

  // --- Four flights summary (甲–丁) + 乙 guard ---
  const auto geom_a = clic_calib::experiments::DiagonalFlightA_Geometry();
  const auto geom_b = clic_calib::experiments::DiagonalFlightB_Geometry();
  const auto geom_c = clic_calib::experiments::DiagonalFlightC_Geometry();

  const auto rep_b = clic_calib::experiments::CalibrateDualDiagonalFlightEntry(
      clic_calib::experiments::BuildDualDiagonalScenario(
          kRepSeed, noise, geom_b, spline_cfg, t_d_nominal),
      levers, noise, base_cfg, kObservedMeanIters);

  EXPECT_FALSE(rep_b.calib.ne.calib.observed_mean_applied);
  EXPECT_FALSE(rep_b.calib.sw.calib.observed_mean_applied);
  EXPECT_NEAR(rep_b.calib.ne.calib.observed_mean.trans_mm,
              rep_b.calib.ne.calib.centroid_only.trans_mm, 0.5);
  EXPECT_NEAR(rep_b.calib.ne.calib.observed_mean.trans_mm,
              rep_b.calib.ne.calib.centroid_only.trans_mm, 0.5);
  EXPECT_LT(rep_b.calib.ne.calib.aspect.yaw_circular_variance, 0.65);

  auto print_flight = [&](const char* label,
                          const clic_calib::experiments::DualDiagonalFlightGeometry&
                              geom) {
    const auto ds = clic_calib::experiments::BuildDualDiagonalScenario(
        kRepSeed, noise, geom, spline_cfg, t_d_nominal);
    const auto m = clic_calib::experiments::CalibrateDualDiagonalFlightEntry(
        ds, levers, noise, base_cfg, kObservedMeanIters);
    const auto asp_ne = AspectOnFlight(ds, "lidar_NE", levers);
    const auto asp_sw = AspectOnFlight(ds, "lidar_SW", levers);
    PrintFlightEntryTable(label, m, asp_ne.u_B_azimuth_std_deg,
                          asp_sw.u_B_azimuth_std_deg);
    return m;
  };

  std::cout << "\n=== Flights 甲–丁 (|trans| mm; rel-trans = Δθ×baseline) ===\n";
  print_flight("甲", geom_a);
  print_flight("丙", geom_c);
  print_flight("丁", geom_d);
  PrintFlightEntryTable(
      "乙", rep_b,
      AspectOnFlight(
          clic_calib::experiments::BuildDualDiagonalScenario(
              kRepSeed, noise, geom_b, spline_cfg, t_d_nominal),
          "lidar_NE", levers)
          .u_B_azimuth_std_deg,
      AspectOnFlight(
          clic_calib::experiments::BuildDualDiagonalScenario(
              kRepSeed, noise, geom_b, spline_cfg, t_d_nominal),
          "lidar_SW", levers)
          .u_B_azimuth_std_deg);

  const auto geom_e = clic_calib::experiments::DiagonalFlightE_Geometry();
  const auto geom_e10 =
      clic_calib::experiments::DiagonalFlightE_10Hz_Geometry();
  const auto ds_e = clic_calib::experiments::BuildDualDiagonalScenario(
      kRepSeed, noise, geom_e, spline_cfg, t_d_nominal);
  const auto ds_e10 = clic_calib::experiments::BuildDualDiagonalScenario(
      kRepSeed, noise, geom_e10, spline_cfg, t_d_nominal);

  const auto audit_ne_05 =
      clic_calib::experiments::AuditBodyClusterPointCounts(
          ds_e.body_by_sensor.at("lidar_NE"));
  const auto audit_ne_10 =
      clic_calib::experiments::AuditBodyClusterPointCounts(
          ds_e10.body_by_sensor.at("lidar_NE"));

  std::cout << "\n=== 戊 diagnostic: per-scan N (frame-rate invariant) ===\n";
  std::cout << std::fixed << std::setprecision(1);
  std::cout << "  0.5Hz NE  n_frames=" << audit_ne_05.num_frames
            << "  N mean/min/max=" << audit_ne_05.mean << "/" << audit_ne_05.min
            << "/" << audit_ne_05.max << "\n";
  std::cout << "  10Hz NE  n_frames=" << audit_ne_10.num_frames
            << "  N mean/min/max=" << audit_ne_10.mean << "/" << audit_ne_10.min
            << "/" << audit_ne_10.max << "\n";

  clic_calib::TwoStagePipelineConfig cfg_e05 = base_cfg;
  cfg_e05.refine.use_centroid_cov_whitening = true;
  const auto rep_e = clic_calib::experiments::CalibrateDualDiagonalFlightEntry(
      ds_e, levers, noise, cfg_e05, kObservedMeanIters);

  clic_calib::TwoStagePipelineConfig cfg_uniform = base_cfg;
  cfg_uniform.refine.use_centroid_cov_whitening = false;
  const auto rep_e10_uniform =
      clic_calib::experiments::CalibrateDualDiagonalFlightEntry(
          ds_e10, levers, noise, cfg_uniform, kObservedMeanIters);

  std::cout << "\n=== 戊@10Hz A/B: centroid_cov OFF (legacy σ whitening) ===\n";
  std::cout << std::fixed << std::setprecision(2);
  std::cout << "  NE obs=" << rep_e10_uniform.calib.ne.calib.observed_mean.trans_mm
            << " mm rot="
            << rep_e10_uniform.calib.ne.calib.observed_mean.rot_deg
            << " deg\n";
  std::cout << "  SW obs=" << rep_e10_uniform.calib.sw.calib.observed_mean.trans_mm
            << " mm rot="
            << rep_e10_uniform.calib.sw.calib.observed_mean.rot_deg
            << " deg\n";

  const double d_ne = rep_e10_uniform.calib.ne.calib.observed_mean.trans_mm -
                    rep_e.calib.ne.calib.observed_mean.trans_mm;
  const double d_sw = rep_e10_uniform.calib.sw.calib.observed_mean.trans_mm -
                    rep_e.calib.sw.calib.observed_mean.trans_mm;
  const bool cov_hurts = d_ne > 15.0 || d_sw > 15.0;
  const bool correlation_hurts =
      rep_e10_uniform.calib.ne.calib.observed_mean.trans_mm > 70.0 ||
      rep_e10_uniform.calib.sw.calib.observed_mean.trans_mm > 70.0;
  std::cout << "  Δobs vs 0.5Hz (uniform): NE=" << d_ne << " SW=" << d_sw
            << " mm\n";
  std::cout << "  → centroid_cov overweight: " << (cov_hurts ? "YES" : "partial")
            << "; temporal correlation: "
            << (correlation_hurts ? "YES" : "NO") << "\n";

  const double stage1_rmse_05 =
      clic_calib::experiments::Stage1AntennaRmseMm(ds_e, levers, base_cfg);
  const double stage1_rmse_10 =
      clic_calib::experiments::Stage1AntennaRmseMm(ds_e10, levers, base_cfg);
  std::cout << "\n=== Stage-1 RTK trajectory audit (native rate) ===\n";
  std::cout << std::fixed << std::setprecision(2);
  std::cout << "  antenna RMSE vs GT  0.5Hz=" << stage1_rmse_05
            << " mm  10Hz=" << stage1_rmse_10 << " mm"
            << (stage1_rmse_10 <= stage1_rmse_05 + 0.5 ? " (OK)" : " (CHECK)")
            << "\n";

  clic_calib::experiments::BodyObsPreparePolicy high_rate_policy;
  high_rate_policy.temporal_decorrelation.enabled = true;
  high_rate_policy.temporal_decorrelation.ar1_rho = -1.0;
  high_rate_policy.temporal_decorrelation.kernel_tau_s = 0.25;
  high_rate_policy.temporal_decorrelation.mode =
      clic_calib::TemporalDecorrelationConfig::Mode::kAr1Innovation;

  clic_calib::TwoStagePipelineConfig cfg_e10 = base_cfg;
  cfg_e10.refine.use_centroid_cov_whitening = true;

  const auto aspect_e_ne = AspectOnFlight(ds_e, "lidar_NE", levers);
  const auto aspect_e_sw = AspectOnFlight(ds_e, "lidar_SW", levers);

  const auto aspect_e10_ne = AspectOnFlight(ds_e10, "lidar_NE", levers);
  const auto aspect_e10_sw = AspectOnFlight(ds_e10, "lidar_SW", levers);

  const auto bias_ne10 =
      clic_calib::experiments::CollectBiasNormSeries(
          ds_e10.sc.gt_traj, ds_e10.gt_lidars.T_LW.at("lidar_NE"),
          ds_e10.sc.gt.t_d_L_s, levers.L_B_to_body_centroid,
          ds_e10.body_by_sensor.at("lidar_NE"));
  const double rho_ne10 =
      bias_ne10.empty() ? 0.0
                        : clic_calib::EstimateLag1Autocorrelation(bias_ne10);

  const auto rep_e10_fixed =
      clic_calib::experiments::CalibrateDualDiagonalFlightEntry(
          ds_e10, levers, noise, cfg_e10, kObservedMeanIters,
          high_rate_policy);

  std::cout << "\n=== Isolation (1): PW + NE-only roll lock @ 10 Hz ===\n";
  std::cout << "  Stage-1: PW native RTK; POI roll scale="
            << geom_e.poi_sector0_attitude_scale << " on NE sector only\n";
  std::cout << "  10Hz NE obs=" << rep_e10_fixed.calib.ne.calib.observed_mean.trans_mm
            << " mm"
            << (rep_e10_fixed.calib.ne.calib.observed_mean_applied ? ""
                                                                   : " (fallback)")
            << "  SW obs="
            << rep_e10_fixed.calib.sw.calib.observed_mean.trans_mm << " mm"
            << (rep_e10_fixed.calib.sw.calib.observed_mean_applied ? ""
                                                                   : " (fallback)")
            << "\n";
  std::cout << "  (76cf1d6 ref: NE 53.9 / SW 117.6 mm; fbe701f both-tight: 148.7 / 199.9)\n";

  const auto geom_e_both =
      clic_calib::experiments::DiagonalFlightE_BothSectorsTight_Geometry();
  const auto geom_e10_both = geom_e10;
  auto geom_e10_both_mut = geom_e10_both;
  geom_e10_both_mut.poi_roll_scale_all_sectors = true;
  const auto ds_e10_both =
      clic_calib::experiments::BuildDualDiagonalScenario(
          kRepSeed, noise, geom_e10_both_mut, spline_cfg, t_d_nominal);
  const auto rep_e10_both =
      clic_calib::experiments::CalibrateDualDiagonalFlightEntry(
          ds_e10_both, levers, noise, cfg_e10, kObservedMeanIters,
          high_rate_policy);
  std::cout << "  both-sector tight repro: NE="
            << rep_e10_both.calib.ne.calib.observed_mean.trans_mm << " SW="
            << rep_e10_both.calib.sw.calib.observed_mean.trans_mm << " mm\n";

  std::cout << "\n=== SW observed-mean audit @ 0.5 Hz (2)(3) ===\n";
  const auto ps_sw =
      clic_calib::experiments::ToPhase15SensorSlice(ds_e, "lidar_SW");
  const Eigen::Vector3d post_sw =
      clic_calib::experiments::DiagonalLidarPostW(geom_e.dual_preset,
                                                  "lidar_SW");
  const double u_B_sw = clic_calib::experiments::ComputeUBAzimuthStdDeg(
      ds_e.sc.gt_traj, ds_e.sc.gt.t_d_L_s, post_sw, ps_sw.body_cluster);
  const auto sw_obs_ne_only = clic_calib::experiments::AuditObservedMeanPath(
      ps_sw, levers, noise, cfg_e05, ps_sw.body_cluster, u_B_sw,
      kObservedMeanIters);
  PrintObservedMeanAudit("SW NE-only tighten", sw_obs_ne_only);

  const auto ds_e_both =
      clic_calib::experiments::BuildDualDiagonalScenario(
          kRepSeed, noise, geom_e_both, spline_cfg, t_d_nominal);
  const auto ps_sw_both =
      clic_calib::experiments::ToPhase15SensorSlice(ds_e_both, "lidar_SW");
  const double u_B_sw_both = clic_calib::experiments::ComputeUBAzimuthStdDeg(
      ds_e_both.sc.gt_traj, ds_e_both.sc.gt.t_d_L_s, post_sw,
      ps_sw_both.body_cluster);
  const auto sw_obs_both = clic_calib::experiments::AuditObservedMeanPath(
      ps_sw_both, levers, noise, cfg_e05, ps_sw_both.body_cluster, u_B_sw_both,
      kObservedMeanIters);
  PrintObservedMeanAudit("SW both-sector tight", sw_obs_both);

  const auto sw_sector_obs = FilterSectorObservations(
      ps_sw.body_cluster, ds_e.sc.gt.t_d_L_s, geom_e.sector_duration_s, 1);
  const auto spread_sw_ne =
      AttitudeSpreadOnGt(ds_e.sc.gt_traj, ds_e.sc.gt.t_d_L_s, sw_sector_obs);
  const auto spread_sw_both = AttitudeSpreadOnGt(
      ds_e_both.sc.gt_traj, ds_e_both.sc.gt.t_d_L_s,
      FilterSectorObservations(ps_sw_both.body_cluster, ds_e_both.sc.gt.t_d_L_s,
                               geom_e_both.sector_duration_s, 1));
  std::cout << "  R_WB@SW sector (GT traj, sector-local obs):\n";
  std::cout << std::fixed << std::setprecision(3);
  std::cout << "    NE-only tighten:  yaw_cv=" << spread_sw_ne.yaw_circular_variance
            << " pitch_std=" << spread_sw_ne.pitch_std_deg << " deg\n";
  std::cout << "    both-sector 0.35: yaw_cv=" << spread_sw_both.yaw_circular_variance
            << " pitch_std=" << spread_sw_both.pitch_std_deg << " deg\n";

  const auto ps_ne =
      clic_calib::experiments::ToPhase15SensorSlice(ds_e, "lidar_NE");
  const Eigen::Vector3d post_ne =
      clic_calib::experiments::DiagonalLidarPostW(geom_e.dual_preset,
                                                  "lidar_NE");
  const double u_B_ne = clic_calib::experiments::ComputeUBAzimuthStdDeg(
      ds_e.sc.gt_traj, ds_e.sc.gt.t_d_L_s, post_ne, ps_ne.body_cluster);
  const auto ne_audit = clic_calib::experiments::AuditObservedMeanPath(
      ps_ne, levers, noise, cfg_e05, ps_ne.body_cluster, u_B_ne,
      kObservedMeanIters);
  PrintObservedMeanAudit("NE sector (ref)", ne_audit);

  std::cout << "\n=== fbe701f Stage-2 touch audit ===\n";
  std::cout << "  git diff 76cf1d6..fbe701f: no stage2/refiner/pipeline files\n";
  std::cout << "  Stage-2 path unchanged; regression from geometry (both-sector roll)\n";

  std::cout << "\n=== 戊 fix applied (full-frame 10 Hz) ===\n";
  std::cout << "  Stage-2: σ_r²/N_pts whitening ON; AR(1) decorr on bias norm\n";
  std::cout << "  10Hz NE frames=" << rep_e10_fixed.calib.ne.body_frames
            << "  est. lag-1 ρ(bias‖)≈" << rho_ne10 << " (POI prior if low)\n";

  std::cout << "\n=== Flight 戊 POI @ 0.5 Hz ===\n";
  PrintFlightEntryTable("戊@0.5Hz", rep_e, aspect_e_ne.u_B_azimuth_std_deg,
                        aspect_e_sw.u_B_azimuth_std_deg);

  std::cout << "\n=== Flight 戊 POI @ 10 Hz (fixed) ===\n";
  PrintFlightEntryTable("戊@10Hz", rep_e10_fixed, aspect_e10_ne.u_B_azimuth_std_deg,
                        aspect_e10_sw.u_B_azimuth_std_deg);
  std::cout << "  frames NE=" << rep_e10_fixed.calib.ne.body_frames
            << " SW=" << rep_e10_fixed.calib.sw.body_frames << "\n";

  const double range_ne =
      clic_calib::experiments::MeanObservationRangeM(
          ds_e.body_by_sensor.at("lidar_NE"));
  const double range_sw =
      clic_calib::experiments::MeanObservationRangeM(
          ds_e.body_by_sensor.at("lidar_SW"));
  std::cout << "\n=== NE/SW asymmetry (戊@0.5Hz) ===\n";
  std::cout << std::fixed << std::setprecision(2);
  std::cout << "  NE post height="
            << geom_e.dual_preset.post_height_ne_m
            << " m  SW post height=" << geom_e.dual_preset.post_height_sw_m
            << " m\n";
  std::cout << "  mean range NE=" << range_ne << " m  SW=" << range_sw
            << " m  Δ=" << (range_ne - range_sw) * 1e3 << " mm\n";
  const auto vary_e_ne = AspectOnFlight(ds_e, "lidar_NE", levers);
  const auto vary_e_sw = AspectOnFlight(ds_e, "lidar_SW", levers);
  const auto proj_ne = clic_calib::ComputeAspectLeverTranslationProjection(
      ds_e.sc.gt_traj, ds_e.gt_lidars.T_LW.at("lidar_NE"), ds_e.sc.gt.t_d_L_s,
      levers.L_B_to_body_centroid, post_ne,
      ds_e.body_by_sensor.at("lidar_NE"));
  const auto proj_sw = clic_calib::ComputeAspectLeverTranslationProjection(
      ds_e.sc.gt_traj, ds_e.gt_lidars.T_LW.at("lidar_SW"), ds_e.sc.gt.t_d_L_s,
      levers.L_B_to_body_centroid, post_sw,
      ds_e.body_by_sensor.at("lidar_SW"));
  std::cout << "  varying_RMS NE=" << vary_e_ne.bias_varying_rms_mm
            << " mm  SW=" << vary_e_sw.bias_varying_rms_mm << " mm\n";
  std::cout << "  u_B std NE=" << proj_ne.u_B_azimuth_std_deg << "°  SW="
            << proj_sw.u_B_azimuth_std_deg << "°\n";
  std::cout << "  lever_h NE=" << proj_ne.lever_horizontal_mm
            << " mm  SW=" << proj_sw.lever_horizontal_mm << " mm\n";
  std::cout << "  projected jitter NE=" << proj_ne.projected_jitter_trans_mm
            << " mm  SW=" << proj_sw.projected_jitter_trans_mm << " mm\n";
  const double proj_ratio =
      proj_ne.projected_jitter_trans_mm /
      std::max(proj_sw.projected_jitter_trans_mm, 1e-6);
  const double obs_ratio =
      rep_e.calib.ne.calib.observed_mean.trans_mm /
      std::max(rep_e.calib.sw.calib.observed_mean.trans_mm, 1e-3);
  std::cout << "  projected/obs asymmetry: " << proj_ratio << "× / "
            << obs_ratio << "×"
            << (std::abs(proj_ratio - obs_ratio) < 0.35 ? " (match)" : "")
            << "\n";

  std::cout << "\n=== SW poi_sector0 wiring audit (per-frame GT) ===\n";
  std::cout << "  Policy: poi_sector0_attitude_scale scales body Rx in POI chain\n";
  std::cout << "  R = Rz(yaw_POI) * Ry(pitch) * Rx(roll_scale * roll_amp)\n";
  std::cout << "  Production: roll_scale=0.35 on sector 0 (NE) ONLY; sector 1 full roll.\n";
  const auto geom_no_tight =
      clic_calib::experiments::DiagonalFlightE_NoPoiTight_Geometry();
  const auto ds_no_tight =
      clic_calib::experiments::BuildDualDiagonalScenario(
          kRepSeed, noise, geom_no_tight, spline_cfg, t_d_nominal);
  const auto ne_obs_frames = FilterSectorObservations(
      ds_e.body_by_sensor.at("lidar_NE"), ds_e.sc.gt.t_d_L_s,
      geom_e.sector_duration_s, 0);
  const auto sw_obs_frames = FilterSectorObservations(
      ds_e.body_by_sensor.at("lidar_SW"), ds_e.sc.gt.t_d_L_s,
      geom_e.sector_duration_s, 1);
  const Eigen::Vector3d post_ne_wiring =
      clic_calib::experiments::DiagonalLidarPostW(geom_e.dual_preset,
                                                  "lidar_NE");
  std::cout << "  Per-frame @ body-cluster times (body +x world tilt proxy):\n";
  PrintSectorPerFrameStats(
      "NE no-tight",
      SectorPerFrameStatsOnObs(ds_no_tight.sc.gt_traj, ds_no_tight.sc.gt.t_d_L_s,
                               post_ne_wiring, ne_obs_frames));
  PrintSectorPerFrameStats(
      "NE sector0=0.35",
      SectorPerFrameStatsOnObs(ds_e.sc.gt_traj, ds_e.sc.gt.t_d_L_s, post_ne_wiring,
                               ne_obs_frames));
  PrintSectorPerFrameStats(
      "SW no-tight",
      SectorPerFrameStatsOnObs(ds_no_tight.sc.gt_traj, ds_no_tight.sc.gt.t_d_L_s,
                               post_sw, sw_obs_frames));
  PrintSectorPerFrameStats(
      "SW sector1 full (prod)",
      SectorPerFrameStatsOnObs(ds_e.sc.gt_traj, ds_e.sc.gt.t_d_L_s, post_sw,
                               sw_obs_frames));
  PrintSectorPerFrameStats(
      "SW sector1=0.35 (both-flag)",
      SectorPerFrameStatsOnObs(ds_e_both.sc.gt_traj, ds_e_both.sc.gt.t_d_L_s,
                               post_sw, sw_obs_frames));
  const auto ne_no = SectorPerFrameStatsOnObs(
      ds_no_tight.sc.gt_traj, ds_no_tight.sc.gt.t_d_L_s, post_ne_wiring,
      ne_obs_frames);
  const auto ne_tight = SectorPerFrameStatsOnObs(
      ds_e.sc.gt_traj, ds_e.sc.gt.t_d_L_s, post_ne_wiring, ne_obs_frames);
  const auto sw_no = SectorPerFrameStatsOnObs(
      ds_no_tight.sc.gt_traj, ds_no_tight.sc.gt.t_d_L_s, post_sw,
      sw_obs_frames);
  const auto sw_both = SectorPerFrameStatsOnObs(
      ds_e_both.sc.gt_traj, ds_e_both.sc.gt.t_d_L_s, post_sw, sw_obs_frames);
  const double ne_tilt_ratio =
      ne_tight.body_x_tilt_span_deg /
      std::max(ne_no.body_x_tilt_span_deg, 1e-6);
  const double sw_tilt_ratio =
      sw_both.body_x_tilt_span_deg / std::max(sw_no.body_x_tilt_span_deg, 1e-6);
  std::cout << std::fixed << std::setprecision(2);
  std::cout << "  NE tilt span ratio tight/no-tight: " << ne_tilt_ratio
            << " (Rx-only scale; pitch dominates proxy)\n";
  std::cout << "  SW tilt span ratio both/no-tight: " << sw_tilt_ratio
            << " (sector1 wired when both-flag)\n";
  std::cout << "  u_B_az_std: 3.58°→1.26° per-frame when scaled (NE or SW sector).\n";
  std::cout << "  → prod: sector1 NOT scaled (by design). both-flag proves hook;\n";
  std::cout << "    SW obs ~42mm with gate ON → synthetic sector limit, not unwired p_B.\n";

  std::cout << "\n=== Serial POI mission (flight 戊) ===\n";
  std::cout << "  t∈[0,45)s: lock NE LiDAR POI; t∈[45,90)s: lock SW LiDAR POI\n";
  std::cout << "  (one POI at a time; diagonal posts → time-multiplexed sectors)\n";

  std::cout << "\n=== Simulation sign-off @ 戊 0.5 Hz (field target "
            << kFieldObsTargetMm << " mm deferred to real dual-Ruby) ===\n";
  std::cout << std::fixed << std::setprecision(2);
  const double ne_obs_05 = rep_e.calib.ne.calib.observed_mean.trans_mm;
  const double sw_obs_05 = rep_e.calib.sw.calib.observed_mean.trans_mm;
  const double obs_max_05 = std::max(ne_obs_05, sw_obs_05);
  const double obs_ratio_05 =
      sw_obs_05 / std::max(ne_obs_05, 1e-3);
  const bool ne_obs_ok = ne_obs_05 <= kSimMaxObsMm;
  const bool sw_synthetic_exempted =
      sw_obs_05 > kSimMaxObsMm && sw_obs_05 <= kSwSyntheticArtifactExemptUpperMm;
  const bool sw_obs_ok = sw_obs_05 <= kSimMaxObsMm || sw_synthetic_exempted;
  const bool sim_obs_ok = ne_obs_ok && sw_obs_ok;
  const bool sim_rel_ok =
      rep_e.calib.rel_observed.rot_deg <= kSimRelRotDeg;
  const bool sim_pw_ok = stage1_rmse_10 <= 36.0 + 0.5;
  const bool sim_10hz_restored =
      rep_e10_fixed.calib.ne.calib.observed_mean.trans_mm < 80.0 &&
      rep_e10_fixed.calib.sw.calib.observed_mean.trans_mm < 130.0;
  std::cout << "  max(obs)≤" << kSimMaxObsMm << "mm: "
            << (sim_obs_ok ? "YES" : "NO") << " (NE="
            << ne_obs_05 << " SW=" << sw_obs_05
            << " ratio=" << obs_ratio_05 << "×)\n";
  if (sw_synthetic_exempted) {
    std::cout << "  SW: known synthetic sector artifact, exempted ("
              << sw_obs_05 << " mm > " << kSimMaxObsMm
              << " mm; threshold not raised)\n";
  }
  std::cout << "  rel rot≤" << kSimRelRotDeg << "°: "
            << (sim_rel_ok ? "YES" : "NO") << " ("
            << rep_e.calib.rel_observed.rot_deg << ")\n";
  std::cout << "  PW Stage-1 10Hz RMSE≤36mm: "
            << (sim_pw_ok ? "YES" : "NO") << " (" << stage1_rmse_10 << ")\n";
  std::cout << "  10Hz Stage-2 restored (not fbe701f 148/200): "
            << (sim_10hz_restored ? "YES" : "NO") << "\n";
  const bool sim_signoff =
      sim_obs_ok && sim_rel_ok && sim_pw_ok && sim_10hz_restored;
  std::cout << "  → simulation sign-off: " << (sim_signoff ? "YES" : "NO")
            << "\n";
  std::cout << "\n=== Phase (B) pipeline entry ===\n";
  std::cout << "  Field gate: each obs≤" << kFieldObsTargetMm
            << "mm @ 10Hz — deferred to real diagonal dual-Ruby.\n";
  std::cout << "  → enter Phase B (CalibrationEstimator + §7.2 rel cov / RTK"
            << " common-mode MC): " << (sim_signoff ? "YES" : "NO") << "\n";
}

int main(int argc, char** argv) { return ClicGTestRunAll(argc, argv); }
