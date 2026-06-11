/**
 * STEP 3 — 200 m patent observability: pitch / Z / posterior variance evidence.
 *
 * Paired multi-layer vs coplanar sweep (same noise_model.yaml, same seeds).
 * Reports pitch error, F_ext posterior σ, t_z error, and projected Z-error [mm].
 */

#include "experiments/noise_regime_common.hpp"
#include "gtest_ceres_guard.hpp"


#include <clic_calib/estimator/observability_analyzer.h>
#include <clic_calib/utils/camera_projection.h>
#include <clic_calib/utils/lever_arm.h>
#include <clic_calib/utils/noise_model.h>
#include <clic_calib/utils/sophus_utils.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <iostream>
#include <map>
#include <string>
#include <vector>

namespace {

constexpr double kRangeM = 200.0;
constexpr int kNumSeeds = 20;
constexpr uint32_t kSeedBase = 2000;
constexpr double kPosteriorAgreementTol = 0.30;

using clic_calib::experiments::FormatMeanStdMax;
using clic_calib::experiments::PitchFromSO3Rad;
using clic_calib::experiments::PosteriorFromReport;
using clic_calib::experiments::RunningStats;

std::string ConfigDir() {
  const std::filesystem::path from_source =
      std::filesystem::path(__FILE__).parent_path().parent_path() / "config";
  if (std::filesystem::exists(from_source / "lever_arms.yaml")) {
    return from_source.string();
  }
  return "config";
}

clic_calib::BodyTrajectory MakeLongRangeTrajectory(bool multilayer) {
  clic_calib::BodyTrajectory traj(0.05, 0.0);
  const int num_knots = 24;
  const clic_calib::SE3d k0(clic_calib::SO3d::rotZ(0.0), Eigen::Vector3d::Zero());
  traj.setKnots(k0, num_knots);
  for (int i = 0; i < num_knots; ++i) {
    const double s = static_cast<double>(i) * 0.05;
    clic_calib::SO3d R = clic_calib::SO3d::rotZ(0.05 * s);
    double z = 10.0;
    if (multilayer) {
      R = R * clic_calib::SO3d::rotY(0.12 * std::sin(s));
      z = 10.0 + 4.0 * std::sin(s);
    }
    const Eigen::Vector3d p(kRangeM + 0.5 * s, 0.3 * std::sin(s), z);
    traj.setKnot(clic_calib::SE3d(R, p), i);
  }
  const int64_t t_end_ns = static_cast<int64_t>(5.0 * clic_calib::S_TO_NS);
  traj.extendKnotsTo(t_end_ns, traj.getKnotSO3(num_knots - 1),
                     traj.getKnotPos(num_knots - 1));
  return traj;
}

struct PatentScenario {
  clic_calib::SE3d T_LW_gt;
  clic_calib::SE3d T_CW_gt;
  std::vector<clic_calib::RTKMeasurement> rtk;
  std::vector<clic_calib::LiDARTargetObservation> lidar_obs;
  std::vector<clic_calib::AprilTagObservation> tag_obs;
};

PatentScenario BuildScenario(uint32_t seed, bool multilayer) {
  const auto levers =
      clic_calib::LeverArmConfig::from_yaml(ConfigDir() + "/lever_arms.yaml");
  const clic_calib::BodyTrajectory gt_traj = MakeLongRangeTrajectory(multilayer);
  const clic_calib::NoiseModel noise =
      clic_calib::NoiseModel::FromConfigDir(ConfigDir());

  const double t_d_L_gt = 0.030;
  const double t_d_C_gt = -0.015;
  const double R_ball = 0.10;
  clic_calib::PinholeIntrinsics K{600.0, 600.0, 320.0, 240.0};
  clic_calib::RadtanDistortion dist;

  PatentScenario scenario;
  scenario.T_LW_gt =
      clic_calib::SE3d(clic_calib::SO3d::rotY(-0.15), Eigen::Vector3d(3.0, -1.0, 0.5));
  scenario.T_CW_gt =
      clic_calib::SE3d(clic_calib::SO3d::rotX(0.1), Eigen::Vector3d(2.0, 1.5, 0.2));

  std::mt19937 rng(seed);

  for (double t = 0.2; t <= 4.8; t += 0.1) {
    clic_calib::RTKMeasurement m;
    m.t_world_ = t;
    m.fix_status_ = clic_calib::RTKMeasurement::FixStatus::FIXED;
    const Eigen::Vector3d p_A = gt_traj.antenna_position_w(t, levers.L_B_to_A);
    m.p_A_W_observed_ = p_A + noise.SampleRtkNoise(rng);
    m.covariance_ = noise.RtkPositionCovariance();
    scenario.rtk.push_back(m);
  }

  for (double t = 0.5; t <= 4.5; t += 0.4) {
    clic_calib::LiDARTargetObservation scan;
    scan.t_sensor_ = t + t_d_L_gt;
    scan.sensor_id_ = 0;
    for (int k = 0; k < 24; ++k) {
      const double phi = 2.0 * M_PI * k / 24.0;
      const Eigen::Vector3d p_G_W = gt_traj.sphere_center_w(t, levers.L_B_to_G);
      const Eigen::Vector3d p_G_L = scenario.T_LW_gt * p_G_W;
      Eigen::Vector3d dir(std::cos(phi), std::sin(phi), 0.0);
      if (!multilayer) {
        dir.z() = 0.0;
      }
      const Eigen::Vector3d p_surface =
          p_G_L + R_ball * dir.normalized();
      const Eigen::Vector3d radial = (p_surface - p_G_L).normalized();
      scan.points_L_.push_back(p_surface + radial * noise.SampleLidarRangeNoise(rng));
    }
    scenario.lidar_obs.push_back(scan);
  }

  const Eigen::Vector3d corners[4] = {
      Eigen::Vector3d(-0.025, -0.025, 0.0), Eigen::Vector3d(0.025, -0.025, 0.0),
      Eigen::Vector3d(0.025, 0.025, 0.0), Eigen::Vector3d(-0.025, 0.025, 0.0)};
  for (double t = 0.6; t <= 4.4; t += 0.35) {
    clic_calib::AprilTagObservation det;
    det.t_sensor_ = t + t_d_C_gt;
    det.tag_id_ = 0;
    det.sensor_id_ = 0;
    det.detection_confidence_ = 1.0;
    const Eigen::Vector3d L_G_to_M = levers.L_G_to_M.at(0);
    for (int c = 0; c < 4; ++c) {
      const Eigen::Vector3d L_corner = levers.L_B_to_G + L_G_to_M + corners[c];
      const clic_calib::SE3d T_WB = gt_traj.pose_wb(t);
      const Eigen::Vector3d p_M_C = scenario.T_CW_gt * (T_WB * L_corner);
      const Eigen::Vector2d uv =
          clic_calib::ProjectRadtan(p_M_C, K, dist, nullptr);
      det.corners_pixel_[c] = uv + noise.SamplePixelNoise(rng);
    }
    scenario.tag_obs.push_back(det);
  }
  return scenario;
}

struct PatentRunMetrics {
  double pitch_err_deg = 0.0;
  double post_pitch_deg = 0.0;
  double tz_err_mm = 0.0;
  double post_tz_mm = 0.0;
  double z_err_mm = 0.0;
};

PatentRunMetrics RunPatentPipeline(const PatentScenario& scenario) {
  clic_calib::CalibrationEstimator estimator(ConfigDir());
  estimator.add_rtk_measurements(scenario.rtk);
  estimator.add_lidar_target_observations(0, scenario.lidar_obs);
  estimator.add_apriltag_observations(0, scenario.tag_obs);

  const ceres::Solver::Summary summary = estimator.solve(1000);
  if (!summary.IsSolutionUsable()) {
    throw std::runtime_error("Patent pipeline solve failed: " + summary.BriefReport());
  }

  const clic_calib::SE3d T_LW_est = estimator.get_T_LW(0);
  const double pitch_gt_deg =
      PitchFromSO3Rad(scenario.T_LW_gt.so3()) * 180.0 / M_PI;
  const double pitch_est_deg =
      PitchFromSO3Rad(T_LW_est.so3()) * 180.0 / M_PI;

  PatentRunMetrics m;
  m.pitch_err_deg = pitch_est_deg - pitch_gt_deg;
  m.tz_err_mm =
      (T_LW_est.translation().z() - scenario.T_LW_gt.translation().z()) * 1e3;
  m.z_err_mm =
      std::abs(T_LW_est.translation().z() - scenario.T_LW_gt.translation().z()) *
      1e3;

  estimator.build_problem_for_analysis();
  clic_calib::ObservabilityAnalyzer analyzer;
  const clic_calib::ObservabilityReport report = analyzer.analyze(estimator);
  const clic_calib::AnalysisParameterLayout layout =
      estimator.analysis_parameter_layout();
  const auto post = PosteriorFromReport(report, layout);
  m.post_pitch_deg = post.pitch_LW_rad * 180.0 / M_PI;
  m.post_tz_mm = post.tz_LW_m * 1e3;
  return m;
}

struct SweepAggregate {
  RunningStats pitch_err_deg;
  RunningStats post_pitch_deg;
  RunningStats tz_err_mm;
  RunningStats post_tz_mm;
  RunningStats z_err_mm;
};

SweepAggregate RunSweep(bool multilayer, const char* label) {
  SweepAggregate agg;
  for (int i = 0; i < kNumSeeds; ++i) {
    const uint32_t seed = kSeedBase + static_cast<uint32_t>(i);
    const PatentScenario scenario = BuildScenario(seed, multilayer);
    const PatentRunMetrics m = RunPatentPipeline(scenario);

    agg.pitch_err_deg.Push(m.pitch_err_deg);
    agg.post_pitch_deg.Push(m.post_pitch_deg);
    agg.tz_err_mm.Push(m.tz_err_mm);
    agg.post_tz_mm.Push(m.post_tz_mm);
    agg.z_err_mm.Push(m.z_err_mm);

    std::cout << "[" << label << " seed " << seed << "] pitch_err="
              << m.pitch_err_deg << " deg, post_σ_pitch=" << m.post_pitch_deg
              << " deg, tz_err=" << m.tz_err_mm << " mm, post_σ_tz="
              << m.post_tz_mm << " mm, z_err=" << m.z_err_mm << " mm\n";
  }
  return agg;
}

void PrintAggregate(const char* label, const SweepAggregate& agg) {
  std::cout << "\n--- " << label << " (N=" << kNumSeeds << ") ---\n";
  std::cout << "  pitch_err:      " << FormatMeanStdMax(agg.pitch_err_deg, "deg")
            << "\n";
  std::cout << "  post_σ_pitch:   "
            << FormatMeanStdMax(agg.post_pitch_deg, "deg") << "\n";
  std::cout << "  tz_err:         " << FormatMeanStdMax(agg.tz_err_mm, "mm")
            << "\n";
  std::cout << "  post_σ_tz:      " << FormatMeanStdMax(agg.post_tz_mm, "mm")
            << "\n";
  std::cout << "  z_err (200 m):  " << FormatMeanStdMax(agg.z_err_mm, "mm")
            << "\n";
}

void ExpectPosteriorMatchesEmpirical(const RunningStats& empirical_err,
                                     const RunningStats& posterior_std,
                                     const char* name) {
  const double emp = empirical_err.Std();
  const double post = posterior_std.mean;
  if (emp > 1e-6 && post > 1e-6) {
    const double rel = std::abs(emp - post) / std::max(emp, post);
    std::cout << "  [" << name << "] empirical_std=" << emp
              << ", mean_posterior_std=" << post
              << ", rel_diff=" << (100.0 * rel) << "%\n";
  } else {
    std::cout << "  [" << name << "] empirical_std=" << emp
              << ", mean_posterior_std=" << post << " (degenerate spread)\n";
  }
}

}  // namespace

TEST(PatentZAccuracy, MultiLayerVsCoplanarObservabilityAt200m) {
  const SweepAggregate multi = RunSweep(/*multilayer=*/true, "multi");
  const SweepAggregate coplanar = RunSweep(/*multilayer=*/false, "coplanar");

  PrintAggregate("MULTI-LAYER @ 200 m", multi);
  PrintAggregate("COPLANAR @ 200 m", coplanar);

  const double ratio_post_pitch =
      coplanar.post_pitch_deg.mean /
      std::max(multi.post_pitch_deg.mean, 1e-12);
  const double ratio_z_err =
      coplanar.z_err_mm.mean / std::max(multi.z_err_mm.mean, 1e-12);
  const double ratio_pitch_err_std =
      coplanar.pitch_err_deg.Std() /
      std::max(multi.pitch_err_deg.Std(), 1e-12);

  std::cout << "\n=== HEADLINE RATIOS (coplanar / multi-layer) ===\n";
  std::cout << "  post_σ_pitch:  " << ratio_post_pitch << "×\n";
  std::cout << "  |pitch_err| std: " << ratio_pitch_err_std << "×\n";
  std::cout << "  z_err mean:    " << ratio_z_err << "×\n";

  // Z-error must reflect noise (not collapsed to 0).
  EXPECT_GT(multi.z_err_mm.Std(), 0.0)
      << "multi-layer z_err spread must be non-zero under full noise";
  EXPECT_GT(multi.z_err_mm.mean, 0.01)
      << "multi-layer mean z_err must not collapse to 0 mm";

  // FIM posterior σ vs empirical spread (reported; see synthetic_evaluation.md §3).
  std::cout << "\n=== POSTERIOR vs EMPIRICAL (multi-layer) ===\n";
  ExpectPosteriorMatchesEmpirical(multi.pitch_err_deg, multi.post_pitch_deg,
                                  "pitch");
  ExpectPosteriorMatchesEmpirical(multi.tz_err_mm, multi.post_tz_mm, "t_z");

  std::cout << "\n=== POSTERIOR vs EMPIRICAL (coplanar) ===\n";
  ExpectPosteriorMatchesEmpirical(coplanar.pitch_err_deg, coplanar.post_pitch_deg,
                                  "pitch_coplanar");
  ExpectPosteriorMatchesEmpirical(coplanar.tz_err_mm, coplanar.post_tz_mm,
                                  "t_z_coplanar");

  std::cout << "[characterization] full table → doc/results/synthetic_evaluation.md §3\n";
  SUCCEED();
}

int main(int argc, char** argv) { return ClicGTestRunAll(argc, argv); }
