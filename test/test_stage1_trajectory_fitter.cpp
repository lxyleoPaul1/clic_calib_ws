#include <clic_calib/estimator/stage1_trajectory_fitter.h>
#include <clic_calib/utils/lever_arm.h>
#include <clic_calib/utils/noise_model.h>
#include <clic_calib/utils/sophus_utils.hpp>
#include "gtest_ceres_guard.hpp"


#include <gtest/gtest.h>

#include "diagnostic/attitude_scenario_common.hpp"
#include "diagnostic/two_stage_probe_common.hpp"
#include "experiments/noise_regime_common.hpp"
#include "experiments/synthetic_flight_geometry.hpp"

#include <cmath>
#include <iostream>
#include <vector>

namespace {

constexpr int kNumSeeds = 50;
constexpr uint32_t kSeedBase = 13000;
constexpr double kStage1AlphaP = 0.01;
constexpr double kStage1AlphaR = 0.01;

std::vector<double> TrajSampleTimes(double t_end) {
  std::vector<double> times;
  for (double t = 0.5; t <= t_end - 0.5; t += 0.5) {
    times.push_back(t);
  }
  return times;
}

double TrajPosRmsMm(const clic_calib::BodyTrajectory& est,
                    const clic_calib::BodyTrajectory& gt, double t_end) {
  const auto times = TrajSampleTimes(t_end);
  double sq = 0.0;
  for (double t : times) {
    sq += (est.position_wb(t) - gt.position_wb(t)).squaredNorm();
  }
  return std::sqrt(sq / std::max<size_t>(times.size(), 1)) * 1e3;
}

struct TrajRpyRms {
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
};

TrajRpyRms TrajRpyRmsDeg(const clic_calib::BodyTrajectory& est,
                       const clic_calib::BodyTrajectory& gt, double t_end) {
  const auto times = TrajSampleTimes(t_end);
  TrajRpyRms out;
  double sq_r = 0.0, sq_p = 0.0, sq_y = 0.0;
  for (double t : times) {
    const clic_calib::SO3d R_err =
        gt.rotation_wb(t).inverse() * est.rotation_wb(t);
    const Eigen::Vector3d e = R_err.log() * 180.0 / M_PI;
    sq_r += e.x() * e.x();
    sq_p += e.y() * e.y();
    sq_y += e.z() * e.z();
  }
  const double n = static_cast<double>(std::max<size_t>(times.size(), 1));
  out.roll = std::sqrt(sq_r / n);
  out.pitch = std::sqrt(sq_p / n);
  out.yaw = std::sqrt(sq_y / n);
  return out;
}

struct RunningStats {
  double sum = 0.0;
  double sum_sq = 0.0;
  int n = 0;
  void Push(double x) {
    sum += x;
    sum_sq += x * x;
    ++n;
  }
  double Mean() const { return n > 0 ? sum / n : 0.0; }
};

clic_calib::Stage1TrajectoryConfig ProdConfig(
    const clic_calib::two_stage_probe::SplineConfig& spline_cfg) {
  clic_calib::Stage1TrajectoryConfig cfg;
  cfg.knot_interval_s = spline_cfg.knot_interval_s;
  cfg.alpha_p = kStage1AlphaP;
  cfg.alpha_R = kStage1AlphaR;
  cfg.attitude_stride = 25;
  cfg.trim_to_observation_support = true;
  return cfg;
}

}  // namespace

TEST(Stage1TrajectoryFitter, ReproducesProbeStage1Metrics) {
  const std::string config_dir =
      clic_calib::experiments::ConfigDirFromExperiments();
  const auto levers =
      clic_calib::LeverArmConfig::from_yaml(config_dir + "/lever_arms.yaml");
  const auto spline_cfg =
      clic_calib::two_stage_probe::LoadSplineConfig(config_dir + "/spline.yaml");
  const clic_calib::NoiseModel noise =
      clic_calib::NoiseModel::FromConfigDir(config_dir);
  const double t_end =
      clic_calib::experiments::NearFieldFlightDurationS(
          clic_calib::experiments::NearFieldFimScenarioGeometry());
  const clic_calib::Stage1TrajectoryConfig prod_cfg = ProdConfig(spline_cfg);

  RunningStats pos_rms, roll_rms, pitch_rms, yaw_rms;
  RunningStats max_pose_diff_mm, max_rot_diff_mrad;
  int usable = 0;

  std::cout << "\n=== Production Stage1TrajectoryFitter vs probe (N="
            << kNumSeeds << ") ===\n";

  for (int i = 0; i < kNumSeeds; ++i) {
    const uint32_t seed = kSeedBase + static_cast<uint32_t>(i);
    const clic_calib::experiments::SyntheticScenarioBundle sc =
        clic_calib::experiments::BuildNearFieldFimNoisyScenarioWithAttitude(
            seed, noise);

    const clic_calib::Stage1TrajectoryInput input =
        clic_calib::Stage1TrajectoryInput::FromRtkAttitudeStreams(
            sc.rtk, sc.attitude_obs);
    const clic_calib::Stage1TrajectoryResult prod =
        clic_calib::Stage1TrajectoryFitter::Fit(input, levers, prod_cfg);
    if (!prod.summary.IsSolutionUsable()) {
      continue;
    }

    // Probe reference: same logic, no post-solve trim (historical probe path).
    clic_calib::Stage1TrajectoryConfig probe_cfg = prod_cfg;
    probe_cfg.trim_to_observation_support = false;
    const clic_calib::Stage1TrajectoryResult probe_ref =
        clic_calib::Stage1TrajectoryFitter::Fit(input, levers, probe_cfg);

    const auto times = TrajSampleTimes(t_end);
    double max_trans = 0.0;
    double max_rot = 0.0;
    for (double t : times) {
      max_trans = std::max(
          max_trans,
          (prod.trajectory->position_wb(t) -
           probe_ref.trajectory->position_wb(t))
              .norm());
      const clic_calib::SO3d dR =
          probe_ref.trajectory->rotation_wb(t).inverse() *
          prod.trajectory->rotation_wb(t);
      max_rot = std::max(max_rot, dR.log().norm());
    }
    max_pose_diff_mm.Push(max_trans * 1e3);
    max_rot_diff_mrad.Push(max_rot * 1e3);

    pos_rms.Push(TrajPosRmsMm(*prod.trajectory, sc.gt_traj, t_end));
    const TrajRpyRms rpy =
        TrajRpyRmsDeg(*prod.trajectory, sc.gt_traj, t_end);
    roll_rms.Push(rpy.roll);
    pitch_rms.Push(rpy.pitch);
    yaw_rms.Push(rpy.yaw);
    ++usable;
  }

  std::cout << "  usable=" << usable << "/" << kNumSeeds << "\n";
  std::cout << "  position RMS [mm]: " << pos_rms.Mean() << "\n";
  std::cout << "  roll RMS [deg]:    " << roll_rms.Mean() << "\n";
  std::cout << "  pitch RMS [deg]:   " << pitch_rms.Mean() << "\n";
  std::cout << "  yaw RMS [deg]:     " << yaw_rms.Mean() << "\n";
  std::cout << "  max |prod-probe| trans [mm]: " << max_pose_diff_mm.Mean()
            << "\n";
  std::cout << "  max |prod-probe| rot [mrad]: " << max_rot_diff_mrad.Mean()
            << "\n";

  EXPECT_GE(usable, 45);
  EXPECT_NEAR(roll_rms.Mean(), 0.15, 0.08);
  EXPECT_NEAR(pitch_rms.Mean(), 0.15, 0.08);
  EXPECT_NEAR(yaw_rms.Mean(), 2.9, 0.8);
  EXPECT_NEAR(pos_rms.Mean(), 25.0, 12.0);
  EXPECT_LT(max_pose_diff_mm.Mean(), 5.0);
  EXPECT_LT(max_rot_diff_mrad.Mean(), 50.0);
}

int main(int argc, char** argv) { return ClicGTestRunAll(argc, argv); }
