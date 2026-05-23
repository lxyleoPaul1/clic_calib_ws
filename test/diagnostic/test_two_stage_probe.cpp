/**
 * Two-stage decoupled solver feasibility probe.
 * RTK-only baseline + RTK+PSDK attitude extension (probe/two-stage-attitude).
 */

#include "diagnostic/attitude_scenario_common.hpp"
#include "diagnostic/closed_form_init_common.hpp"
#include "diagnostic/two_stage_probe_common.hpp"
#include "experiments/noise_regime_common.hpp"
#include "experiments/synthetic_flight_geometry.hpp"

#include <clic_calib/factor/attitude_factor_pose_form.h>

#include <gtest/gtest.h>

#include <ceres/ceres.h>
#include <ceres/crs_matrix.h>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <functional>
#include <string>
#include <thread>
#include <vector>

#include "../factor_test_utils.h"

namespace {

constexpr int kNumSeeds = 50;
constexpr uint32_t kSeedBase = 13000;
constexpr uint32_t kRepSeed = 13025;
constexpr double kRatioLo = 0.7;
constexpr double kRatioHi = 1.4;
constexpr double kStage1AlphaP = 0.01;
constexpr double kStage1AlphaR = 0.01;
constexpr double kSphereRadiusM = 0.10;
constexpr double kJointCondRef = 5.6865465329150156e+12;
constexpr double kCmTransMm = 50.0;
constexpr double kCmPitchDeg = 1.0;
constexpr double kWrongBasinTransMm = 500.0;

using clic_calib::AprilTagObservation;
using clic_calib::BodyTrajectory;
using clic_calib::LiDARTargetObservation;
using clic_calib::PinholeIntrinsics;
using clic_calib::RadtanDistortion;
using clic_calib::RTKMeasurement;
using clic_calib::SE3d;
using clic_calib::SO3d;
using clic_calib::SplineSegmentMeta;
using clic_calib::experiments::BuildNearFieldCoplanarFimNoisyScenario;
using clic_calib::experiments::BuildNearFieldFimNoisyScenario;
using clic_calib::experiments::ConfigDirFromExperiments;
using clic_calib::experiments::NearFieldFlightDurationS;
using clic_calib::experiments::NearFieldFimScenarioGeometry;
using clic_calib::experiments::RealisticNoiseSpec;
using clic_calib::experiments::RunningStats;
using clic_calib::experiments::SyntheticScenarioBundle;
using clic_calib::two_stage_probe::AnalyzeSymmetricF;
using clic_calib::two_stage_probe::CoarseExtrinsicInit;
using clic_calib::two_stage_probe::EigenReport;
using clic_calib::two_stage_probe::ExtrinsicState;
using clic_calib::two_stage_probe::GetActiveKnotPointers;
using clic_calib::two_stage_probe::InformationFromCRS;
using clic_calib::two_stage_probe::InterpolateRtkPosition;
using clic_calib::two_stage_probe::LoadCoarseExtrinsicsFromYaml;
using clic_calib::two_stage_probe::MakeCoarseExtrinsicInitPerturbed;
using clic_calib::two_stage_probe::GeometricInitReport;
using clic_calib::two_stage_probe::InitTCWFromAprilTagPnPPerFrame;
using clic_calib::two_stage_probe::InitTLWFromLidarSphereCenters;
using clic_calib::two_stage_probe::UmeyamaRigid;
using clic_calib::two_stage_probe::MakeGeometricExtrinsicInit;
using clic_calib::two_stage_probe::SynthesizeTagObsForTrajectory;
using clic_calib::two_stage_probe::LoadSplineConfig;
using clic_calib::two_stage_probe::MinSpacingInSortedTimes;
using clic_calib::two_stage_probe::CeresSo3ProblemScope;
using clic_calib::two_stage_probe::ReseedKnotPositionsFromRtkLeverArm;
using clic_calib::two_stage_probe::RtkAntennaRmsMm;
using clic_calib::two_stage_probe::StateToSE3;
using clic_calib::AttitudeObservation;
using clic_calib::experiments::AttitudeNoiseSpec;
using clic_calib::experiments::AttitudeObsErrorDeg;
using clic_calib::experiments::BuildNearFieldFimNoisyScenarioWithAttitude;
using clic_calib::two_stage_probe::AttitudeInitMode;
using clic_calib::two_stage_probe::InitStage1TrajectoryNoGtAttitude;
using clic_calib::two_stage_probe::Stage1KnotDt;
using clic_calib::two_stage_probe::TrimTrajectoryToObservedSupport;
using clic_calib::two_stage_probe::FixedTrajAprilTagFactor;
using clic_calib::two_stage_probe::FixedTrajSphereFactor;
using clic_calib::two_stage_probe::SplineConfig;

struct ExtrinsicSixDofErrors {
  Eigen::Vector3d rot_err_rad = Eigen::Vector3d::Zero();
  Eigen::Vector3d trans_err_m = Eigen::Vector3d::Zero();
};

ExtrinsicSixDofErrors ExtrinsicError(const SE3d& T_est, const SE3d& T_gt) {
  ExtrinsicSixDofErrors e;
  const SO3d R_err = T_gt.so3().inverse() * T_est.so3();
  e.rot_err_rad = R_err.log();
  e.trans_err_m = T_est.translation() - T_gt.translation();
  return e;
}

double StandardError(const RunningStats& s) {
  return s.n >= 2 ? s.Std() / std::sqrt(static_cast<double>(s.n)) : 0.0;
}

void PrintBiasStd(const std::string& label, const RunningStats& s,
                  const char* unit, int prec = 4) {
  std::cout << "  " << std::setw(28) << std::left << label << "  bias="
            << std::fixed << std::setprecision(prec) << s.mean << " ± "
            << StandardError(s) << " " << unit << "   std=" << s.Std() << " "
            << unit << "\n";
}

std::vector<double> TrajSampleTimes(double t_end) {
  std::vector<double> times;
  for (double t = 1.0; t <= t_end - 1.0; t += 2.0) {
    times.push_back(t);
  }
  return times;
}

double TrajPosRmsMm(const BodyTrajectory& est, const BodyTrajectory& gt,
                    double t_end) {
  const auto times = TrajSampleTimes(t_end);
  double sq = 0.0;
  for (double t : times) {
    sq += (est.position_wb(t) - gt.position_wb(t)).squaredNorm();
  }
  return std::sqrt(sq / std::max<size_t>(times.size(), 1)) * 1e3;
}

double TrajRotRmsDeg(const BodyTrajectory& est, const BodyTrajectory& gt,
                     double t_end) {
  const auto times = TrajSampleTimes(t_end);
  double sq = 0.0;
  for (double t : times) {
    const SO3d R_err = gt.rotation_wb(t).inverse() * est.rotation_wb(t);
    sq += R_err.log().squaredNorm();
  }
  return std::sqrt(sq / std::max<size_t>(times.size(), 1)) * 180.0 / M_PI;
}

double RtkOnlyKnotDt(const std::vector<RTKMeasurement>& rtk,
                     double knot_interval_s) {
  std::vector<double> times;
  times.reserve(rtk.size());
  for (const auto& m : rtk) {
    times.push_back(m.t_world_);
  }
  const double min_dt = MinSpacingInSortedTimes(times);
  return std::max(knot_interval_s, min_dt);
}

SplineSegmentMeta<clic_calib::SplineOrder> TrajectoryMeta(
    const BodyTrajectory& traj) {
  return SplineSegmentMeta<clic_calib::SplineOrder>(
      traj.minTimeNs(), traj.getDtNs(), traj.numKnots());
}

struct Stage1Result {
  std::shared_ptr<BodyTrajectory> trajectory;
  ceres::Solver::Summary summary;
  EigenReport fim;
};

Stage1Result FitStage1Trajectory(const SyntheticScenarioBundle& scenario,
                                   const clic_calib::LeverArmConfig& levers,
                                   const SplineConfig& spline_cfg,
                                   double alpha_p, double alpha_R) {
  Stage1Result out;
  if (scenario.rtk.empty()) {
    throw std::runtime_error("FitStage1Trajectory: no RTK");
  }
  const double t_lo = scenario.rtk.front().t_world_;
  const double t_hi = scenario.rtk.back().t_world_;
  const double knot_dt =
      RtkOnlyKnotDt(scenario.rtk, spline_cfg.knot_interval_s);
  out.trajectory = TrimTrajectoryToObservedSupport(
      scenario.gt_traj, t_lo, t_hi, knot_dt);

  const SplineSegmentMeta<clic_calib::SplineOrder> meta =
      TrajectoryMeta(*out.trajectory);

  auto build_and_solve = [&](int max_iters, ceres::Solver::Summary* summary) {
    CeresSo3ProblemScope scope;
    ceres::Problem& problem = *scope.problem;
    std::set<double*> so3_registered;

    auto register_so3_knots =
        [&](const std::array<double*, clic_calib::SplineOrder>& rot_knots) {
          for (double* q : rot_knots) {
            if (so3_registered.insert(q).second) {
              scope.SetLocalParamSO3(q);
            }
          }
        };

    for (const auto& m : scenario.rtk) {
      if (m.fix_status_ != RTKMeasurement::FixStatus::FIXED) {
        continue;
      }
      const int64_t t_ns =
          static_cast<int64_t>(m.t_world_ * clic_calib::S_TO_NS);
      std::array<double*, clic_calib::SplineOrder> rot_knots{};
      std::array<double*, clic_calib::SplineOrder> pos_knots{};
      if (!GetActiveKnotPointers(*out.trajectory, t_ns, &rot_knots,
                                 &pos_knots)) {
        continue;
      }
      scope.owned_costs.push_back(
          std::make_unique<clic_calib::analytic_derivative::RTKPositionFactor>(
              t_ns, m.p_A_W_observed_, m.covariance_, levers.L_B_to_A, meta));
      auto* factor = scope.owned_costs.back().get();
      std::vector<double*> blocks;
      blocks.insert(blocks.end(), rot_knots.begin(), rot_knots.end());
      blocks.insert(blocks.end(), pos_knots.begin(), pos_knots.end());
      problem.AddResidualBlock(factor, nullptr, blocks);
      register_so3_knots(rot_knots);
    }

    if (alpha_p > 0.0 || alpha_R > 0.0) {
      const double dt_s = out.trajectory->getDt();
      for (size_t seg = 0;
           seg + clic_calib::SplineOrder <= out.trajectory->numKnots(); ++seg) {
        const int64_t t_mid_ns =
            meta.t0_ns + static_cast<int64_t>((static_cast<double>(seg) + 0.5) *
                                              meta.dt_ns);
        if (t_mid_ns < meta.MinTimeNs() || t_mid_ns >= meta.MaxTimeNs()) {
          continue;
        }
        std::array<double*, clic_calib::SplineOrder> rot_knots{};
        std::array<double*, clic_calib::SplineOrder> pos_knots{};
        if (!GetActiveKnotPointers(*out.trajectory, t_mid_ns, &rot_knots,
                                   &pos_knots)) {
          continue;
        }
        scope.owned_costs.push_back(std::make_unique<
            clic_calib::analytic_derivative::TrajectorySmoothnessFactor>(
            t_mid_ns, alpha_p, alpha_R, dt_s, meta));
        auto* factor = scope.owned_costs.back().get();
        std::vector<double*> blocks;
        blocks.insert(blocks.end(), rot_knots.begin(), rot_knots.end());
        blocks.insert(blocks.end(), pos_knots.begin(), pos_knots.end());
        problem.AddResidualBlock(factor, nullptr, blocks);
        register_so3_knots(rot_knots);
      }
    }

    ceres::Solver::Options opts;
    opts.max_num_iterations = max_iters;
    opts.minimizer_progress_to_stdout = false;
    ceres::Solve(opts, &problem, summary);
    return scope;
  };

  build_and_solve(100, &out.summary);
  ReseedKnotPositionsFromRtkLeverArm(out.trajectory.get(), scenario.rtk,
                                      levers.L_B_to_A);
  build_and_solve(200, &out.summary);
  ReseedKnotPositionsFromRtkLeverArm(out.trajectory.get(), scenario.rtk,
                                      levers.L_B_to_A);
  ceres::Solver::Summary polish;
  build_and_solve(70, &polish);
  if (polish.final_cost < out.summary.final_cost) {
    out.summary = polish;
  }

  {
    CeresSo3ProblemScope scope;
    ceres::Problem& problem = *scope.problem;
    std::set<double*> so3_registered;
    auto register_so3_knots =
        [&](const std::array<double*, clic_calib::SplineOrder>& rot_knots) {
          for (double* q : rot_knots) {
            if (so3_registered.insert(q).second) {
              scope.SetLocalParamSO3(q);
            }
          }
        };
    for (const auto& m : scenario.rtk) {
      if (m.fix_status_ != RTKMeasurement::FixStatus::FIXED) {
        continue;
      }
      const int64_t t_ns =
          static_cast<int64_t>(m.t_world_ * clic_calib::S_TO_NS);
      std::array<double*, clic_calib::SplineOrder> rot_knots{};
      std::array<double*, clic_calib::SplineOrder> pos_knots{};
      if (!GetActiveKnotPointers(*out.trajectory, t_ns, &rot_knots,
                                 &pos_knots)) {
        continue;
      }
      scope.owned_costs.push_back(
          std::make_unique<clic_calib::analytic_derivative::RTKPositionFactor>(
              t_ns, m.p_A_W_observed_, m.covariance_, levers.L_B_to_A, meta));
      auto* factor = scope.owned_costs.back().get();
      std::vector<double*> blocks;
      blocks.insert(blocks.end(), rot_knots.begin(), rot_knots.end());
      blocks.insert(blocks.end(), pos_knots.begin(), pos_knots.end());
      problem.AddResidualBlock(factor, nullptr, blocks);
      register_so3_knots(rot_knots);
    }
    if (alpha_p > 0.0 || alpha_R > 0.0) {
      const double dt_s = out.trajectory->getDt();
      for (size_t seg = 0;
           seg + clic_calib::SplineOrder <= out.trajectory->numKnots(); ++seg) {
        const int64_t t_mid_ns =
            meta.t0_ns + static_cast<int64_t>((static_cast<double>(seg) + 0.5) *
                                              meta.dt_ns);
        if (t_mid_ns < meta.MinTimeNs() || t_mid_ns >= meta.MaxTimeNs()) {
          continue;
        }
        std::array<double*, clic_calib::SplineOrder> rot_knots{};
        std::array<double*, clic_calib::SplineOrder> pos_knots{};
        if (!GetActiveKnotPointers(*out.trajectory, t_mid_ns, &rot_knots,
                                   &pos_knots)) {
          continue;
        }
        scope.owned_costs.push_back(std::make_unique<
            clic_calib::analytic_derivative::TrajectorySmoothnessFactor>(
            t_mid_ns, alpha_p, alpha_R, dt_s, meta));
        auto* factor = scope.owned_costs.back().get();
        std::vector<double*> blocks;
        blocks.insert(blocks.end(), rot_knots.begin(), rot_knots.end());
        blocks.insert(blocks.end(), pos_knots.begin(), pos_knots.end());
        problem.AddResidualBlock(factor, nullptr, blocks);
        register_so3_knots(rot_knots);
      }
    }
    ceres::Problem::EvaluateOptions eval_opts;
    eval_opts.apply_loss_function = false;
    eval_opts.num_threads = 1;
    ceres::CRSMatrix J;
    problem.Evaluate(eval_opts, nullptr, nullptr, nullptr, &J);
    out.fim = AnalyzeSymmetricF(InformationFromCRS(J));
  }

  return out;
}

Eigen::Vector3d RotationErrorRpyDeg(const SO3d& R_est, const SO3d& R_gt) {
  const SO3d R_err = R_gt.inverse() * R_est;
  return R_err.log() * 180.0 / M_PI;
}

struct TrajRpyRms {
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
};

TrajRpyRms TrajRpyRmsDeg(const BodyTrajectory& est, const BodyTrajectory& gt,
                         double t_end) {
  const auto times = TrajSampleTimes(t_end);
  TrajRpyRms out;
  double sq_r = 0.0, sq_p = 0.0, sq_y = 0.0;
  for (double t : times) {
    const Eigen::Vector3d e =
        RotationErrorRpyDeg(est.rotation_wb(t), gt.rotation_wb(t));
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

void AddStage1AttitudeFactors(
    ceres::Problem* problem,
    std::vector<std::unique_ptr<ceres::CostFunction>>* owned,
    const std::vector<AttitudeObservation>& attitude,
    const BodyTrajectory& traj,
    const SplineSegmentMeta<clic_calib::SplineOrder>& meta,
    const std::function<void(const std::array<double*, clic_calib::SplineOrder>&)>&
        register_so3,
    int stride = 25) {
  for (size_t idx = 0; idx < attitude.size(); idx += static_cast<size_t>(stride)) {
    const auto& obs = attitude[idx];
    const int64_t t_ns =
        static_cast<int64_t>(obs.t_world_ * clic_calib::S_TO_NS);
    std::array<double*, clic_calib::SplineOrder> rot_knots{};
    std::array<double*, clic_calib::SplineOrder> pos_knots{};
    if (!GetActiveKnotPointers(traj, t_ns, &rot_knots, &pos_knots)) {
      continue;
    }
    owned->push_back(
        std::make_unique<clic_calib::analytic_derivative::AttitudeFactorPoseForm>(
            t_ns, obs.R_WB_observed_, obs.covariance_, meta));
    std::vector<double*> blocks(rot_knots.begin(), rot_knots.end());
    problem->AddResidualBlock(owned->back().get(), nullptr, blocks);
    register_so3(rot_knots);
  }
}

Stage1Result FitStage1AttitudeOnly(
    const SyntheticScenarioBundle& scenario,
    const clic_calib::LeverArmConfig& levers, const SplineConfig& spline_cfg,
    int attitude_stride = 1, double alpha_R = 0.0) {
  Stage1Result out;
  const double t_lo = scenario.attitude_obs.front().t_world_;
  const double t_hi = scenario.attitude_obs.back().t_world_;
  const double knot_dt = Stage1KnotDt(scenario.rtk, scenario.attitude_obs,
                                      spline_cfg.knot_interval_s);
  out.trajectory = InitStage1TrajectoryNoGtAttitude(
      t_lo, t_hi, knot_dt, scenario.rtk, levers.L_B_to_A,
      AttitudeInitMode::kAttitudeStream, &scenario.attitude_obs);
  const SplineSegmentMeta<clic_calib::SplineOrder> meta =
      TrajectoryMeta(*out.trajectory);
  CeresSo3ProblemScope scope;
  std::set<double*> so3_registered;
  auto register_so3_knots =
      [&](const std::array<double*, clic_calib::SplineOrder>& rot_knots) {
        for (double* q : rot_knots) {
          if (so3_registered.insert(q).second) {
            scope.SetLocalParamSO3(q);
          }
        }
      };
  AddStage1AttitudeFactors(scope.problem.get(), &scope.owned_costs,
                           scenario.attitude_obs, *out.trajectory, meta,
                           register_so3_knots, attitude_stride);
  ceres::Solver::Options opts;
  opts.max_num_iterations = 200;
  ceres::Solve(opts, scope.problem.get(), &out.summary);
  return out;
}

Stage1Result FitStage1WithAttitude(
    const SyntheticScenarioBundle& scenario,
    const clic_calib::LeverArmConfig& levers, const SplineConfig& spline_cfg,
    double alpha_p, double alpha_R) {
  Stage1Result out;
  if (scenario.rtk.empty()) {
    throw std::runtime_error("FitStage1WithAttitude: no RTK");
  }
  if (scenario.attitude_obs.empty()) {
    throw std::runtime_error("FitStage1WithAttitude: no attitude stream");
  }
  const double t_lo = scenario.rtk.front().t_world_;
  const double t_hi = scenario.rtk.back().t_world_;
  const double knot_dt = Stage1KnotDt(scenario.rtk, scenario.attitude_obs,
                                      spline_cfg.knot_interval_s);
  out.trajectory = InitStage1TrajectoryNoGtAttitude(
      t_lo, t_hi, knot_dt, scenario.rtk, levers.L_B_to_A,
      AttitudeInitMode::kAttitudeStream, &scenario.attitude_obs);

  const SplineSegmentMeta<clic_calib::SplineOrder> meta =
      TrajectoryMeta(*out.trajectory);

  struct Stage1BuildOpts {
    bool include_rtk = true;
    bool include_attitude = true;
    bool include_smoothness = true;
    bool rotation_constant = false;
  };

  auto build_and_solve = [&](int max_iters, const Stage1BuildOpts& opts,
                             ceres::Solver::Summary* summary) {
    CeresSo3ProblemScope scope;
    ceres::Problem& problem = *scope.problem;
    std::set<double*> so3_registered;
    std::set<double*> rot_blocks_all;
    auto register_so3_knots =
        [&](const std::array<double*, clic_calib::SplineOrder>& rot_knots) {
          for (double* q : rot_knots) {
            rot_blocks_all.insert(q);
            if (so3_registered.insert(q).second) {
              scope.SetLocalParamSO3(q);
            }
          }
        };

    if (opts.include_rtk) {
      for (const auto& m : scenario.rtk) {
        if (m.fix_status_ != RTKMeasurement::FixStatus::FIXED) {
          continue;
        }
        const int64_t t_ns =
            static_cast<int64_t>(m.t_world_ * clic_calib::S_TO_NS);
        std::array<double*, clic_calib::SplineOrder> rot_knots{};
        std::array<double*, clic_calib::SplineOrder> pos_knots{};
        if (!GetActiveKnotPointers(*out.trajectory, t_ns, &rot_knots,
                                   &pos_knots)) {
          continue;
        }
        scope.owned_costs.push_back(std::make_unique<
            clic_calib::analytic_derivative::RTKPositionFactor>(
            t_ns, m.p_A_W_observed_, m.covariance_, levers.L_B_to_A, meta));
        std::vector<double*> blocks;
        blocks.insert(blocks.end(), rot_knots.begin(), rot_knots.end());
        blocks.insert(blocks.end(), pos_knots.begin(), pos_knots.end());
        problem.AddResidualBlock(scope.owned_costs.back().get(), nullptr,
                                 blocks);
        register_so3_knots(rot_knots);
      }
    }

    if (opts.include_attitude) {
      AddStage1AttitudeFactors(&problem, &scope.owned_costs,
                               scenario.attitude_obs, *out.trajectory, meta,
                               register_so3_knots);
    }

    if (opts.include_smoothness && (alpha_p > 0.0 || alpha_R > 0.0)) {
      const double dt_s = out.trajectory->getDt();
      for (size_t seg = 0;
           seg + clic_calib::SplineOrder <= out.trajectory->numKnots(); ++seg) {
        const int64_t t_mid_ns =
            meta.t0_ns + static_cast<int64_t>((static_cast<double>(seg) + 0.5) *
                                              meta.dt_ns);
        if (t_mid_ns < meta.MinTimeNs() || t_mid_ns >= meta.MaxTimeNs()) {
          continue;
        }
        std::array<double*, clic_calib::SplineOrder> rot_knots{};
        std::array<double*, clic_calib::SplineOrder> pos_knots{};
        if (!GetActiveKnotPointers(*out.trajectory, t_mid_ns, &rot_knots,
                                   &pos_knots)) {
          continue;
        }
        scope.owned_costs.push_back(std::make_unique<
            clic_calib::analytic_derivative::TrajectorySmoothnessFactor>(
            t_mid_ns, alpha_p, alpha_R, dt_s, meta));
        std::vector<double*> blocks;
        blocks.insert(blocks.end(), rot_knots.begin(), rot_knots.end());
        blocks.insert(blocks.end(), pos_knots.begin(), pos_knots.end());
        problem.AddResidualBlock(scope.owned_costs.back().get(), nullptr,
                                 blocks);
        register_so3_knots(rot_knots);
      }
    }

    if (opts.rotation_constant) {
      for (double* q : rot_blocks_all) {
        problem.SetParameterBlockConstant(q);
      }
    }

    ceres::Solver::Options opts_solver;
    opts_solver.max_num_iterations = max_iters;
    opts_solver.minimizer_progress_to_stdout = false;
    ceres::Solve(opts_solver, &problem, summary);
    return scope;
  };

  // Pass A: PSDK attitude establishes R(t); RTK cannot observe yaw alone.
  build_and_solve(120, Stage1BuildOpts{false, true, true, false}, &out.summary);
  ReseedKnotPositionsFromRtkLeverArm(out.trajectory.get(), scenario.rtk,
                                     levers.L_B_to_A);
  // Pass B: RTK positions with R(t) fixed (lever-arm-corrected).
  ceres::Solver::Summary rtk_pass;
  build_and_solve(120, Stage1BuildOpts{true, false, true, true}, &rtk_pass);
  out.summary = rtk_pass;
  return out;
}

Eigen::MatrixXd ComputeStage1InformationMatrix(
    const SyntheticScenarioBundle& scenario, const BodyTrajectory& trajectory,
    const clic_calib::LeverArmConfig& levers, double alpha_p, double alpha_R);

EigenReport ComputeStage1FimWithAttitude(
    const SyntheticScenarioBundle& scenario,
    const BodyTrajectory& trajectory,
    const clic_calib::LeverArmConfig& levers, double alpha_p, double alpha_R) {
  return AnalyzeSymmetricF(ComputeStage1InformationMatrix(
      scenario, trajectory, levers, alpha_p, alpha_R));
}

Eigen::MatrixXd ComputeStage1InformationMatrix(
    const SyntheticScenarioBundle& scenario, const BodyTrajectory& trajectory,
    const clic_calib::LeverArmConfig& levers, double alpha_p, double alpha_R) {
  const SplineSegmentMeta<clic_calib::SplineOrder> meta =
      TrajectoryMeta(trajectory);
  CeresSo3ProblemScope scope;
  ceres::Problem& problem = *scope.problem;
  std::set<double*> so3_registered;
  auto register_so3_knots =
      [&](const std::array<double*, clic_calib::SplineOrder>& rot_knots) {
        for (double* q : rot_knots) {
          if (so3_registered.insert(q).second) {
            scope.SetLocalParamSO3(q);
          }
        }
      };
  for (const auto& m : scenario.rtk) {
    if (m.fix_status_ != RTKMeasurement::FixStatus::FIXED) {
      continue;
    }
    const int64_t t_ns = static_cast<int64_t>(m.t_world_ * clic_calib::S_TO_NS);
    std::array<double*, clic_calib::SplineOrder> rot_knots{};
    std::array<double*, clic_calib::SplineOrder> pos_knots{};
    if (!GetActiveKnotPointers(trajectory, t_ns, &rot_knots, &pos_knots)) {
      continue;
    }
    scope.owned_costs.push_back(
        std::make_unique<clic_calib::analytic_derivative::RTKPositionFactor>(
            t_ns, m.p_A_W_observed_, m.covariance_, levers.L_B_to_A, meta));
    std::vector<double*> blocks;
    blocks.insert(blocks.end(), rot_knots.begin(), rot_knots.end());
    blocks.insert(blocks.end(), pos_knots.begin(), pos_knots.end());
    problem.AddResidualBlock(scope.owned_costs.back().get(), nullptr, blocks);
    register_so3_knots(rot_knots);
  }
  AddStage1AttitudeFactors(&problem, &scope.owned_costs, scenario.attitude_obs,
                           trajectory, meta, register_so3_knots);
  if (alpha_p > 0.0 || alpha_R > 0.0) {
    const double dt_s = trajectory.getDt();
    for (size_t seg = 0; seg + clic_calib::SplineOrder <= trajectory.numKnots();
         ++seg) {
      const int64_t t_mid_ns =
          meta.t0_ns + static_cast<int64_t>((static_cast<double>(seg) + 0.5) *
                                            meta.dt_ns);
      if (t_mid_ns < meta.MinTimeNs() || t_mid_ns >= meta.MaxTimeNs()) {
        continue;
      }
      std::array<double*, clic_calib::SplineOrder> rot_knots{};
      std::array<double*, clic_calib::SplineOrder> pos_knots{};
      if (!GetActiveKnotPointers(trajectory, t_mid_ns, &rot_knots, &pos_knots)) {
        continue;
      }
      scope.owned_costs.push_back(std::make_unique<
          clic_calib::analytic_derivative::TrajectorySmoothnessFactor>(
          t_mid_ns, alpha_p, alpha_R, dt_s, meta));
      std::vector<double*> blocks;
      blocks.insert(blocks.end(), rot_knots.begin(), rot_knots.end());
      blocks.insert(blocks.end(), pos_knots.begin(), pos_knots.end());
      problem.AddResidualBlock(scope.owned_costs.back().get(), nullptr, blocks);
      register_so3_knots(rot_knots);
    }
  }
  ceres::Problem::EvaluateOptions eval_opts;
  eval_opts.apply_loss_function = false;
  eval_opts.num_threads = 1;
  ceres::CRSMatrix J;
  problem.Evaluate(eval_opts, nullptr, nullptr, nullptr, &J);
  return InformationFromCRS(J);
}

struct Stage2Result {
  ExtrinsicState lidar;
  ExtrinsicState camera;
  ceres::Solver::Summary summary;
  EigenReport fim_ext;
  bool converged = false;
};

void AddStage2FixedFactors(
    ceres::Problem* problem,
    std::vector<std::unique_ptr<ceres::CostFunction>>* owned,
    const SyntheticScenarioBundle& scenario,
    const BodyTrajectory& fixed_traj,
    const clic_calib::LeverArmConfig& levers,
    const RealisticNoiseSpec& noise,
    ExtrinsicState* lidar, ExtrinsicState* camera) {
  const PinholeIntrinsics K{600.0, 600.0, 320.0, 240.0};
  const RadtanDistortion dist;
  const double inv_sigma_r = 1.0 / noise.lidar_ranging_sigma_m;
  const double inv_sigma_pix = 1.0 / noise.camera_pixel_sigma;

  for (const auto& scan : scenario.lidar_obs) {
    const double t_bar = scan.t_sensor_;
    for (const auto& q_L : scan.points_L_) {
      owned->push_back(std::make_unique<FixedTrajSphereFactor>(
          fixed_traj, t_bar, q_L, levers.L_B_to_G, kSphereRadiusM, inv_sigma_r));
      problem->AddResidualBlock(
          owned->back().get(), nullptr, &lidar->t_d, lidar->q.coeffs().data(),
          lidar->t.data());
    }
  }

  for (const auto& det : scenario.tag_obs) {
    const double t_bar = det.t_sensor_;
    for (int c = 0; c < 4; ++c) {
      owned->push_back(std::make_unique<FixedTrajAprilTagFactor>(
          fixed_traj, t_bar, det.corners_pixel_[c], levers.L_B_to_G,
          levers.L_G_to_M.at(0), K, dist, inv_sigma_pix));
      problem->AddResidualBlock(
          owned->back().get(), nullptr, &camera->t_d,
          camera->q.coeffs().data(), camera->t.data());
    }
  }
}

Eigen::MatrixXd Stage2ExtrinsicInformation(
    const SyntheticScenarioBundle& scenario,
    const BodyTrajectory& fixed_traj,
    const ExtrinsicState& lidar, const ExtrinsicState& camera,
    const clic_calib::LeverArmConfig& levers,
    const RealisticNoiseSpec& noise) {
  ExtrinsicState lw = lidar;
  ExtrinsicState cw = camera;
  CeresSo3ProblemScope scope;
  AddStage2FixedFactors(scope.problem.get(), &scope.owned_costs, scenario,
                        fixed_traj, levers, noise, &lw, &cw);
  scope.SetLocalParamSO3(lw.q.coeffs().data());
  scope.SetLocalParamSO3(cw.q.coeffs().data());

  ceres::Problem::EvaluateOptions opts;
  opts.apply_loss_function = false;
  opts.num_threads = 1;
  ceres::CRSMatrix J;
  scope.problem->Evaluate(opts, nullptr, nullptr, nullptr, &J);
  return InformationFromCRS(J);
}

const char* FixedFextDofLabel(int idx) {
  static const char* kLabels[14] = {
      "t_d_L",   "LW_roll",  "LW_pitch", "LW_yaw",  "LW_tx",  "LW_ty",
      "LW_tz",   "t_d_C",    "CW_roll",  "CW_pitch", "CW_yaw", "CW_tx",
      "CW_ty",   "CW_tz",
  };
  if (idx < 0 || idx >= 14) {
    return "?";
  }
  return kLabels[idx];
}

int McDofFromFimDof(int fim_dof) {
  static const int kMap[14] = {12, 0, 1, 2, 3, 4, 5,
                               13, 6, 7, 8, 9, 10, 11};
  if (fim_dof < 0 || fim_dof >= 14) {
    return -1;
  }
  return kMap[fim_dof];
}

struct FixedFextEigenReport {
  Eigen::VectorXd evals_asc;
  Eigen::MatrixXd evecs_asc;
  double rank_tol = 0.0;
  int numerical_rank = 0;
  bool strictly_pd = false;
};

FixedFextEigenReport DiagnoseFixedFextEigen(const Eigen::MatrixXd& F) {
  FixedFextEigenReport out;
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(F);
  out.evals_asc = es.eigenvalues();
  out.evecs_asc = es.eigenvectors();
  out.strictly_pd = out.evals_asc.minCoeff() > 0.0;
  const double lambda_max = out.evals_asc.maxCoeff();
  out.rank_tol =
      std::max(1e-12, 1e-10 * std::max(std::abs(lambda_max), 1.0));
  for (int i = 0; i < out.evals_asc.size(); ++i) {
    if (out.evals_asc(i) > out.rank_tol) {
      ++out.numerical_rank;
    }
  }
  return out;
}

void PrintFixedFextEigenReport(const FixedFextEigenReport& rep) {
  std::cout << std::scientific << std::setprecision(16);
  std::cout << "  rank_tol (AnalyzeSymmetricF) = " << rep.rank_tol << "\n";
  std::cout << "  numerical_rank = " << rep.numerical_rank << "/14"
            << "  strictly_pd (λ_min>0) = " << (rep.strictly_pd ? "yes" : "NO")
            << "\n\n";
  std::cout << "  14 eigenvalues (ascending):\n";
  for (int i = 0; i < rep.evals_asc.size(); ++i) {
    const bool above = rep.evals_asc(i) > rep.rank_tol;
    std::cout << "    λ[" << i << "] = " << rep.evals_asc(i)
              << (above ? "  (ranked)" : "  **NEAR-NULL**") << "\n";
  }
  std::cout << "\n  3 near-null eigenvector DoF composition (|component|, fraction):\n";
  for (int m = 0; m < 3; ++m) {
    const Eigen::VectorXd v = rep.evecs_asc.col(m);
    const double norm = v.norm();
    std::cout << "    mode " << m << "  λ=" << rep.evals_asc(m) << "\n";
    int dominant = 0;
    double dom_abs = 0.0;
    for (int d = 0; d < 14; ++d) {
      const double frac = norm > 0.0 ? std::abs(v(d)) / norm : 0.0;
      if (std::abs(v(d)) > dom_abs) {
        dom_abs = std::abs(v(d));
        dominant = d;
      }
      if (frac >= 0.15) {
        std::cout << "      " << std::setw(10) << FixedFextDofLabel(d)
                  << "  |v|=" << std::abs(v(d)) << "  frac=" << frac << "\n";
      }
    }
    std::cout << "      dominant: " << FixedFextDofLabel(dominant)
              << "  (|v|=" << dom_abs << ")\n";
  }
  std::cout << std::defaultfloat;
}

/** Local index sets for Schur-marginalizing trajectory out of joint FIM. */
struct ProbeParameterLayout {
  int num_local = 0;
  std::vector<int> ext14;
  std::vector<int> traj_rest;
};

Eigen::MatrixXd ExtractSubmatrix(const Eigen::MatrixXd& F,
                                 const std::vector<int>& rows,
                                 const std::vector<int>& cols) {
  Eigen::MatrixXd out(rows.size(), cols.size());
  for (size_t i = 0; i < rows.size(); ++i) {
    for (size_t j = 0; j < cols.size(); ++j) {
      out(i, j) = F(rows[i], cols[j]);
    }
  }
  return out;
}

int SvdNumericalRank(const Eigen::MatrixXd& A, double* lambda_min_out = nullptr,
                     double* lambda_max_out = nullptr) {
  if (A.rows() == 0 || A.cols() == 0) {
    if (lambda_min_out) {
      *lambda_min_out = 0.0;
    }
    if (lambda_max_out) {
      *lambda_max_out = 0.0;
    }
    return 0;
  }
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(A);
  const Eigen::VectorXd evals = es.eigenvalues();
  const double lambda_max = evals.maxCoeff();
  const double lambda_min = evals.minCoeff();
  if (lambda_max_out) {
    *lambda_max_out = lambda_max;
  }
  if (lambda_min_out) {
    *lambda_min_out = lambda_min;
  }
  const double tol =
      1e-12 * std::max(static_cast<int>(A.rows()), static_cast<int>(A.cols())) *
      std::max(std::abs(lambda_max), 1.0);
  int rank = 0;
  for (int i = 0; i < evals.size(); ++i) {
    if (evals(i) > tol) {
      ++rank;
    }
  }
  return rank;
}

Eigen::MatrixXd SymmetricMatrixPinv(const Eigen::MatrixXd& A, int* rank_out,
                                    double* lambda_min_out = nullptr,
                                    double* lambda_max_out = nullptr) {
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(A);
  const Eigen::VectorXd evals = es.eigenvalues();
  const double lambda_max = evals.maxCoeff();
  const double lambda_min = evals.minCoeff();
  if (lambda_max_out) {
    *lambda_max_out = lambda_max;
  }
  if (lambda_min_out) {
    *lambda_min_out = lambda_min;
  }
  const double tol =
      1e-12 * std::max(static_cast<int>(A.rows()), static_cast<int>(A.cols())) *
      std::max(std::abs(lambda_max), 1.0);
  Eigen::VectorXd inv_evals = Eigen::VectorXd::Zero(evals.size());
  int rank = 0;
  for (int i = 0; i < evals.size(); ++i) {
    if (evals(i) > tol) {
      inv_evals(i) = 1.0 / evals(i);
      ++rank;
    }
  }
  if (rank_out) {
    *rank_out = rank;
  }
  return es.eigenvectors() * inv_evals.asDiagonal() *
         es.eigenvectors().transpose();
}

Eigen::MatrixXd SvdMoorePenrosePinv(const Eigen::MatrixXd& A, int* rank_out) {
  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      A, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Eigen::VectorXd& s = svd.singularValues();
  const double pinv_tol =
      1e-12 * std::max(static_cast<int>(A.rows()),
                       static_cast<int>(A.cols())) *
      (s.size() > 0 ? s(0) : 1.0);
  Eigen::VectorXd inv_s = Eigen::VectorXd::Zero(s.size());
  int rank = 0;
  for (int i = 0; i < s.size(); ++i) {
    if (s(i) > pinv_tol) {
      inv_s(i) = 1.0 / s(i);
      ++rank;
    }
  }
  if (rank_out) {
    *rank_out = rank;
  }
  return svd.matrixV() * inv_s.asDiagonal() * svd.matrixU().transpose();
}

ProbeParameterLayout BuildMarginalProbeLayout(
    ceres::Problem& prob, ExtrinsicState& lidar, ExtrinsicState& camera) {
  std::set<double*> ext_ptrs;
  ext_ptrs.insert(&lidar.t_d);
  ext_ptrs.insert(lidar.q.coeffs().data());
  ext_ptrs.insert(lidar.t.data());
  ext_ptrs.insert(&camera.t_d);
  ext_ptrs.insert(camera.q.coeffs().data());
  ext_ptrs.insert(camera.t.data());

  const std::vector<double*> ext_order = {
      &lidar.t_d,
      lidar.q.coeffs().data(),
      lidar.t.data(),
      &camera.t_d,
      camera.q.coeffs().data(),
      camera.t.data(),
  };

  std::vector<double*> blocks;
  prob.GetParameterBlocks(&blocks);
  std::map<double*, int> ptr_start;
  int cursor = 0;
  for (double* ptr : blocks) {
    ptr_start[ptr] = cursor;
    cursor += prob.ParameterBlockLocalSize(ptr);
  }

  ProbeParameterLayout layout;
  layout.num_local = cursor;
  for (double* ptr : ext_order) {
    const int start = ptr_start.at(ptr);
    const int ls = prob.ParameterBlockLocalSize(ptr);
    for (int k = 0; k < ls; ++k) {
      layout.ext14.push_back(start + k);
    }
  }
  for (double* ptr : blocks) {
    if (ext_ptrs.count(ptr) > 0) {
      continue;
    }
    const int start = ptr_start.at(ptr);
    const int ls = prob.ParameterBlockLocalSize(ptr);
    for (int k = 0; k < ls; ++k) {
      layout.traj_rest.push_back(start + k);
    }
  }
  return layout;
}

struct MarginalSchurReport {
  Eigen::MatrixXd F_marg = Eigen::MatrixXd::Zero(14, 14);
  int f_rr_dim = 0;
  int f_rr_svd_rank = 0;
  int f_rr_eigen_rank = 0;
  double f_rr_lambda_min = 0.0;
  double f_rr_lambda_max = 0.0;
  EigenReport f_marg;
};

int SymmetricSvdRankFromEigenvalues(const Eigen::VectorXd& evals,
                                    double lambda_max, int dim) {
  const double tol =
      1e-12 * std::max(dim, 1) * std::max(std::abs(lambda_max), 1.0);
  int rank = 0;
  for (int i = 0; i < evals.size(); ++i) {
    if (std::abs(evals(i)) > tol) {
      ++rank;
    }
  }
  return rank;
}

MarginalSchurReport SchurMarginalExtrinsicSvd(
    const Eigen::MatrixXd& F, const ProbeParameterLayout& layout) {
  MarginalSchurReport out;
  const auto& ext = layout.ext14;
  const auto& rest = layout.traj_rest;
  out.f_rr_dim = static_cast<int>(rest.size());
  if (rest.empty()) {
    out.F_marg = ExtractSubmatrix(F, ext, ext);
    out.f_marg = AnalyzeSymmetricF(out.F_marg);
    return out;
  }
  const Eigen::MatrixXd F_ee = ExtractSubmatrix(F, ext, ext);
  const Eigen::MatrixXd F_er = ExtractSubmatrix(F, ext, rest);
  const Eigen::MatrixXd F_re = ExtractSubmatrix(F, rest, ext);
  const Eigen::MatrixXd F_rr = ExtractSubmatrix(F, rest, rest);
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(F_rr);
  const Eigen::VectorXd evals = es.eigenvalues();
  out.f_rr_lambda_max = evals.maxCoeff();
  out.f_rr_lambda_min = evals.minCoeff();
  const double eig_tol =
      1e-12 * std::max(out.f_rr_dim, 1) *
      std::max(std::abs(out.f_rr_lambda_max), 1.0);
  out.f_rr_svd_rank =
      SymmetricSvdRankFromEigenvalues(evals, out.f_rr_lambda_max, out.f_rr_dim);
  out.F_marg = F_ee;
  out.f_rr_eigen_rank = 0;
  for (int i = 0; i < evals.size(); ++i) {
    if (evals(i) <= eig_tol) {
      continue;
    }
    ++out.f_rr_eigen_rank;
    const Eigen::VectorXd mode = es.eigenvectors().col(i);
    const Eigen::VectorXd w = F_er * mode;
    out.F_marg -= (1.0 / evals(i)) * w * (mode.transpose() * F_re);
  }
  out.F_marg = 0.5 * (out.F_marg + out.F_marg.transpose());
  out.f_marg = AnalyzeSymmetricF(out.F_marg);
  return out;
}

Eigen::VectorXd MarginalTheoStd14Svd(const Eigen::MatrixXd& F_marg) {
  Eigen::VectorXd stds = Eigen::VectorXd::Zero(14);
  int pinv_rank = 0;
  const Eigen::MatrixXd cov = SymmetricMatrixPinv(F_marg, &pinv_rank);
  for (int d = 0; d < 14; ++d) {
    if (cov(d, d) > 0.0) {
      stds(d) = std::sqrt(cov(d, d));
    }
  }
  return stds;
}

void AddStage1FactorsForJointFim(
    ceres::Problem* problem,
    std::vector<std::unique_ptr<ceres::CostFunction>>* owned,
    const SyntheticScenarioBundle& scenario, const BodyTrajectory& trajectory,
    const clic_calib::LeverArmConfig& levers, double alpha_p, double alpha_R,
    const std::function<void(const std::array<double*, clic_calib::SplineOrder>&)>&
        register_so3) {
  const SplineSegmentMeta<clic_calib::SplineOrder> meta =
      TrajectoryMeta(trajectory);
  for (const auto& m : scenario.rtk) {
    if (m.fix_status_ != RTKMeasurement::FixStatus::FIXED) {
      continue;
    }
    const int64_t t_ns = static_cast<int64_t>(m.t_world_ * clic_calib::S_TO_NS);
    std::array<double*, clic_calib::SplineOrder> rot_knots{};
    std::array<double*, clic_calib::SplineOrder> pos_knots{};
    if (!GetActiveKnotPointers(trajectory, t_ns, &rot_knots, &pos_knots)) {
      continue;
    }
    owned->push_back(
        std::make_unique<clic_calib::analytic_derivative::RTKPositionFactor>(
            t_ns, m.p_A_W_observed_, m.covariance_, levers.L_B_to_A, meta));
    std::vector<double*> blocks;
    blocks.insert(blocks.end(), rot_knots.begin(), rot_knots.end());
    blocks.insert(blocks.end(), pos_knots.begin(), pos_knots.end());
    problem->AddResidualBlock(owned->back().get(), nullptr, blocks);
    register_so3(rot_knots);
  }
  AddStage1AttitudeFactors(problem, owned, scenario.attitude_obs, trajectory,
                           meta, register_so3);
  if (alpha_p > 0.0 || alpha_R > 0.0) {
    const double dt_s = trajectory.getDt();
    for (size_t seg = 0; seg + clic_calib::SplineOrder <= trajectory.numKnots();
         ++seg) {
      const int64_t t_mid_ns =
          meta.t0_ns + static_cast<int64_t>((static_cast<double>(seg) + 0.5) *
                                            meta.dt_ns);
      if (t_mid_ns < meta.MinTimeNs() || t_mid_ns >= meta.MaxTimeNs()) {
        continue;
      }
      std::array<double*, clic_calib::SplineOrder> rot_knots{};
      std::array<double*, clic_calib::SplineOrder> pos_knots{};
      if (!GetActiveKnotPointers(trajectory, t_mid_ns, &rot_knots, &pos_knots)) {
        continue;
      }
      owned->push_back(std::make_unique<
          clic_calib::analytic_derivative::TrajectorySmoothnessFactor>(
          t_mid_ns, alpha_p, alpha_R, dt_s, meta));
      std::vector<double*> blocks;
      blocks.insert(blocks.end(), rot_knots.begin(), rot_knots.end());
      blocks.insert(blocks.end(), pos_knots.begin(), pos_knots.end());
      problem->AddResidualBlock(owned->back().get(), nullptr, blocks);
      register_so3(rot_knots);
    }
  }
}

void AddJointStage2Factors(
    ceres::Problem* problem,
    std::vector<std::unique_ptr<ceres::CostFunction>>* owned,
    const SyntheticScenarioBundle& scenario, BodyTrajectory& trajectory,
    const clic_calib::LeverArmConfig& levers, const RealisticNoiseSpec& noise,
    ExtrinsicState* lidar, ExtrinsicState* camera,
    const std::function<void(double*)>& register_so3) {
  const SplineSegmentMeta<clic_calib::SplineOrder> meta =
      TrajectoryMeta(trajectory);
  const PinholeIntrinsics K{600.0, 600.0, 320.0, 240.0};
  const RadtanDistortion dist;
  const Eigen::Vector3d L_B_to_G_M = levers.L_B_to_G + levers.L_G_to_M.at(0);

  for (const auto& scan : scenario.lidar_obs) {
    const int64_t bar_t_ns =
        static_cast<int64_t>(scan.t_sensor_ * clic_calib::S_TO_NS);
    const int64_t t_eval_ns =
        bar_t_ns - static_cast<int64_t>(lidar->t_d * clic_calib::S_TO_NS);
    std::array<double*, clic_calib::SplineOrder> rot_knots{};
    std::array<double*, clic_calib::SplineOrder> pos_knots{};
    if (!GetActiveKnotPointers(trajectory, t_eval_ns, &rot_knots, &pos_knots)) {
      continue;
    }
    for (const auto& q_L : scan.points_L_) {
      owned->push_back(std::make_unique<
          clic_calib::analytic_derivative::SphereImplicitFactor>(
          bar_t_ns, q_L, levers.L_B_to_G, kSphereRadiusM,
          noise.lidar_ranging_sigma_m, meta));
      std::vector<double*> blocks = {&lidar->t_d, lidar->q.coeffs().data(),
                                     lidar->t.data()};
      blocks.insert(blocks.end(), rot_knots.begin(), rot_knots.end());
      blocks.insert(blocks.end(), pos_knots.begin(), pos_knots.end());
      problem->AddResidualBlock(owned->back().get(), nullptr, blocks);
      register_so3(lidar->q.coeffs().data());
      for (double* q : rot_knots) {
        register_so3(q);
      }
    }
  }

  for (const auto& det : scenario.tag_obs) {
    const int64_t bar_t_ns =
        static_cast<int64_t>(det.t_sensor_ * clic_calib::S_TO_NS);
    const int64_t t_eval_ns =
        bar_t_ns - static_cast<int64_t>(camera->t_d * clic_calib::S_TO_NS);
    std::array<double*, clic_calib::SplineOrder> rot_knots{};
    std::array<double*, clic_calib::SplineOrder> pos_knots{};
    if (!GetActiveKnotPointers(trajectory, t_eval_ns, &rot_knots, &pos_knots)) {
      continue;
    }
    for (int c = 0; c < 4; ++c) {
      owned->push_back(std::make_unique<
          clic_calib::analytic_derivative::AprilTagReprojFactor>(
          bar_t_ns, det.corners_pixel_[c], L_B_to_G_M, K, dist,
          noise.camera_pixel_sigma, meta));
      std::vector<double*> blocks = {&camera->t_d, camera->q.coeffs().data(),
                                     camera->t.data()};
      blocks.insert(blocks.end(), rot_knots.begin(), rot_knots.end());
      blocks.insert(blocks.end(), pos_knots.begin(), pos_knots.end());
      problem->AddResidualBlock(owned->back().get(), nullptr, blocks);
      register_so3(camera->q.coeffs().data());
      for (double* q : rot_knots) {
        register_so3(q);
      }
    }
  }
}

struct JointFimResult {
  Eigen::MatrixXd F_full;
  ProbeParameterLayout layout;
  MarginalSchurReport marginal;
};

JointFimResult JointInformationWithMarginal(
    const SyntheticScenarioBundle& scenario, BodyTrajectory& trajectory,
    const ExtrinsicState& lidar, const ExtrinsicState& camera,
    const clic_calib::LeverArmConfig& levers, const RealisticNoiseSpec& noise,
    double alpha_p, double alpha_R, bool joint_refine_before_fim) {
  ExtrinsicState lw = lidar;
  ExtrinsicState cw = camera;
  CeresSo3ProblemScope scope;
  std::set<double*> so3_registered;
  auto register_so3_knots =
      [&](const std::array<double*, clic_calib::SplineOrder>& rot_knots) {
        for (double* q : rot_knots) {
          if (so3_registered.insert(q).second) {
            scope.SetLocalParamSO3(q);
          }
        }
      };
  auto register_so3_single = [&](double* q) {
    if (so3_registered.insert(q).second) {
      scope.SetLocalParamSO3(q);
    }
  };

  AddStage1FactorsForJointFim(scope.problem.get(), &scope.owned_costs, scenario,
                              trajectory, levers, alpha_p, alpha_R,
                              register_so3_knots);
  AddJointStage2Factors(scope.problem.get(), &scope.owned_costs, scenario,
                        trajectory, levers, noise, &lw, &cw,
                        register_so3_single);

  if (joint_refine_before_fim) {
    ceres::Solver::Options solve_opts;
    solve_opts.max_num_iterations = 40;
    solve_opts.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary joint_summary;
    ceres::Solve(solve_opts, scope.problem.get(), &joint_summary);
  }

  ceres::Problem::EvaluateOptions opts;
  opts.apply_loss_function = false;
  opts.num_threads =
      static_cast<int>(std::max(1u, std::thread::hardware_concurrency()));
  ceres::CRSMatrix J;
  scope.problem->Evaluate(opts, nullptr, nullptr, nullptr, &J);

  JointFimResult out;
  out.F_full = InformationFromCRS(J);
  out.layout = BuildMarginalProbeLayout(*scope.problem, lw, cw);
  out.marginal = SchurMarginalExtrinsicSvd(out.F_full, out.layout);
  return out;
}

Stage2Result SolveStage2Extrinsics(
    const SyntheticScenarioBundle& scenario,
    const BodyTrajectory& fixed_traj,
    const clic_calib::LeverArmConfig& levers,
    const RealisticNoiseSpec& noise,
    const CoarseExtrinsicInit& coarse,
    double t_d_max_abs_s) {
  Stage2Result out;
  out.lidar.q = coarse.T_LW.so3().unit_quaternion();
  out.lidar.t = coarse.T_LW.translation();
  out.lidar.t_d = coarse.t_d_L_s;
  out.camera.q = coarse.T_CW.so3().unit_quaternion();
  out.camera.t = coarse.T_CW.translation();
  out.camera.t_d = coarse.t_d_C_s;

  CeresSo3ProblemScope scope;
  AddStage2FixedFactors(scope.problem.get(), &scope.owned_costs, scenario,
                        fixed_traj, levers, noise, &out.lidar, &out.camera);
  scope.SetLocalParamSO3(out.lidar.q.coeffs().data());
  scope.SetLocalParamSO3(out.camera.q.coeffs().data());

  scope.problem->SetParameterLowerBound(&out.lidar.t_d, 0, -t_d_max_abs_s);
  scope.problem->SetParameterUpperBound(&out.lidar.t_d, 0, t_d_max_abs_s);
  scope.problem->SetParameterLowerBound(&out.camera.t_d, 0, -t_d_max_abs_s);
  scope.problem->SetParameterUpperBound(&out.camera.t_d, 0, t_d_max_abs_s);

  ceres::Solver::Options opts;
  opts.max_num_iterations = 500;
  opts.minimizer_progress_to_stdout = false;
  ceres::Solve(opts, scope.problem.get(), &out.summary);
  out.converged = out.summary.IsSolutionUsable();

  if (out.converged) {
    const Eigen::MatrixXd F = Stage2ExtrinsicInformation(
        scenario, fixed_traj, out.lidar, out.camera, levers, noise);
    out.fim_ext = AnalyzeSymmetricF(F);
  }
  return out;
}

struct LadderExtrinsicStats {
  RunningStats lw_rot_deg;
  RunningStats lw_trans_mm;
  RunningStats lw_pitch_deg;
  RunningStats cw_rot_deg;
  RunningStats cw_trans_mm;
  RunningStats cw_pitch_deg;
  int converged = 0;
  int cm_success = 0;
  int wrong_basin = 0;
};

enum class LadderTraj { kGt, kStage1 };
enum class LadderInit { kOracle, kClosedForm, kCoarse };

struct LadderConfigSpec {
  const char* label;
  LadderTraj traj;
  LadderInit init;
  bool remap_tags = false;
};

CoarseExtrinsicInit OracleExtrinsicInit(const SyntheticScenarioBundle& sc) {
  CoarseExtrinsicInit out;
  out.T_LW = sc.gt.T_LW;
  out.T_CW = sc.gt.T_CW;
  out.t_d_L_s = sc.gt.t_d_L_s;
  out.t_d_C_s = sc.gt.t_d_C_s;
  return out;
}

bool IsCmLevelExtrinsic(const ExtrinsicSixDofErrors& lw_e,
                        const ExtrinsicSixDofErrors& cw_e) {
  return lw_e.trans_err_m.norm() * 1e3 < kCmTransMm &&
         cw_e.trans_err_m.norm() * 1e3 < kCmTransMm &&
         std::abs(lw_e.rot_err_rad.y()) * 180.0 / M_PI < kCmPitchDeg;
}

bool IsWrongBasinExtrinsic(const ExtrinsicSixDofErrors& lw_e,
                           const ExtrinsicSixDofErrors& cw_e) {
  return lw_e.trans_err_m.norm() * 1e3 > kWrongBasinTransMm ||
         cw_e.trans_err_m.norm() * 1e3 > kWrongBasinTransMm;
}

LadderExtrinsicStats RunBasinLadderConfig(
    const LadderConfigSpec& cfg,
    const clic_calib::LeverArmConfig& levers,
    const SplineConfig& spline_cfg,
    const CoarseExtrinsicInit& t_d_prior,
    const RealisticNoiseSpec& noise) {
  LadderExtrinsicStats stats;
  const PinholeIntrinsics K{600.0, 600.0, 320.0, 240.0};
  const RadtanDistortion dist;
  const Eigen::Vector3d L_G_to_M = levers.L_G_to_M.at(0);

  for (int i = 0; i < kNumSeeds; ++i) {
    const uint32_t seed = kSeedBase + static_cast<uint32_t>(i);
    SyntheticScenarioBundle sc =
        BuildNearFieldFimNoisyScenarioWithAttitude(seed, noise);

    const BodyTrajectory* traj = nullptr;
    std::shared_ptr<BodyTrajectory> traj_owned;
    if (cfg.traj == LadderTraj::kGt) {
      traj = &sc.gt_traj;
    } else {
      const Stage1Result s1 = FitStage1WithAttitude(
          sc, levers, spline_cfg, kStage1AlphaP, kStage1AlphaR);
      if (!s1.summary.IsSolutionUsable()) {
        continue;
      }
      traj_owned = s1.trajectory;
      traj = traj_owned.get();
    }

    if (cfg.remap_tags) {
      sc.tag_obs = SynthesizeTagObsForTrajectory(
          sc.tag_obs, *traj, sc.gt.T_CW, levers.L_B_to_G, L_G_to_M,
          t_d_prior.t_d_C_s, K, dist, noise, seed);
    }

    CoarseExtrinsicInit init;
    if (cfg.init == LadderInit::kOracle) {
      init = OracleExtrinsicInit(sc);
    } else if (cfg.init == LadderInit::kClosedForm) {
      init = MakeGeometricExtrinsicInit(*traj, sc.lidar_obs, sc.tag_obs, levers,
                                        kSphereRadiusM, t_d_prior)
                 .init;
    } else {
      init = MakeCoarseExtrinsicInitPerturbed(
          sc.gt.T_LW, sc.gt.T_CW, sc.gt.t_d_L_s, sc.gt.t_d_C_s, seed);
    }

    const Stage2Result s2 = SolveStage2Extrinsics(
        sc, *traj, levers, noise, init, spline_cfg.t_d_max_abs_s);
    if (!s2.converged) {
      continue;
    }
    ++stats.converged;

    const ExtrinsicSixDofErrors lw_e =
        ExtrinsicError(StateToSE3(s2.lidar), sc.gt.T_LW);
    const ExtrinsicSixDofErrors cw_e =
        ExtrinsicError(StateToSE3(s2.camera), sc.gt.T_CW);
    stats.lw_rot_deg.Push(lw_e.rot_err_rad.norm() * 180.0 / M_PI);
    stats.lw_trans_mm.Push(lw_e.trans_err_m.norm() * 1e3);
    stats.lw_pitch_deg.Push(lw_e.rot_err_rad.y() * 180.0 / M_PI);
    stats.cw_rot_deg.Push(cw_e.rot_err_rad.norm() * 180.0 / M_PI);
    stats.cw_trans_mm.Push(cw_e.trans_err_m.norm() * 1e3);
    stats.cw_pitch_deg.Push(cw_e.rot_err_rad.y() * 180.0 / M_PI);

    if (IsCmLevelExtrinsic(lw_e, cw_e)) {
      ++stats.cm_success;
    }
    if (IsWrongBasinExtrinsic(lw_e, cw_e)) {
      ++stats.wrong_basin;
    }
  }
  return stats;
}

void PrintLadderConfigBlock(const char* label, const LadderExtrinsicStats& s) {
  std::cout << "\n--- Config " << label << " (after Stage-2 polish, N="
            << kNumSeeds << ") ---\n";
  std::cout << "  converged=" << s.converged << "/" << kNumSeeds
            << "  cm-level=" << s.cm_success << "/" << kNumSeeds
            << "  wrong-basin=" << s.wrong_basin << "/" << kNumSeeds << "\n";
  PrintBiasStd("LW rot |err|", s.lw_rot_deg, "deg", 4);
  PrintBiasStd("LW pitch err", s.lw_pitch_deg, "deg", 4);
  PrintBiasStd("|LW trans| err", s.lw_trans_mm, "mm", 3);
  PrintBiasStd("CW rot |err|", s.cw_rot_deg, "deg", 4);
  PrintBiasStd("CW pitch err", s.cw_pitch_deg, "deg", 4);
  PrintBiasStd("|CW trans| err", s.cw_trans_mm, "mm", 3);
}

double SphereLateralErrorMm(const BodyTrajectory& est, const BodyTrajectory& gt,
                            double t, const Eigen::Vector3d& L_B_to_G);

struct YawSweepRow {
  double sigma_yaw_deg = 0.0;
  RunningStats lw_trans_mm;
  RunningStats cw_trans_mm;
  RunningStats lw_rot_deg;
  RunningStats cw_rot_deg;
  RunningStats lw_yaw_deg;
  RunningStats cw_yaw_deg;
  RunningStats lw_lateral_mm;
  RunningStats traj_yaw_deg;
  RunningStats pG_lat_mm;
  int converged = 0;
  int cm_success = 0;
};

YawSweepRow RunYawSensitivityAtSigma(
    double sigma_yaw_deg,
    const clic_calib::LeverArmConfig& levers,
    const SplineConfig& spline_cfg,
    const CoarseExtrinsicInit& t_d_prior,
    const RealisticNoiseSpec& noise,
    double t_end) {
  YawSweepRow row;
  row.sigma_yaw_deg = sigma_yaw_deg;
  AttitudeNoiseSpec att;
  att.sigma_roll_deg = 0.2;
  att.sigma_pitch_deg = 0.2;
  att.sigma_yaw_deg = sigma_yaw_deg;

  const PinholeIntrinsics K{600.0, 600.0, 320.0, 240.0};
  const RadtanDistortion dist;
  const Eigen::Vector3d L_G_to_M = levers.L_G_to_M.at(0);

  for (int i = 0; i < kNumSeeds; ++i) {
    const uint32_t seed = kSeedBase + static_cast<uint32_t>(i);
    SyntheticScenarioBundle sc =
        BuildNearFieldFimNoisyScenarioWithAttitude(seed, noise, att);
    const Stage1Result s1 = FitStage1WithAttitude(
        sc, levers, spline_cfg, kStage1AlphaP, kStage1AlphaR);
    if (!s1.summary.IsSolutionUsable()) {
      continue;
    }
    sc.tag_obs = SynthesizeTagObsForTrajectory(
        sc.tag_obs, *s1.trajectory, sc.gt.T_CW, levers.L_B_to_G, L_G_to_M,
        t_d_prior.t_d_C_s, K, dist, noise, seed);
    const GeometricInitReport geo = MakeGeometricExtrinsicInit(
        *s1.trajectory, sc.lidar_obs, sc.tag_obs, levers, kSphereRadiusM,
        t_d_prior);
    const Stage2Result s2 = SolveStage2Extrinsics(
        sc, *s1.trajectory, levers, noise, geo.init, spline_cfg.t_d_max_abs_s);
    if (!s2.converged) {
      continue;
    }
    ++row.converged;

    const ExtrinsicSixDofErrors lw_e =
        ExtrinsicError(StateToSE3(s2.lidar), sc.gt.T_LW);
    const ExtrinsicSixDofErrors cw_e =
        ExtrinsicError(StateToSE3(s2.camera), sc.gt.T_CW);
    row.lw_trans_mm.Push(lw_e.trans_err_m.norm() * 1e3);
    row.cw_trans_mm.Push(cw_e.trans_err_m.norm() * 1e3);
    row.lw_rot_deg.Push(lw_e.rot_err_rad.norm() * 180.0 / M_PI);
    row.cw_rot_deg.Push(cw_e.rot_err_rad.norm() * 180.0 / M_PI);
    row.lw_yaw_deg.Push(lw_e.rot_err_rad.z() * 180.0 / M_PI);
    row.cw_yaw_deg.Push(cw_e.rot_err_rad.z() * 180.0 / M_PI);
    row.lw_lateral_mm.Push(lw_e.trans_err_m.head<2>().norm() * 1e3);
    row.traj_yaw_deg.Push(
        TrajRpyRmsDeg(*s1.trajectory, sc.gt_traj, t_end).yaw);
    for (double t = 1.0; t <= t_end - 1.0; t += 5.0) {
      row.pG_lat_mm.Push(SphereLateralErrorMm(*s1.trajectory, sc.gt_traj, t,
                                              levers.L_B_to_G));
    }
    if (IsCmLevelExtrinsic(lw_e, cw_e)) {
      ++row.cm_success;
    }
  }
  return row;
}

void PrintYawSweepTable(const std::vector<YawSweepRow>& rows) {
  std::cout << std::setw(8) << "σ_yaw" << std::setw(12) << "|LW|mm"
            << std::setw(12) << "|CW|mm" << std::setw(10) << "LW yaw°"
            << std::setw(12) << "LW lat mm" << std::setw(11) << "traj_yaw°"
            << std::setw(12) << "p_G lat mm" << std::setw(10) << "cm-ok"
            << "\n";
  for (const auto& r : rows) {
    std::cout << std::fixed << std::setprecision(3);
    std::cout << std::setw(8) << r.sigma_yaw_deg << std::setw(12)
              << r.lw_trans_mm.mean << std::setw(12) << r.cw_trans_mm.mean
              << std::setw(10) << r.lw_yaw_deg.mean << std::setw(12)
              << r.lw_lateral_mm.mean << std::setw(11) << r.traj_yaw_deg.mean
              << std::setw(12) << r.pG_lat_mm.mean << std::setw(7)
              << r.cm_success << "/" << kNumSeeds << "\n";
  }
}

bool EvaluateGate4YawVerdict(const std::vector<YawSweepRow>& rows,
                             std::string* verdict_msg) {
  if (rows.size() < 2 || !verdict_msg) {
    *verdict_msg = "insufficient sweep points";
    return false;
  }
  const YawSweepRow& lo = rows.front();
  const YawSweepRow& hi = rows.back();
  const double lw_delta = hi.lw_trans_mm.mean - lo.lw_trans_mm.mean;
  const double lat_delta = hi.pG_lat_mm.mean - lo.pG_lat_mm.mean;
  const double rel_rise =
      lo.lw_trans_mm.mean > 1e-3
          ? (hi.lw_trans_mm.mean - lo.lw_trans_mm.mean) / lo.lw_trans_mm.mean
          : hi.lw_trans_mm.mean;

  // Flat: cm-level at all σ_yaw and modest growth from 0.3°→2.0°.
  const bool all_cm = std::all_of(rows.begin(), rows.end(), [](const YawSweepRow& r) {
    return r.cm_success >= 45 && r.lw_trans_mm.mean < kCmTransMm &&
           r.cw_trans_mm.mean < kCmTransMm;
  });
  const bool flat_curve =
      all_cm && hi.lw_trans_mm.mean < kCmTransMm && lw_delta < 15.0 &&
      lat_delta < 20.0 && rel_rise < 0.75;

  if (flat_curve) {
    *verdict_msg =
        "FLAT — extrinsic accuracy stays cm-level across σ_yaw; simple two-stage "
        "suffices (no AprilTag yaw refinement needed).";
    return true;
  }
  *verdict_msg =
      "RISING — |T_LW| / lateral error grows with σ_yaw; AprilTag-constrained yaw "
      "refinement is warranted (next architecture; not implemented here).";
  return false;
}

struct FimMcSampleStats {
  RunningStats lw_roll;
  RunningStats lw_pitch;
  RunningStats lw_yaw;
  RunningStats lw_tx;
  RunningStats lw_ty;
  RunningStats lw_tz;
  RunningStats cw_roll;
  RunningStats cw_pitch;
  RunningStats cw_yaw;
  RunningStats cw_tx;
  RunningStats cw_ty;
  RunningStats cw_tz;
  RunningStats t_d_L;
  RunningStats t_d_C;
  int converged = 0;
};

bool SolveClosedFormStage2AtSeed(
    uint32_t seed,
    const clic_calib::LeverArmConfig& levers,
    const SplineConfig& spline_cfg,
    const CoarseExtrinsicInit& t_d_prior,
    const RealisticNoiseSpec& noise,
    double t_d_max_abs_s,
    SyntheticScenarioBundle* sc_out,
    std::shared_ptr<BodyTrajectory>* traj_out,
    Stage2Result* s2_out) {
  if (!sc_out || !traj_out || !s2_out) {
    return false;
  }
  *sc_out = BuildNearFieldFimNoisyScenarioWithAttitude(seed, noise);
  const Stage1Result s1 = FitStage1WithAttitude(
      *sc_out, levers, spline_cfg, kStage1AlphaP, kStage1AlphaR);
  if (!s1.summary.IsSolutionUsable()) {
    return false;
  }
  *traj_out = s1.trajectory;
  const PinholeIntrinsics K{600.0, 600.0, 320.0, 240.0};
  const RadtanDistortion dist;
  sc_out->tag_obs = SynthesizeTagObsForTrajectory(
      sc_out->tag_obs, *s1.trajectory, sc_out->gt.T_CW, levers.L_B_to_G,
      levers.L_G_to_M.at(0), t_d_prior.t_d_C_s, K, dist, noise, seed);
  const GeometricInitReport geo = MakeGeometricExtrinsicInit(
      *s1.trajectory, sc_out->lidar_obs, sc_out->tag_obs, levers,
      kSphereRadiusM, t_d_prior);
  CoarseExtrinsicInit init = geo.init;
  // Nominal yaml t_d (matches world-time pairing); init t_d:=0 pins MC at bounds.
  init.t_d_L_s = t_d_prior.t_d_L_s;
  init.t_d_C_s = t_d_prior.t_d_C_s;
  *s2_out = SolveStage2Extrinsics(*sc_out, *s1.trajectory, levers, noise, init,
                                  t_d_max_abs_s);
  return s2_out->converged;
}

void PushFimMcSample(FimMcSampleStats* out, const SyntheticScenarioBundle& sc,
                     const Stage2Result& s2) {
  const ExtrinsicSixDofErrors lw =
      ExtrinsicError(StateToSE3(s2.lidar), sc.gt.T_LW);
  const ExtrinsicSixDofErrors cw =
      ExtrinsicError(StateToSE3(s2.camera), sc.gt.T_CW);
  out->lw_roll.Push(lw.rot_err_rad.x());
  out->lw_pitch.Push(lw.rot_err_rad.y());
  out->lw_yaw.Push(lw.rot_err_rad.z());
  out->lw_tx.Push(lw.trans_err_m.x());
  out->lw_ty.Push(lw.trans_err_m.y());
  out->lw_tz.Push(lw.trans_err_m.z());
  out->cw_roll.Push(cw.rot_err_rad.x());
  out->cw_pitch.Push(cw.rot_err_rad.y());
  out->cw_yaw.Push(cw.rot_err_rad.z());
  out->cw_tx.Push(cw.trans_err_m.x());
  out->cw_ty.Push(cw.trans_err_m.y());
  out->cw_tz.Push(cw.trans_err_m.z());
  out->t_d_L.Push(s2.lidar.t_d - sc.gt.t_d_L_s);
  out->t_d_C.Push(s2.camera.t_d - sc.gt.t_d_C_s);
  ++out->converged;
}

bool SolveClosedFormStage2WithNoiseSeeds(
    uint32_t seed_traj, uint32_t seed_obs,
    const clic_calib::LeverArmConfig& levers, const SplineConfig& spline_cfg,
    const CoarseExtrinsicInit& t_d_prior, const RealisticNoiseSpec& noise,
    double t_d_max_abs_s, SyntheticScenarioBundle* sc_out,
    std::shared_ptr<BodyTrajectory>* traj_out, Stage2Result* s2_out) {
  if (!sc_out || !traj_out || !s2_out) {
    return false;
  }
  *sc_out = BuildNearFieldFimNoisyScenarioWithAttitude(seed_traj, seed_obs, noise);
  const Stage1Result s1 = FitStage1WithAttitude(
      *sc_out, levers, spline_cfg, kStage1AlphaP, kStage1AlphaR);
  if (!s1.summary.IsSolutionUsable()) {
    return false;
  }
  *traj_out = s1.trajectory;
  const PinholeIntrinsics K{600.0, 600.0, 320.0, 240.0};
  const RadtanDistortion dist;
  sc_out->tag_obs = SynthesizeTagObsForTrajectory(
      sc_out->tag_obs, *s1.trajectory, sc_out->gt.T_CW, levers.L_B_to_G,
      levers.L_G_to_M.at(0), t_d_prior.t_d_C_s, K, dist, noise, seed_obs);
  const GeometricInitReport geo = MakeGeometricExtrinsicInit(
      *s1.trajectory, sc_out->lidar_obs, sc_out->tag_obs, levers,
      kSphereRadiusM, t_d_prior);
  CoarseExtrinsicInit init = geo.init;
  init.t_d_L_s = t_d_prior.t_d_L_s;
  init.t_d_C_s = t_d_prior.t_d_C_s;
  *s2_out = SolveStage2Extrinsics(*sc_out, *s1.trajectory, levers, noise, init,
                                  t_d_max_abs_s);
  return s2_out->converged;
}

bool SolveStage2FixedTrajArm(
    uint32_t obs_seed, const BodyTrajectory& fixed_traj,
    const clic_calib::LeverArmConfig& levers, const SplineConfig& spline_cfg,
    const CoarseExtrinsicInit& t_d_prior, const RealisticNoiseSpec& noise,
    double t_d_max_abs_s, SyntheticScenarioBundle* sc_out, Stage2Result* s2_out) {
  if (!sc_out || !s2_out) {
    return false;
  }
  *sc_out =
      BuildNearFieldFimNoisyScenarioWithAttitude(kRepSeed, obs_seed, noise);
  const PinholeIntrinsics K{600.0, 600.0, 320.0, 240.0};
  const RadtanDistortion dist;
  sc_out->tag_obs = SynthesizeTagObsForTrajectory(
      sc_out->tag_obs, fixed_traj, sc_out->gt.T_CW, levers.L_B_to_G,
      levers.L_G_to_M.at(0), t_d_prior.t_d_C_s, K, dist, noise, obs_seed);
  const GeometricInitReport geo = MakeGeometricExtrinsicInit(
      fixed_traj, sc_out->lidar_obs, sc_out->tag_obs, levers, kSphereRadiusM,
      t_d_prior);
  CoarseExtrinsicInit init = geo.init;
  init.t_d_L_s = t_d_prior.t_d_L_s;
  init.t_d_C_s = t_d_prior.t_d_C_s;
  *s2_out = SolveStage2Extrinsics(*sc_out, fixed_traj, levers, noise, init,
                                  t_d_max_abs_s);
  return s2_out->converged;
}

struct UqDecompositionMcResult {
  FimMcSampleStats total;
  FimMcSampleStats fixed_traj;
  FimMcSampleStats traj_prop;
  bool rep_traj_ok = false;
};

constexpr double kUqAdditiveLo = 0.65;
constexpr double kUqAdditiveHi = 1.35;
constexpr double kDecompLo = 0.8;
constexpr double kDecompHi = 1.25;

struct DecompositionGateReport {
  int in_band = 0;
  int trans_in_band = 0;
  std::vector<std::string> failed_dofs;
};

double SampleVar(const RunningStats& s) {
  const double st = s.Std();
  return st * st;
}

const char* kExtrinsicDofNames[14] = {
    "LW_roll", "LW_pitch", "LW_yaw", "LW_tx", "LW_ty", "LW_tz",
    "CW_roll", "CW_pitch", "CW_yaw", "CW_tx", "CW_ty", "CW_tz",
    "t_d_L",   "t_d_C"};

const RunningStats* McArmDofConst(const FimMcSampleStats& arm, int dof) {
  switch (dof) {
    case 0: return &arm.lw_roll;
    case 1: return &arm.lw_pitch;
    case 2: return &arm.lw_yaw;
    case 3: return &arm.lw_tx;
    case 4: return &arm.lw_ty;
    case 5: return &arm.lw_tz;
    case 6: return &arm.cw_roll;
    case 7: return &arm.cw_pitch;
    case 8: return &arm.cw_yaw;
    case 9: return &arm.cw_tx;
    case 10: return &arm.cw_ty;
    case 11: return &arm.cw_tz;
    case 12: return &arm.t_d_L;
    case 13: return &arm.t_d_C;
    default: return nullptr;
  }
}

RunningStats* McArmDof(FimMcSampleStats* arm, int dof) {
  return const_cast<RunningStats*>(McArmDofConst(*arm, dof));
}

DecompositionGateReport PrintDecompositionValidationTable(
    const UqDecompositionMcResult& r, double lo, double hi) {
  DecompositionGateReport rep;
  std::cout << std::scientific << std::setprecision(4);
  std::cout << "\n  14-DoF variance decomposition (diag Σ):\n";
  std::cout << "  DoF          Var_fixed    Var_traj     Var_sum"
            << "      Var_total    ratio(sum/tot)  traj/fixed\n";
  for (int d = 0; d < 14; ++d) {
    const double vf = SampleVar(*McArmDofConst(r.fixed_traj, d));
    const double vp = SampleVar(*McArmDofConst(r.traj_prop, d));
    const double vs = vf + vp;
    const double vt = SampleVar(*McArmDofConst(r.total, d));
    const double ratio = vt > 1e-24 ? vs / vt : std::numeric_limits<double>::quiet_NaN();
    const double decouple = vf > 1e-24 ? vp / vf : std::numeric_limits<double>::quiet_NaN();
    const bool pass = std::isfinite(ratio) && ratio >= lo && ratio <= hi;
    if (pass) {
      ++rep.in_band;
      if (d == 3 || d == 4 || d == 5 || d == 9 || d == 10) {
        ++rep.trans_in_band;
      }
    } else {
      rep.failed_dofs.push_back(kExtrinsicDofNames[d]);
    }
    std::cout << "  " << std::setw(10) << std::left << kExtrinsicDofNames[d]
              << std::right << std::setw(12) << vf << std::setw(12) << vp
              << std::setw(12) << vs << std::setw(12) << vt << std::setw(16)
              << ratio << std::setw(12) << decouple << (pass ? "  OK" : "")
              << "\n";
  }
  std::cout << std::fixed;
  std::cout << "\n  Per-DoF std (mrad / mm / 0.01 ms):\n";
  std::cout << "  DoF          std_fixed    std_traj     std_sum      std_total\n";
  for (int d = 0; d < 14; ++d) {
    const double sf = McArmDofConst(r.fixed_traj, d)->Std();
    const double sp = McArmDofConst(r.traj_prop, d)->Std();
    const double st = McArmDofConst(r.total, d)->Std();
    const double ss = std::sqrt(SampleVar(*McArmDofConst(r.fixed_traj, d)) +
                                SampleVar(*McArmDofConst(r.traj_prop, d)));
    double scale = 1e3;
    if (d % 6 < 3) {
      scale = 180e3 / M_PI;
    } else if (d >= 12) {
      scale = 1e5;
    }
    std::cout << "  " << std::setw(10) << std::left << kExtrinsicDofNames[d]
              << std::right << std::setprecision(3) << std::setw(12) << sf * scale
              << std::setw(12) << sp * scale << std::setw(12) << ss * scale
              << std::setw(12) << st * scale << "\n";
  }
  std::cout << "\n  MC sampling note: independent-arm variance SE ~ sqrt(2/N)"
            << " ≈ 14% at N=100; band [" << lo << ", " << hi << "].\n";
  return rep;
}

int PrintUqVarianceDecompositionTable(const UqDecompositionMcResult& r) {
  std::cout << std::scientific << std::setprecision(4);
  std::cout << "\n  Per-DoF variance decomposition (diag Σ):\n";
  std::cout << "  DoF          Var_total    Var_fixed    Var_traj     Var_sum"
            << "      ratio\n";
  int in_band = 0;
  for (int d = 0; d < 14; ++d) {
    const double vt = SampleVar(*McArmDofConst(r.total, d));
    const double vf = SampleVar(*McArmDofConst(r.fixed_traj, d));
    const double vp = SampleVar(*McArmDofConst(r.traj_prop, d));
    const double vs = vf + vp;
    const double ratio = vt > 1e-24 ? vs / vt : std::numeric_limits<double>::quiet_NaN();
    const bool pass =
        std::isfinite(ratio) && ratio >= kUqAdditiveLo && ratio <= kUqAdditiveHi;
    if (pass) {
      ++in_band;
    }
    std::cout << "  " << std::setw(10) << std::left << kExtrinsicDofNames[d]
              << std::right << std::setw(12) << vt << std::setw(12) << vf
              << std::setw(12) << vp << std::setw(12) << vs << std::setw(12)
              << ratio << (pass ? "  OK" : "") << "\n";
  }
  std::cout << std::fixed;
  std::cout << "\n  Per-DoF std (units: mrad / mm / 0.01 ms):\n";
  std::cout << "  DoF          std_total    std_fixed    std_traj     std_sum\n";
  for (int d = 0; d < 14; ++d) {
    const double st = McArmDofConst(r.total, d)->Std();
    const double sf = McArmDofConst(r.fixed_traj, d)->Std();
    const double sp = McArmDofConst(r.traj_prop, d)->Std();
    const double ss = std::sqrt(SampleVar(*McArmDofConst(r.fixed_traj, d)) +
                                SampleVar(*McArmDofConst(r.traj_prop, d)));
    double scale = 1e3;
    if (d % 6 < 3) {
      scale = 180e3 / M_PI;
    } else if (d >= 12) {
      scale = 1e5;
    }
    std::cout << "  " << std::setw(10) << std::left << kExtrinsicDofNames[d]
              << std::right << std::setprecision(3) << std::setw(12) << st * scale
              << std::setw(12) << sf * scale << std::setw(12) << sp * scale
              << std::setw(12) << ss * scale << "\n";
  }
  return in_band;
}

UqDecompositionMcResult CollectUqDecompositionMc(
    const clic_calib::LeverArmConfig& levers, const SplineConfig& spline_cfg,
    const CoarseExtrinsicInit& t_d_prior, const RealisticNoiseSpec& noise,
    int num_seeds) {
  UqDecompositionMcResult out;
  SyntheticScenarioBundle rep_sc =
      BuildNearFieldFimNoisyScenarioWithAttitude(kRepSeed, kRepSeed, noise);
  const Stage1Result s1_rep = FitStage1WithAttitude(
      rep_sc, levers, spline_cfg, kStage1AlphaP, kStage1AlphaR);
  out.rep_traj_ok = s1_rep.summary.IsSolutionUsable();
  if (!out.rep_traj_ok) {
    return out;
  }

  for (int i = 0; i < num_seeds; ++i) {
    const uint32_t seed = kSeedBase + static_cast<uint32_t>(i);
    SyntheticScenarioBundle sc;
    std::shared_ptr<BodyTrajectory> traj;
    Stage2Result s2;

    if (SolveClosedFormStage2WithNoiseSeeds(
            seed, seed, levers, spline_cfg, t_d_prior, noise,
            spline_cfg.t_d_max_abs_s, &sc, &traj, &s2)) {
      PushFimMcSample(&out.total, sc, s2);
    }

    if (SolveStage2FixedTrajArm(seed, *s1_rep.trajectory, levers, spline_cfg,
                                t_d_prior, noise, spline_cfg.t_d_max_abs_s, &sc,
                                &s2)) {
      PushFimMcSample(&out.fixed_traj, sc, s2);
    }

    if (SolveClosedFormStage2WithNoiseSeeds(
            seed, kRepSeed, levers, spline_cfg, t_d_prior, noise,
            spline_cfg.t_d_max_abs_s, &sc, &traj, &s2)) {
      PushFimMcSample(&out.traj_prop, sc, s2);
    }

    if ((i + 1) % 10 == 0 || i + 1 == num_seeds) {
      std::cout << "  UQ MC progress: " << (i + 1) << "/" << num_seeds
                << "  total=" << out.total.converged
                << " fixed=" << out.fixed_traj.converged
                << " traj=" << out.traj_prop.converged << "\n"
                << std::flush;
    }
  }
  return out;
}

std::vector<int> KnotTangentIndices(ceres::Problem& problem,
                                    const BodyTrajectory& traj, int knot) {
  std::map<double*, int> start;
  int cursor = 0;
  std::vector<double*> blocks;
  problem.GetParameterBlocks(&blocks);
  for (double* ptr : blocks) {
    start[ptr] = cursor;
    cursor += problem.ParameterBlockLocalSize(ptr);
  }
  double* rot_ptr = const_cast<SO3d&>(traj.getKnotSO3(knot)).data();
  double* pos_ptr = const_cast<Eigen::Vector3d&>(traj.getKnotPos(knot)).data();
  std::vector<int> idx;
  if (start.count(rot_ptr)) {
    for (int k = 0; k < 3; ++k) {
      idx.push_back(start.at(rot_ptr) + k);
    }
  }
  if (start.count(pos_ptr)) {
    for (int k = 0; k < 3; ++k) {
      idx.push_back(start.at(pos_ptr) + k);
    }
  }
  return idx;
}

Eigen::MatrixXd ExtractCovarianceBlock(const Eigen::MatrixXd& F,
                                       const std::vector<int>& idx) {
  const int n = static_cast<int>(idx.size());
  Eigen::MatrixXd F_sub(n, n);
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      F_sub(i, j) = F(idx[i], idx[j]);
    }
  }
  Eigen::MatrixXd cov = Eigen::MatrixXd::Zero(n, n);
  Eigen::LDLT<Eigen::MatrixXd> ldlt(F_sub);
  if (ldlt.info() == Eigen::Success &&
      ldlt.vectorD().minCoeff() > 1e-12 * F_sub.trace() / n) {
    cov = ldlt.solve(Eigen::MatrixXd::Identity(n, n));
  } else {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(
        F_sub, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const double lam_max = svd.singularValues().maxCoeff();
    const double tol =
        1e-10 * std::max(n, 1) * std::max(lam_max, 1.0);
    for (int k = 0; k < svd.singularValues().size(); ++k) {
      if (svd.singularValues()(k) > tol) {
        cov += svd.matrixV().col(k) * svd.matrixV().col(k).transpose() /
               svd.singularValues()(k);
      }
    }
  }
  return cov;
}

Eigen::VectorXd PackExtrinsic14Error(const ExtrinsicState& lw,
                                     const ExtrinsicState& cw,
                                     const SyntheticScenarioBundle& sc) {
  const ExtrinsicSixDofErrors lw_e =
      ExtrinsicError(StateToSE3(lw), sc.gt.T_LW);
  const ExtrinsicSixDofErrors cw_e =
      ExtrinsicError(StateToSE3(cw), sc.gt.T_CW);
  Eigen::VectorXd x(14);
  x(0) = lw.t_d - sc.gt.t_d_L_s;
  x(1) = lw_e.rot_err_rad.x();
  x(2) = lw_e.rot_err_rad.y();
  x(3) = lw_e.rot_err_rad.z();
  x(4) = lw_e.trans_err_m.x();
  x(5) = lw_e.trans_err_m.y();
  x(6) = lw_e.trans_err_m.z();
  x(7) = cw.t_d - sc.gt.t_d_C_s;
  x(8) = cw_e.rot_err_rad.x();
  x(9) = cw_e.rot_err_rad.y();
  x(10) = cw_e.rot_err_rad.z();
  x(11) = cw_e.trans_err_m.x();
  x(12) = cw_e.trans_err_m.y();
  x(13) = cw_e.trans_err_m.z();
  return x;
}

bool SolveStage2AtTraj(
    const SyntheticScenarioBundle& sc_template, const BodyTrajectory& traj,
    const clic_calib::LeverArmConfig& levers, const SplineConfig& spline_cfg,
    const CoarseExtrinsicInit& t_d_prior, const RealisticNoiseSpec& noise,
    Stage2Result* s2) {
  SyntheticScenarioBundle sc = sc_template;
  const PinholeIntrinsics K{600.0, 600.0, 320.0, 240.0};
  const RadtanDistortion dist;
  sc.tag_obs = SynthesizeTagObsForTrajectory(
      sc.tag_obs, traj, sc.gt.T_CW, levers.L_B_to_G, levers.L_G_to_M.at(0),
      t_d_prior.t_d_C_s, K, dist, noise, kRepSeed);
  const GeometricInitReport geo = MakeGeometricExtrinsicInit(
      traj, sc.lidar_obs, sc.tag_obs, levers, kSphereRadiusM, t_d_prior);
  CoarseExtrinsicInit init = geo.init;
  init.t_d_L_s = t_d_prior.t_d_L_s;
  init.t_d_C_s = t_d_prior.t_d_C_s;
  *s2 = SolveStage2Extrinsics(sc, traj, levers, noise, init,
                              spline_cfg.t_d_max_abs_s);
  return s2->converged;
}

Eigen::MatrixXd FiniteDifferenceJthetaAtKnot(
    const SyntheticScenarioBundle& sc_template, const BodyTrajectory& traj_nom,
    const clic_calib::LeverArmConfig& levers, const SplineConfig& spline_cfg,
    const CoarseExtrinsicInit& t_d_prior, const RealisticNoiseSpec& noise,
    int knot, double eps_pos = 1e-4, double eps_rot = 1e-5) {
  Stage2Result s0;
  if (!SolveStage2AtTraj(sc_template, traj_nom, levers, spline_cfg, t_d_prior,
                         noise, &s0)) {
    return Eigen::MatrixXd::Zero(14, 6);
  }
  const Eigen::VectorXd e0 =
      PackExtrinsic14Error(s0.lidar, s0.camera, sc_template);
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(14, 6);
  BodyTrajectory traj = traj_nom;
  for (int ax = 0; ax < 3; ++ax) {
    traj = traj_nom;
    Eigen::Vector3d dp = Eigen::Vector3d::Zero();
    dp(ax) = eps_pos;
    traj.setKnotPos(traj.getKnotPos(knot) + dp, knot);
    Stage2Result s1;
    if (SolveStage2AtTraj(sc_template, traj, levers, spline_cfg, t_d_prior,
                          noise, &s1)) {
      const Eigen::VectorXd e1 =
          PackExtrinsic14Error(s1.lidar, s1.camera, sc_template);
      J.col(ax) = (e1 - e0) / eps_pos;
    }
  }
  for (int ax = 0; ax < 3; ++ax) {
    traj = traj_nom;
    const SO3d dR = SO3d::exp(Eigen::Vector3d::Unit(ax) * eps_rot);
    traj.setKnotSO3(dR * traj.getKnotSO3(knot), knot);
    Stage2Result s1;
    if (SolveStage2AtTraj(sc_template, traj, levers, spline_cfg, t_d_prior,
                          noise, &s1)) {
      const Eigen::VectorXd e1 =
          PackExtrinsic14Error(s1.lidar, s1.camera, sc_template);
      J.col(3 + ax) = (e1 - e0) / eps_rot;
    }
  }
  return J;
}

struct Stage1FimContext {
  CeresSo3ProblemScope scope;
  BodyTrajectory trajectory;
  Eigen::MatrixXd F;

  explicit Stage1FimContext(const BodyTrajectory& traj) : trajectory(traj) {}
};

Stage1FimContext BuildStage1FimContext(
    const SyntheticScenarioBundle& scenario, const BodyTrajectory& trajectory,
    const clic_calib::LeverArmConfig& levers, double alpha_p, double alpha_R) {
  Stage1FimContext ctx(trajectory);
  const SplineSegmentMeta<clic_calib::SplineOrder> meta =
      TrajectoryMeta(ctx.trajectory);
  ceres::Problem& problem = *ctx.scope.problem;
  std::set<double*> so3_registered;
  auto register_so3_knots =
      [&](const std::array<double*, clic_calib::SplineOrder>& rot_knots) {
        for (double* q : rot_knots) {
          if (so3_registered.insert(q).second) {
            ctx.scope.SetLocalParamSO3(q);
          }
        }
      };
  for (const auto& m : scenario.rtk) {
    if (m.fix_status_ != RTKMeasurement::FixStatus::FIXED) {
      continue;
    }
    const int64_t t_ns = static_cast<int64_t>(m.t_world_ * clic_calib::S_TO_NS);
    std::array<double*, clic_calib::SplineOrder> rot_knots{};
    std::array<double*, clic_calib::SplineOrder> pos_knots{};
    if (!GetActiveKnotPointers(ctx.trajectory, t_ns, &rot_knots, &pos_knots)) {
      continue;
    }
    ctx.scope.owned_costs.push_back(
        std::make_unique<clic_calib::analytic_derivative::RTKPositionFactor>(
            t_ns, m.p_A_W_observed_, m.covariance_, levers.L_B_to_A, meta));
    std::vector<double*> blocks;
    blocks.insert(blocks.end(), rot_knots.begin(), rot_knots.end());
    blocks.insert(blocks.end(), pos_knots.begin(), pos_knots.end());
    problem.AddResidualBlock(ctx.scope.owned_costs.back().get(), nullptr,
                             blocks);
    register_so3_knots(rot_knots);
  }
  AddStage1AttitudeFactors(&problem, &ctx.scope.owned_costs,
                           scenario.attitude_obs, ctx.trajectory, meta,
                           register_so3_knots);
  if (alpha_p > 0.0 || alpha_R > 0.0) {
    const double dt_s = ctx.trajectory.getDt();
    for (size_t seg = 0;
         seg + clic_calib::SplineOrder <= ctx.trajectory.numKnots(); ++seg) {
      const int64_t t_mid_ns =
          meta.t0_ns + static_cast<int64_t>((static_cast<double>(seg) + 0.5) *
                                            meta.dt_ns);
      if (t_mid_ns < meta.MinTimeNs() || t_mid_ns >= meta.MaxTimeNs()) {
        continue;
      }
      std::array<double*, clic_calib::SplineOrder> rot_knots{};
      std::array<double*, clic_calib::SplineOrder> pos_knots{};
      if (!GetActiveKnotPointers(ctx.trajectory, t_mid_ns, &rot_knots,
                                 &pos_knots)) {
        continue;
      }
      ctx.scope.owned_costs.push_back(std::make_unique<
          clic_calib::analytic_derivative::TrajectorySmoothnessFactor>(
          t_mid_ns, alpha_p, alpha_R, dt_s, meta));
      std::vector<double*> blocks;
      blocks.insert(blocks.end(), rot_knots.begin(), rot_knots.end());
      blocks.insert(blocks.end(), pos_knots.begin(), pos_knots.end());
      problem.AddResidualBlock(ctx.scope.owned_costs.back().get(), nullptr,
                               blocks);
      register_so3_knots(rot_knots);
    }
  }
  ceres::Problem::EvaluateOptions eval_opts;
  eval_opts.apply_loss_function = false;
  eval_opts.num_threads = 1;
  ceres::CRSMatrix J;
  problem.Evaluate(eval_opts, nullptr, nullptr, nullptr, &J);
  ctx.F = InformationFromCRS(J);
  return ctx;
}

struct DeltaMethodReport {
  Eigen::VectorXd analytic_std = Eigen::VectorXd::Zero(14);
  Eigen::VectorXd mc_std = Eigen::VectorXd::Zero(14);
  EigenReport stage1_fim;
  int num_knots = 0;
  int mid_knot = 0;
  int knots_used = 0;
};

DeltaMethodReport RunDeltaMethodCrossCheck(
    const clic_calib::LeverArmConfig& levers, const SplineConfig& spline_cfg,
    const CoarseExtrinsicInit& t_d_prior, const RealisticNoiseSpec& noise,
    const FimMcSampleStats& mc_total) {
  DeltaMethodReport out;
  SyntheticScenarioBundle rep_sc =
      BuildNearFieldFimNoisyScenarioWithAttitude(kRepSeed, kRepSeed, noise);
  const Stage1Result s1 = FitStage1WithAttitude(
      rep_sc, levers, spline_cfg, kStage1AlphaP, kStage1AlphaR);
  if (!s1.summary.IsSolutionUsable()) {
    return out;
  }

  const Stage1FimContext s1_ctx = BuildStage1FimContext(
      rep_sc, *s1.trajectory, levers, kStage1AlphaP, kStage1AlphaR);
  out.stage1_fim = AnalyzeSymmetricF(s1_ctx.F);
  out.num_knots = static_cast<int>(s1.trajectory->numKnots());
  out.mid_knot = out.num_knots / 2;

  SyntheticScenarioBundle sc_fixed =
      BuildNearFieldFimNoisyScenarioWithAttitude(kRepSeed, kRepSeed, noise);
  sc_fixed.tag_obs = SynthesizeTagObsForTrajectory(
      sc_fixed.tag_obs, *s1.trajectory, sc_fixed.gt.T_CW, levers.L_B_to_G,
      levers.L_G_to_M.at(0), t_d_prior.t_d_C_s,
      PinholeIntrinsics{600, 600, 320, 240}, RadtanDistortion{}, noise,
      kRepSeed);

  Stage2Result rep_s2;
  SolveStage2FixedTrajArm(kRepSeed, *s1.trajectory, levers, spline_cfg,
                          t_d_prior, noise, spline_cfg.t_d_max_abs_s, &sc_fixed,
                          &rep_s2);
  const Eigen::MatrixXd F_ext = Stage2ExtrinsicInformation(
      sc_fixed, *s1.trajectory, rep_s2.lidar, rep_s2.camera, levers, noise);
  Eigen::MatrixXd Sigma_fixed = Eigen::MatrixXd::Zero(14, 14);
  const EigenReport fext = AnalyzeSymmetricF(F_ext);
  if (fext.is_pd) {
    Sigma_fixed = F_ext.ldlt().solve(Eigen::MatrixXd::Identity(14, 14));
  }

  Eigen::MatrixXd Sigma_prop = Eigen::MatrixXd::Zero(14, 14);
  const int mid_knot = out.mid_knot;
  const std::vector<int> knot_idx =
      KnotTangentIndices(*s1_ctx.scope.problem, s1_ctx.trajectory, mid_knot);
  if (knot_idx.size() == 6) {
    const Eigen::MatrixXd Sigma_k =
        ExtractCovarianceBlock(s1_ctx.F, knot_idx);
    const Eigen::MatrixXd J = FiniteDifferenceJthetaAtKnot(
        sc_fixed, *s1.trajectory, levers, spline_cfg, t_d_prior, noise,
        mid_knot);
    Sigma_prop = J * Sigma_k * J.transpose();
    out.knots_used = 1;
  }

  const Eigen::MatrixXd Sigma_analytic = Sigma_fixed + Sigma_prop;
  for (int d = 0; d < 14; ++d) {
    if (Sigma_analytic(d, d) > 0.0) {
      out.analytic_std(d) = std::sqrt(Sigma_analytic(d, d));
    }
    const int mc_d = McDofFromFimDof(d);
    if (mc_d >= 0) {
      out.mc_std(d) = McArmDofConst(mc_total, mc_d)->Std();
    }
  }
  return out;
}

int CompareAnalyticMcRatio(const DeltaMethodReport& r, double lo, double hi) {
  std::cout << std::setw(12) << "DoF" << std::setw(14) << "analytic_std"
            << std::setw(14) << "mc_std" << std::setw(12) << "ratio"
            << std::setw(8) << "in_band" << "\n";
  int in_band = 0;
  for (int d = 0; d < 14; ++d) {
    const double a = r.analytic_std(d);
    const double m = r.mc_std(d);
    const double ratio =
        m > 1e-18 ? a / m : std::numeric_limits<double>::quiet_NaN();
    const bool ok = std::isfinite(ratio) && ratio >= lo && ratio <= hi;
    if (ok) {
      ++in_band;
    }
    std::cout << std::scientific << std::setprecision(4);
    std::cout << std::setw(12) << FixedFextDofLabel(d) << std::setw(14) << a
              << std::setw(14) << m << std::setw(12) << ratio << std::setw(8)
              << (ok ? "yes" : "NO") << std::defaultfloat << "\n";
  }
  return in_band;
}

Eigen::VectorXd TheoStdFromFixedFim(const Eigen::MatrixXd& F_fixed) {
  Eigen::VectorXd theo_std = Eigen::VectorXd::Zero(14);
  const EigenReport fobs = AnalyzeSymmetricF(F_fixed);
  if (!fobs.is_pd) {
    return theo_std;
  }
  const Eigen::MatrixXd cov =
      F_fixed.ldlt().solve(Eigen::MatrixXd::Identity(14, 14));
  for (int d = 0; d < 14; ++d) {
    if (cov(d, d) > 0.0) {
      theo_std(d) = std::sqrt(cov(d, d));
    }
  }
  return theo_std;
}

struct FixedTrajMcFimResult {
  FimMcSampleStats mc;
  Eigen::VectorXd theo_std = Eigen::VectorXd::Zero(14);
  EigenReport fobs;
  std::string traj_label;
};

FixedTrajMcFimResult CollectFixedTrajMcVsFim(
    const BodyTrajectory& fixed_traj, const char* traj_label,
    const clic_calib::LeverArmConfig& levers, const SplineConfig& spline_cfg,
    const CoarseExtrinsicInit& t_d_prior, const RealisticNoiseSpec& noise,
    int num_seeds, const Stage2Result* fim_linearization = nullptr) {
  FixedTrajMcFimResult out;
  out.traj_label = traj_label;

  SyntheticScenarioBundle rep_sc;
  Stage2Result rep_s2;
  if (SolveStage2FixedTrajArm(kRepSeed, fixed_traj, levers, spline_cfg,
                              t_d_prior, noise, spline_cfg.t_d_max_abs_s,
                              &rep_sc, &rep_s2)) {
    const ExtrinsicState& lw =
        fim_linearization ? fim_linearization->lidar : rep_s2.lidar;
    const ExtrinsicState& cw =
        fim_linearization ? fim_linearization->camera : rep_s2.camera;
    const Eigen::MatrixXd F_fixed = Stage2ExtrinsicInformation(
        rep_sc, fixed_traj, lw, cw, levers, noise);
    out.fobs = AnalyzeSymmetricF(F_fixed);
    out.theo_std = TheoStdFromFixedFim(F_fixed);
  }

  for (int i = 0; i < num_seeds; ++i) {
    const uint32_t seed = kSeedBase + static_cast<uint32_t>(i);
    SyntheticScenarioBundle sc;
    Stage2Result s2;
    if (!SolveStage2FixedTrajArm(seed, fixed_traj, levers, spline_cfg,
                                 t_d_prior, noise, spline_cfg.t_d_max_abs_s, &sc,
                                 &s2)) {
      continue;
    }
    PushFimMcSample(&out.mc, sc, s2);
    if ((i + 1) % 20 == 0 || i + 1 == num_seeds) {
      std::cout << "  Σ_fixed MC progress: " << (i + 1) << "/" << num_seeds
                << " (" << out.mc.converged << " converged)\n"
                << std::flush;
    }
  }
  return out;
}

void PrintMcStdTable(const FimMcSampleStats& mc, const char* title) {
  std::cout << std::fixed << std::setprecision(3);
  std::cout << "\n  " << title << " per-DoF std:\n";
  for (int d = 0; d < 14; ++d) {
    const double st = McArmDofConst(mc, d)->Std();
    double scale = 1e3;
    const char* unit = "mm";
    if (d % 6 < 3) {
      scale = 180e3 / M_PI;
      unit = "mrad";
    } else if (d >= 12) {
      scale = 1e5;
      unit = "0.01ms";
    }
    std::cout << "    " << std::setw(10) << std::left << kExtrinsicDofNames[d]
              << std::right << std::setw(10) << st * scale << " " << unit
              << "\n";
  }
}

void PrintFixedMcStdTable(const FimMcSampleStats& mc) {
  PrintMcStdTable(mc, "Σ_fixed_MC");
}

struct TrajPropDominanceReport {
  double trans_var_sum = 0.0;
  double rot_var_sum = 0.0;
  double lw_trans_var = 0.0;
  double lw_rot_var = 0.0;
  double cw_trans_var = 0.0;
  double cw_rot_var = 0.0;
};

TrajPropDominanceReport ComputeTrajPropDominance(const FimMcSampleStats& mc) {
  TrajPropDominanceReport r;
  const int trans_idx[] = {3, 4, 5, 9, 10, 11};
  const int rot_idx[] = {0, 1, 2, 6, 7, 8};
  for (int idx : trans_idx) {
    const double v = SampleVar(*McArmDofConst(mc, idx));
    r.trans_var_sum += v;
    if (idx <= 5) {
      r.lw_trans_var += v;
    } else {
      r.cw_trans_var += v;
    }
  }
  for (int idx : rot_idx) {
    const double v = SampleVar(*McArmDofConst(mc, idx));
    r.rot_var_sum += v;
    if (idx <= 2) {
      r.lw_rot_var += v;
    } else {
      r.cw_rot_var += v;
    }
  }
  return r;
}

FimMcSampleStats CollectTrajPropMc(
    const clic_calib::LeverArmConfig& levers, const SplineConfig& spline_cfg,
    const CoarseExtrinsicInit& t_d_prior, const RealisticNoiseSpec& noise,
    int num_seeds, uint32_t fixed_obs_seed = kRepSeed) {
  FimMcSampleStats out;
  for (int i = 0; i < num_seeds; ++i) {
    const uint32_t seed = kSeedBase + static_cast<uint32_t>(i);
    SyntheticScenarioBundle sc;
    std::shared_ptr<BodyTrajectory> traj;
    Stage2Result s2;
    if (!SolveClosedFormStage2WithNoiseSeeds(
            seed, fixed_obs_seed, levers, spline_cfg, t_d_prior, noise,
            spline_cfg.t_d_max_abs_s, &sc, &traj, &s2)) {
      continue;
    }
    PushFimMcSample(&out, sc, s2);
    if ((i + 1) % 20 == 0 || i + 1 == num_seeds) {
      std::cout << "  Σ_traj-prop MC progress: " << (i + 1) << "/" << num_seeds
                << " (" << out.converged << " converged)\n"
                << std::flush;
    }
  }
  return out;
}

struct FimMcAggregateResult {
  FimMcSampleStats mc;
  Eigen::VectorXd theo_std = Eigen::VectorXd::Zero(14);
  Eigen::VectorXd theo_std_marginal = Eigen::VectorXd::Zero(14);
  EigenReport fobs_rep;
  MarginalSchurReport marginal_rep;
  int joint_full_dim = 0;
};

FimMcAggregateResult CollectClosedFormStage2FimMc(
    const clic_calib::LeverArmConfig& levers,
    const SplineConfig& spline_cfg,
    const CoarseExtrinsicInit& t_d_prior,
    const RealisticNoiseSpec& noise,
    int num_seeds = kNumSeeds,
    bool include_marginal = false,
    bool joint_refine_before_fim = false) {
  FimMcAggregateResult out;
  Eigen::VectorXd theo_sq_sum = Eigen::VectorXd::Zero(14);
  Eigen::VectorXd marg_sq_sum = Eigen::VectorXd::Zero(14);
  int fim_count = 0;

  for (int i = 0; i < num_seeds; ++i) {
    const uint32_t seed = kSeedBase + static_cast<uint32_t>(i);
    SyntheticScenarioBundle sc;
    std::shared_ptr<BodyTrajectory> traj;
    Stage2Result s2;
    if (!SolveClosedFormStage2AtSeed(seed, levers, spline_cfg, t_d_prior, noise,
                                     spline_cfg.t_d_max_abs_s, &sc, &traj, &s2)) {
      continue;
    }
    ++out.mc.converged;

    const ExtrinsicSixDofErrors lw = ExtrinsicError(StateToSE3(s2.lidar), sc.gt.T_LW);
    const ExtrinsicSixDofErrors cw = ExtrinsicError(StateToSE3(s2.camera), sc.gt.T_CW);
    out.mc.lw_roll.Push(lw.rot_err_rad.x());
    out.mc.lw_pitch.Push(lw.rot_err_rad.y());
    out.mc.lw_yaw.Push(lw.rot_err_rad.z());
    out.mc.lw_tx.Push(lw.trans_err_m.x());
    out.mc.lw_ty.Push(lw.trans_err_m.y());
    out.mc.lw_tz.Push(lw.trans_err_m.z());
    out.mc.cw_roll.Push(cw.rot_err_rad.x());
    out.mc.cw_pitch.Push(cw.rot_err_rad.y());
    out.mc.cw_yaw.Push(cw.rot_err_rad.z());
    out.mc.cw_tx.Push(cw.trans_err_m.x());
    out.mc.cw_ty.Push(cw.trans_err_m.y());
    out.mc.cw_tz.Push(cw.trans_err_m.z());
    out.mc.t_d_L.Push(s2.lidar.t_d - sc.gt.t_d_L_s);
    out.mc.t_d_C.Push(s2.camera.t_d - sc.gt.t_d_C_s);

    const Eigen::MatrixXd F_fixed = Stage2ExtrinsicInformation(
        sc, *traj, s2.lidar, s2.camera, levers, noise);
    const EigenReport fobs = AnalyzeSymmetricF(F_fixed);
    if (fobs.is_pd) {
      const Eigen::MatrixXd cov =
          F_fixed.ldlt().solve(Eigen::MatrixXd::Identity(14, 14));
      for (int d = 0; d < 14; ++d) {
        if (cov(d, d) > 0.0) {
          theo_sq_sum(d) += cov(d, d);
        }
      }
      ++fim_count;
      if (seed == kRepSeed) {
        out.fobs_rep = fobs;
      }
    }

    if (include_marginal) {
      JointFimResult joint = JointInformationWithMarginal(
          sc, *traj, s2.lidar, s2.camera, levers, noise, kStage1AlphaP,
          kStage1AlphaR, joint_refine_before_fim);
      const Eigen::VectorXd marg_std =
          MarginalTheoStd14Svd(joint.marginal.F_marg);
      for (int d = 0; d < 14; ++d) {
        if (marg_std(d) > 0.0) {
          marg_sq_sum(d) += marg_std(d) * marg_std(d);
        }
      }
      if (seed == kRepSeed ||
          (num_seeds <= static_cast<int>(kRepSeed - kSeedBase) &&
           i + 1 == num_seeds)) {
        out.marginal_rep = joint.marginal;
        out.joint_full_dim = static_cast<int>(joint.F_full.rows());
      }
      if ((i + 1) % 5 == 0 || i + 1 == num_seeds) {
        std::cout << "  FIM↔MC progress: " << (i + 1) << "/" << num_seeds
                  << " seeds (" << out.mc.converged << " converged)\n"
                  << std::flush;
      }
    }
  }
  if (fim_count > 0) {
    for (int d = 0; d < 14; ++d) {
      out.theo_std(d) =
          std::sqrt(theo_sq_sum(d) / static_cast<double>(fim_count));
    }
    if (include_marginal) {
      for (int d = 0; d < 14; ++d) {
        out.theo_std_marginal(d) =
            std::sqrt(marg_sq_sum(d) / static_cast<double>(fim_count));
      }
    }
  }
  return out;
}

int PrintFimMcRatioTable(const FimMcSampleStats& mc,
                         const Eigen::VectorXd& theo_std) {
  auto theo = [&](int i) {
    if (i < 0 || i >= theo_std.size()) {
      return 0.0;
    }
    return theo_std(i);
  };
  struct Row {
    const char* name;
    double emp;
    double th;
    bool extrinsic_only;
  };
  const std::vector<Row> rows = {
      {"t_d_L", mc.t_d_L.Std(), theo(0), false},
      {"LW_roll", mc.lw_roll.Std(), theo(1), true},
      {"LW_pitch", mc.lw_pitch.Std(), theo(2), true},
      {"LW_yaw", mc.lw_yaw.Std(), theo(3), true},
      {"LW_tx", mc.lw_tx.Std(), theo(4), true},
      {"LW_ty", mc.lw_ty.Std(), theo(5), true},
      {"LW_tz", mc.lw_tz.Std(), theo(6), true},
      {"t_d_C", mc.t_d_C.Std(), theo(7), false},
      {"CW_roll", mc.cw_roll.Std(), theo(8), true},
      {"CW_pitch", mc.cw_pitch.Std(), theo(9), true},
      {"CW_yaw", mc.cw_yaw.Std(), theo(10), true},
      {"CW_tx", mc.cw_tx.Std(), theo(11), true},
      {"CW_ty", mc.cw_ty.Std(), theo(12), true},
      {"CW_tz", mc.cw_tz.Std(), theo(13), true},
  };

  std::cout << std::setw(12) << "DoF" << std::setw(16) << "emp_std"
            << std::setw(16) << "theo_std" << std::setw(12) << "ratio"
            << std::setw(8) << "in_band" << "\n";
  int in_band = 0;
  int in_band_ext = 0;
  int n_ext = 0;
  for (const Row& r : rows) {
    const double ratio =
        r.th > 1e-18 ? r.emp / r.th : std::numeric_limits<double>::quiet_NaN();
    const bool ok =
        std::isfinite(ratio) && ratio >= kRatioLo && ratio <= kRatioHi;
    if (ok) {
      ++in_band;
    }
    if (r.extrinsic_only) {
      ++n_ext;
      if (ok) {
        ++in_band_ext;
      }
    }
    std::cout << std::scientific << std::setprecision(16);
    std::cout << std::setw(12) << r.name << std::setw(16) << r.emp
              << std::setw(16) << r.th << std::setw(12) << ratio << std::setw(8)
              << (ok ? "yes" : "NO") << std::defaultfloat << "\n";
  }
  std::cout << "  summary: " << in_band << "/14 total, " << in_band_ext << "/"
            << n_ext << " extrinsic in band\n";
  return in_band;
}

struct DualRatioSummary {
  int in_band_fixed = 0;
  int in_band_marginal = 0;
  int in_band_ext_fixed = 0;
  int in_band_ext_marginal = 0;
};

DualRatioSummary PrintFimMcDualRatioTable(
    const FimMcSampleStats& mc, const Eigen::VectorXd& theo_fixed,
    const Eigen::VectorXd& theo_marginal) {
  auto theo = [&](const Eigen::VectorXd& v, int i) {
    if (i < 0 || i >= v.size()) {
      return 0.0;
    }
    return v(i);
  };
  struct Row {
    const char* name;
    double emp;
    double th_fixed;
    double th_marg;
    bool extrinsic_only;
  };
  const std::vector<Row> rows = {
      {"t_d_L", mc.t_d_L.Std(), theo(theo_fixed, 0), theo(theo_marginal, 0),
       false},
      {"LW_roll", mc.lw_roll.Std(), theo(theo_fixed, 1), theo(theo_marginal, 1),
       true},
      {"LW_pitch", mc.lw_pitch.Std(), theo(theo_fixed, 2),
       theo(theo_marginal, 2), true},
      {"LW_yaw", mc.lw_yaw.Std(), theo(theo_fixed, 3), theo(theo_marginal, 3),
       true},
      {"LW_tx", mc.lw_tx.Std(), theo(theo_fixed, 4), theo(theo_marginal, 4),
       true},
      {"LW_ty", mc.lw_ty.Std(), theo(theo_fixed, 5), theo(theo_marginal, 5),
       true},
      {"LW_tz", mc.lw_tz.Std(), theo(theo_fixed, 6), theo(theo_marginal, 6),
       true},
      {"t_d_C", mc.t_d_C.Std(), theo(theo_fixed, 7), theo(theo_marginal, 7),
       false},
      {"CW_roll", mc.cw_roll.Std(), theo(theo_fixed, 8), theo(theo_marginal, 8),
       true},
      {"CW_pitch", mc.cw_pitch.Std(), theo(theo_fixed, 9),
       theo(theo_marginal, 9), true},
      {"CW_yaw", mc.cw_yaw.Std(), theo(theo_fixed, 10), theo(theo_marginal, 10),
       true},
      {"CW_tx", mc.cw_tx.Std(), theo(theo_fixed, 11), theo(theo_marginal, 11),
       true},
      {"CW_ty", mc.cw_ty.Std(), theo(theo_fixed, 12), theo(theo_marginal, 12),
       true},
      {"CW_tz", mc.cw_tz.Std(), theo(theo_fixed, 13), theo(theo_marginal, 13),
       true},
  };

  DualRatioSummary summary;
  std::cout << std::setw(12) << "DoF" << std::setw(16) << "emp_std"
            << std::setw(16) << "fixed_theo" << std::setw(12) << "fix_ratio"
            << std::setw(16) << "marg_theo" << std::setw(12) << "marg_ratio"
            << std::setw(8) << "fix_ok" << std::setw(8) << "marg_ok" << "\n";
  int n_ext = 0;
  for (const Row& r : rows) {
    const double ratio_fixed =
        r.th_fixed > 1e-18 ? r.emp / r.th_fixed
                           : std::numeric_limits<double>::quiet_NaN();
    const double ratio_marg =
        r.th_marg > 1e-18 ? r.emp / r.th_marg
                          : std::numeric_limits<double>::quiet_NaN();
    const bool ok_fixed =
        std::isfinite(ratio_fixed) && ratio_fixed >= kRatioLo &&
        ratio_fixed <= kRatioHi;
    const bool ok_marg =
        std::isfinite(ratio_marg) && ratio_marg >= kRatioLo &&
        ratio_marg <= kRatioHi;
    if (ok_fixed) {
      ++summary.in_band_fixed;
    }
    if (ok_marg) {
      ++summary.in_band_marginal;
    }
    if (r.extrinsic_only) {
      ++n_ext;
      if (ok_fixed) {
        ++summary.in_band_ext_fixed;
      }
      if (ok_marg) {
        ++summary.in_band_ext_marginal;
      }
    }
    std::cout << std::scientific << std::setprecision(16);
    std::cout << std::setw(12) << r.name << std::setw(16) << r.emp
              << std::setw(16) << r.th_fixed << std::setw(12) << ratio_fixed
              << std::setw(16) << r.th_marg << std::setw(12) << ratio_marg
              << std::setw(8) << (ok_fixed ? "yes" : "NO") << std::setw(8)
              << (ok_marg ? "yes" : "NO") << std::defaultfloat << "\n";
  }
  std::cout << "  summary fixed:   " << summary.in_band_fixed << "/14 ("
            << summary.in_band_ext_fixed << "/" << n_ext << " extrinsic)\n";
  std::cout << "  summary marginal:" << summary.in_band_marginal << "/14 ("
            << summary.in_band_ext_marginal << "/" << n_ext << " extrinsic)\n";
  return summary;
}

void PrintMcBiasTable(const FimMcSampleStats& mc) {
  struct Row {
    const char* name;
    const RunningStats& s;
    const char* unit;
  };
  const std::vector<Row> rows = {
      {"t_d_L", mc.t_d_L, "s"},
      {"LW_roll", mc.lw_roll, "rad"},
      {"LW_pitch", mc.lw_pitch, "rad"},
      {"LW_yaw", mc.lw_yaw, "rad"},
      {"LW_tx", mc.lw_tx, "m"},
      {"LW_ty", mc.lw_ty, "m"},
      {"LW_tz", mc.lw_tz, "m"},
      {"t_d_C", mc.t_d_C, "s"},
      {"CW_roll", mc.cw_roll, "rad"},
      {"CW_pitch", mc.cw_pitch, "rad"},
      {"CW_yaw", mc.cw_yaw, "rad"},
      {"CW_tx", mc.cw_tx, "m"},
      {"CW_ty", mc.cw_ty, "m"},
      {"CW_tz", mc.cw_tz, "m"},
  };
  std::cout << "\n  MC bias (mean estimate−GT; zero-mean assumption for std/ratio):\n";
  std::cout << std::setw(12) << "DoF" << std::setw(20) << "mean_bias"
            << std::setw(16) << "emp_std" << std::setw(8) << "unit" << "\n";
  for (const Row& r : rows) {
    std::cout << std::scientific << std::setprecision(16);
    std::cout << std::setw(12) << r.name << std::setw(20) << r.s.mean
              << std::setw(16) << r.s.Std() << std::setw(8) << r.unit
              << std::defaultfloat << "\n";
  }
}

int CountTranslationInBand(const FimMcSampleStats& mc,
                           const Eigen::VectorXd& theo) {
  auto ok = [&](const RunningStats& s, int idx) {
    const double th = idx < theo.size() ? theo(idx) : 0.0;
    return th > 1e-18 && s.Std() / th >= kRatioLo && s.Std() / th <= kRatioHi;
  };
  int n = 0;
  if (ok(mc.lw_tx, 4)) ++n;
  if (ok(mc.lw_tz, 6)) ++n;
  if (ok(mc.cw_tx, 11)) ++n;
  if (ok(mc.cw_ty, 12)) ++n;
  if (ok(mc.cw_tz, 13)) ++n;
  return n;
}

void PrintMarginalOutOfBandDofs(const FimMcSampleStats& mc,
                                const Eigen::VectorXd& theo_marginal) {
  struct Row {
    const char* name;
    const RunningStats& s;
    int idx;
  };
  const std::vector<Row> rows = {
      {"t_d_L", mc.t_d_L, 0},       {"LW_roll", mc.lw_roll, 1},
      {"LW_pitch", mc.lw_pitch, 2}, {"LW_yaw", mc.lw_yaw, 3},
      {"LW_tx", mc.lw_tx, 4},       {"LW_ty", mc.lw_ty, 5},
      {"LW_tz", mc.lw_tz, 6},       {"t_d_C", mc.t_d_C, 7},
      {"CW_roll", mc.cw_roll, 8},   {"CW_pitch", mc.cw_pitch, 9},
      {"CW_yaw", mc.cw_yaw, 10},    {"CW_tx", mc.cw_tx, 11},
      {"CW_ty", mc.cw_ty, 12},      {"CW_tz", mc.cw_tz, 13},
  };
  std::cout << "  Marginal out-of-band DoF (ratio = emp_std / marg_theo):\n";
  for (const Row& r : rows) {
    const double th =
        r.idx < theo_marginal.size() ? theo_marginal(r.idx) : 0.0;
    const double ratio =
        th > 1e-18 ? r.s.Std() / th : std::numeric_limits<double>::quiet_NaN();
    if (!std::isfinite(ratio) || ratio < kRatioLo || ratio > kRatioHi) {
      std::cout << std::scientific << std::setprecision(16);
      std::cout << "    " << r.name << "  ratio=" << ratio << "  emp="
                << r.s.Std() << "  marg_theo=" << th << std::defaultfloat
                << "\n";
    }
  }
}

EigenReport Stage2ExtrinsicFim(
    const SyntheticScenarioBundle& scenario,
    const BodyTrajectory& fixed_traj,
    const ExtrinsicState& lidar, const ExtrinsicState& camera,
    const clic_calib::LeverArmConfig& levers,
    const RealisticNoiseSpec& noise) {
  const Eigen::MatrixXd F = Stage2ExtrinsicInformation(
      scenario, fixed_traj, lidar, camera, levers, noise);
  return AnalyzeSymmetricF(F);
}

Eigen::Vector3d BackprojectPixelToGround(const Eigen::Vector2d& uv,
                                         const SE3d& T_CW,
                                         const PinholeIntrinsics& K,
                                         const RadtanDistortion& dist,
                                         double z_plane = 0.0) {
  Eigen::Vector2d uv_n((uv.x() - K.cx) / K.fx, (uv.y() - K.cy) / K.fy);
  uv_n = clic_calib::UndistortRadtan(uv_n, dist);
  const Eigen::Vector3d dir_C(uv_n.x(), uv_n.y(), 1.0);
  const SE3d T_WC = T_CW.inverse();
  const Eigen::Vector3d o_W = T_WC * Eigen::Vector3d::Zero();
  const Eigen::Vector3d d_W = T_WC.so3() * dir_C;
  const double t = (z_plane - o_W.z()) / d_W.z();
  return o_W + t * d_W;
}

double CameraGroundReprojM(const SE3d& T_CW_est, const SE3d& T_CW_gt,
                           double range_m) {
  const PinholeIntrinsics K{600.0, 600.0, 320.0, 240.0};
  const RadtanDistortion dist;
  const Eigen::Vector3d p_V_W(range_m, 0.0, 0.0);
  const Eigen::Vector3d p_C_gt = T_CW_gt * p_V_W;
  const Eigen::Vector2d uv_gt =
      clic_calib::ProjectRadtan(p_C_gt, K, dist, nullptr);
  const Eigen::Vector3d p_W_from_est =
      BackprojectPixelToGround(uv_gt, T_CW_est, K, dist, 0.0);
  return (p_W_from_est - p_V_W).norm();
}

double LidarHorizReprojM(const SE3d& T_LW_est, const SE3d& T_LW_gt,
                         double range_m) {
  const Eigen::Vector3d p_V_W(range_m, 0.0, 0.0);
  const Eigen::Vector3d p_L_gt = T_LW_gt * p_V_W;
  const Eigen::Vector3d p_V_W_est = T_LW_est.inverse() * p_L_gt;
  return std::sqrt(std::pow(p_V_W_est.x() - p_V_W.x(), 2) +
                   std::pow(p_V_W_est.y() - p_V_W.y(), 2));
}

TEST(TwoStageProbe, Step1Stage1TrajectoryRtkOnly) {
  const std::string config_dir = ConfigDirFromExperiments();
  const auto levers = clic_calib::LeverArmConfig::from_yaml(
      config_dir + "/lever_arms.yaml");
  const auto spline_cfg = LoadSplineConfig(config_dir + "/spline.yaml");
  const RealisticNoiseSpec noise =
      clic_calib::NoiseModel::FromConfigDir(config_dir);
  const double t_end = NearFieldFlightDurationS(NearFieldFimScenarioGeometry());

  std::cout << "\n=== STEP 1 — Stage 1 RTK-only trajectory (N=" << kNumSeeds
            << ") ===\n";

  // FIM before/after smoothness (rep seed)
  {
    const SyntheticScenarioBundle rep =
        BuildNearFieldFimNoisyScenario(kRepSeed, noise);
    const Stage1Result no_smooth =
        FitStage1Trajectory(rep, levers, spline_cfg, 0.0, 0.0);
    const Stage1Result with_smooth =
        FitStage1Trajectory(rep, levers, spline_cfg, kStage1AlphaP, kStage1AlphaR);
    std::cout << "  Trajectory-only FIM (rep seed " << kRepSeed << "):\n";
    std::cout << "    no smoothness:   rank=" << no_smooth.fim.rank << "/"
              << no_smooth.fim.dim << "  λ_min=" << std::scientific
              << no_smooth.fim.lambda_min << "  cond=" << no_smooth.fim.cond
              << std::defaultfloat << "\n";
    std::cout << "    alpha_p/R=" << kStage1AlphaP << ": rank="
              << with_smooth.fim.rank << "/" << with_smooth.fim.dim
              << "  λ_min=" << std::scientific << with_smooth.fim.lambda_min
              << "  cond=" << with_smooth.fim.cond << std::defaultfloat << "\n";
  }

  // Smoothness weight sweep (rep seed)
  std::cout << "  Smoothness sweep (traj pos RMS mm, rep seed):\n";
  for (double alpha : {0.0, 0.001, 0.01, 0.1}) {
    const SyntheticScenarioBundle rep =
        BuildNearFieldFimNoisyScenario(kRepSeed, noise);
    const Stage1Result r =
        FitStage1Trajectory(rep, levers, spline_cfg, alpha, alpha);
    const double rms =
        TrajPosRmsMm(*r.trajectory, rep.gt_traj, t_end);
    std::cout << "    alpha=" << alpha << "  pos_RMS=" << rms << " mm\n";
  }

  RunningStats pos_rms, rot_rms;
  for (int i = 0; i < kNumSeeds; ++i) {
    const SyntheticScenarioBundle sc =
        BuildNearFieldFimNoisyScenario(kSeedBase + static_cast<uint32_t>(i),
                                       noise);
    const Stage1Result r = FitStage1Trajectory(
        sc, levers, spline_cfg, kStage1AlphaP, kStage1AlphaR);
    if (!r.summary.IsSolutionUsable()) {
      continue;
    }
    pos_rms.Push(TrajPosRmsMm(*r.trajectory, sc.gt_traj, t_end));
    rot_rms.Push(TrajRotRmsDeg(*r.trajectory, sc.gt_traj, t_end));
  }

  PrintBiasStd("traj position RMS", pos_rms, "mm", 3);
  PrintBiasStd("traj orientation RMS", rot_rms, "deg", 4);
  {
    const SyntheticScenarioBundle rep =
        BuildNearFieldFimNoisyScenario(kRepSeed, noise);
    const Stage1Result r = FitStage1Trajectory(
        rep, levers, spline_cfg, kStage1AlphaP, kStage1AlphaR);
    std::cout << "  RTK antenna residual RMS (rep seed): "
              << RtkAntennaRmsMm(*r.trajectory, rep.rtk, levers.L_B_to_A)
              << " mm\n";
  }
  std::cout << "  smoothness: alpha_p=alpha_R=" << kStage1AlphaP << "\n";
  SUCCEED();
}

TEST(TwoStageProbe, Step2Stage2ExtrinsicsCoarseInit) {
  const std::string config_dir = ConfigDirFromExperiments();
  const auto levers = clic_calib::LeverArmConfig::from_yaml(
      config_dir + "/lever_arms.yaml");
  const auto spline_cfg = LoadSplineConfig(config_dir + "/spline.yaml");
  const CoarseExtrinsicInit coarse = LoadCoarseExtrinsicsFromYaml(config_dir);
  const RealisticNoiseSpec noise =
      clic_calib::NoiseModel::FromConfigDir(config_dir);

  RunningStats lw_pitch, lw_trans_mm, cw_pitch, cw_trans_mm;
  RunningStats t_d_L, t_d_C;
  RunningStats stage2_cond;
  int converged = 0;

  std::cout << "\n=== STEP 2 — Stage 2 extrinsics, coarse yaml init (N="
            << kNumSeeds << ") ===\n";
  std::cout << "  Coarse T_LW t norm="
            << coarse.T_LW.translation().norm() << " m  (GT near-field differs)\n";

  for (int i = 0; i < kNumSeeds; ++i) {
    const SyntheticScenarioBundle sc =
        BuildNearFieldFimNoisyScenario(kSeedBase + static_cast<uint32_t>(i),
                                       noise);
    const Stage1Result s1 = FitStage1Trajectory(
        sc, levers, spline_cfg, kStage1AlphaP, kStage1AlphaR);
    const Stage2Result s2 = SolveStage2Extrinsics(
        sc, *s1.trajectory, levers, noise, coarse, spline_cfg.t_d_max_abs_s);
    if (!s2.converged) {
      continue;
    }
    ++converged;
    const ExtrinsicSixDofErrors lw_e =
        ExtrinsicError(StateToSE3(s2.lidar), sc.gt.T_LW);
    const ExtrinsicSixDofErrors cw_e =
        ExtrinsicError(StateToSE3(s2.camera), sc.gt.T_CW);
    lw_pitch.Push(lw_e.rot_err_rad.y() * 180.0 / M_PI);
    lw_trans_mm.Push(lw_e.trans_err_m.norm() * 1e3);
    cw_pitch.Push(cw_e.rot_err_rad.y() * 180.0 / M_PI);
    cw_trans_mm.Push(cw_e.trans_err_m.norm() * 1e3);
    t_d_L.Push((s2.lidar.t_d - sc.gt.t_d_L_s) * 1e3);
    t_d_C.Push((s2.camera.t_d - sc.gt.t_d_C_s) * 1e3);

    const EigenReport fim = Stage2ExtrinsicFim(
        sc, *s1.trajectory, s2.lidar, s2.camera, levers, noise);
    if (fim.is_pd) {
      stage2_cond.Push(std::log10(fim.cond));
    }
  }

  std::cout << "  converged=" << converged << "/" << kNumSeeds << "\n";
  PrintBiasStd("LW pitch err", lw_pitch, "deg", 4);
  PrintBiasStd("|LW trans| err", lw_trans_mm, "mm", 3);
  PrintBiasStd("CW pitch err", cw_pitch, "deg", 4);
  PrintBiasStd("|CW trans| err", cw_trans_mm, "mm", 3);
  PrintBiasStd("t_d_L err", t_d_L, "ms", 3);
  PrintBiasStd("t_d_C err", t_d_C, "ms", 3);
  if (stage2_cond.n > 0) {
    std::cout << "  Stage-2 cond (log10) mean=" << stage2_cond.mean
              << "  (joint ref log10=" << std::log10(kJointCondRef) << ")\n";
  }

  std::cout << "\n=== GATE 2 — two-stage decisive verdict ===\n";
  const bool cm_level =
      lw_trans_mm.mean < 50.0 && cw_trans_mm.mean < 50.0 &&
      std::abs(lw_pitch.mean) < 1.0 && converged >= 45;
  std::cout << (cm_level ? "  → Stage 2 reaches ~cm-level from coarse init\n"
                         : "  → Stage 2 did NOT reach cm-level — investigate\n");
  SUCCEED();
}

TEST(TwoStageProbe, Step3FimMcStage2) {
  std::cout << "\n=== STEP 3 (legacy) — superseded ===\n";
  std::cout << "  Use TwoStageClosedFormInit.Gate5FimMcClosedFormStage2 "
            << "(closed-form init + attitude Stage-1).\n";
  GTEST_SKIP();
}

TEST(TwoStageProbe, Step4DegeneracyTwoStage) {
  const std::string config_dir = ConfigDirFromExperiments();
  const auto levers = clic_calib::LeverArmConfig::from_yaml(
      config_dir + "/lever_arms.yaml");
  const auto spline_cfg = LoadSplineConfig(config_dir + "/spline.yaml");
  const CoarseExtrinsicInit coarse = LoadCoarseExtrinsicsFromYaml(config_dir);
  const RealisticNoiseSpec noise =
      clic_calib::NoiseModel::FromConfigDir(config_dir);

  auto run_pattern = [&](const SyntheticScenarioBundle& sc,
                         RunningStats* cam200, RunningStats* lid200,
                         EigenReport* fim) {
    const Stage1Result s1 = FitStage1Trajectory(
        sc, levers, spline_cfg, kStage1AlphaP, kStage1AlphaR);
    const Stage2Result s2 = SolveStage2Extrinsics(
        sc, *s1.trajectory, levers, noise, coarse, spline_cfg.t_d_max_abs_s);
    if (!s2.converged) {
      return false;
    }
    cam200->Push(CameraGroundReprojM(StateToSE3(s2.camera), sc.gt.T_CW, 200.0));
    lid200->Push(LidarHorizReprojM(StateToSE3(s2.lidar), sc.gt.T_LW, 200.0));
    *fim = Stage2ExtrinsicFim(sc, *s1.trajectory, s2.lidar, s2.camera, levers,
                              noise);
    return true;
  };

  RunningStats cam_multi, cam_coplanar, lid_multi, lid_coplanar;
  EigenReport fim_multi, fim_coplanar;
  int ok_multi = 0, ok_coplanar = 0;

  std::cout << "\n=== STEP 4 — degeneracy on two-stage solver (N=" << kNumSeeds
            << ") ===\n";

  for (int i = 0; i < kNumSeeds; ++i) {
    const uint32_t seed = kSeedBase + static_cast<uint32_t>(i);
    if (run_pattern(BuildNearFieldFimNoisyScenario(seed, noise), &cam_multi,
                    &lid_multi, &fim_multi)) {
      ++ok_multi;
    }
    if (run_pattern(BuildNearFieldCoplanarFimNoisyScenario(seed, noise),
                    &cam_coplanar, &lid_coplanar, &fim_coplanar)) {
      ++ok_coplanar;
    }
  }

  PrintBiasStd("camera_world multi @200m", cam_multi, "m", 4);
  PrintBiasStd("camera_world coplanar @200m", cam_coplanar, "m", 4);
  PrintBiasStd("LiDAR horiz multi @200m", lid_multi, "m", 4);
  PrintBiasStd("LiDAR horiz coplanar @200m", lid_coplanar, "m", 4);
  const double cam_ratio = cam_coplanar.mean / std::max(cam_multi.mean, 1e-12);
  const double lid_ratio = lid_coplanar.mean / std::max(lid_multi.mean, 1e-12);
  std::cout << "  Ratio coplanar/multi: camera=" << cam_ratio
            << "  lidar=" << lid_ratio << "\n";
  std::cout << "  F_ext λ_min multi (last seed aggregate): see per-pattern FIM\n";
  std::cout << "  converged multi=" << ok_multi << " coplanar=" << ok_coplanar
            << "\n";
  SUCCEED();
}

double SphereLateralErrorMm(const BodyTrajectory& est, const BodyTrajectory& gt,
                            double t, const Eigen::Vector3d& L_B_to_G) {
  const Eigen::Vector3d p_est = est.sphere_center_w(t, L_B_to_G);
  const Eigen::Vector3d p_gt = gt.sphere_center_w(t, L_B_to_G);
  return (p_est.head<2>() - p_gt.head<2>()).norm() * 1e3;
}

SyntheticScenarioBundle BuildScenarioWithAttitude(uint32_t seed,
                                                  const RealisticNoiseSpec& noise,
                                                  const AttitudeNoiseSpec& att) {
  return BuildNearFieldFimNoisyScenarioWithAttitude(seed, noise, att);
}

TEST(TwoStageAttitudeProbe, Stage2OracleGtTrajectory) {
  const std::string config_dir = ConfigDirFromExperiments();
  const auto levers = clic_calib::LeverArmConfig::from_yaml(
      config_dir + "/lever_arms.yaml");
  const auto spline_cfg = LoadSplineConfig(config_dir + "/spline.yaml");
  const RealisticNoiseSpec noise =
      clic_calib::NoiseModel::FromConfigDir(config_dir);
  const SyntheticScenarioBundle sc =
      BuildNearFieldFimNoisyScenarioWithAttitude(kRepSeed, noise);
  const double t_lo = sc.rtk.front().t_world_;
  const double t_hi = sc.rtk.back().t_world_;
  const double knot_dt = Stage1KnotDt(sc.rtk, sc.attitude_obs,
                                      spline_cfg.knot_interval_s);
  const auto traj =
      TrimTrajectoryToObservedSupport(sc.gt_traj, t_lo, t_hi, knot_dt);
  CoarseExtrinsicInit coarse;
  coarse.T_LW = sc.gt.T_LW;
  coarse.T_CW = sc.gt.T_CW;
  coarse.t_d_L_s = sc.gt.t_d_L_s;
  coarse.t_d_C_s = sc.gt.t_d_C_s;
  const Stage2Result s2 = SolveStage2Extrinsics(
      sc, *traj, levers, noise, coarse, spline_cfg.t_d_max_abs_s);
  const ExtrinsicSixDofErrors lw_e =
      ExtrinsicError(StateToSE3(s2.lidar), sc.gt.T_LW);
  std::cout << "Oracle GT traj: |LW| err mm="
            << lw_e.trans_err_m.norm() * 1e3 << " converged=" << s2.converged
            << "\n";
  EXPECT_TRUE(s2.converged);
  EXPECT_LT(lw_e.trans_err_m.norm() * 1e3, 50.0);
}

TEST(TwoStageAttitudeProbe, Stage2OracleGtTrajCoarseInit) {
  const std::string config_dir = ConfigDirFromExperiments();
  const auto levers = clic_calib::LeverArmConfig::from_yaml(
      config_dir + "/lever_arms.yaml");
  const auto spline_cfg = LoadSplineConfig(config_dir + "/spline.yaml");
  const RealisticNoiseSpec noise =
      clic_calib::NoiseModel::FromConfigDir(config_dir);
  const SyntheticScenarioBundle sc =
      BuildNearFieldFimNoisyScenarioWithAttitude(kRepSeed, noise);
  const double t_lo = sc.rtk.front().t_world_;
  const double t_hi = sc.rtk.back().t_world_;
  const double knot_dt = Stage1KnotDt(sc.rtk, sc.attitude_obs,
                                      spline_cfg.knot_interval_s);
  const auto traj =
      TrimTrajectoryToObservedSupport(sc.gt_traj, t_lo, t_hi, knot_dt);
  const CoarseExtrinsicInit coarse = MakeCoarseExtrinsicInitPerturbed(
      sc.gt.T_LW, sc.gt.T_CW, sc.gt.t_d_L_s, sc.gt.t_d_C_s, kRepSeed);
  const Stage2Result s2 = SolveStage2Extrinsics(
      sc, *traj, levers, noise, coarse, spline_cfg.t_d_max_abs_s);
  const ExtrinsicSixDofErrors lw_e =
      ExtrinsicError(StateToSE3(s2.lidar), sc.gt.T_LW);
  std::cout << "Oracle GT traj + coarse init: |LW| err mm="
            << lw_e.trans_err_m.norm() * 1e3 << "\n";
  EXPECT_LT(lw_e.trans_err_m.norm() * 1e3, 50.0);
}

TEST(TwoStageAttitudeProbe, AttitudeFactorJacobian) {
  const RealisticNoiseSpec noise =
      clic_calib::NoiseModel::FromConfigDir(ConfigDirFromExperiments());
  const SyntheticScenarioBundle sc =
      BuildNearFieldFimNoisyScenarioWithAttitude(kRepSeed, noise);
  ASSERT_FALSE(sc.attitude_obs.empty());
  const auto& obs = sc.attitude_obs[sc.attitude_obs.size() / 2];
  const int64_t t_ns =
      static_cast<int64_t>(obs.t_world_ * clic_calib::S_TO_NS);

  const double knot_dt = 0.05;
  BodyTrajectory traj(knot_dt, 0.0);
  traj.setKnots(SE3d(SO3d::rotZ(0.3), Eigen::Vector3d::Zero()), 80);
  const SplineSegmentMeta<clic_calib::SplineOrder> meta =
      TrajectoryMeta(traj);

  std::array<double*, clic_calib::SplineOrder> rot_knots{};
  std::array<double*, clic_calib::SplineOrder> pos_knots{};
  ASSERT_TRUE(GetActiveKnotPointers(traj, t_ns, &rot_knots, &pos_knots));

  clic_calib::analytic_derivative::AttitudeFactorPoseForm factor(
      t_ns, obs.R_WB_observed_, obs.covariance_, meta);
  std::vector<double*> params(rot_knots.begin(), rot_knots.end());
  std::vector<int> block_sizes(4, 4);

  const int num_residuals = factor.num_residuals();
  std::vector<double> residuals(num_residuals);
  std::vector<double*> jac_ptrs(4);
  std::vector<std::vector<double>> jac_storage(4);
  for (int b = 0; b < 4; ++b) {
    jac_storage[b].resize(num_residuals * 4);
    jac_ptrs[b] = jac_storage[b].data();
  }
  factor.Evaluate(params.data(), residuals.data(), jac_ptrs.data());
  Eigen::MatrixXd num_jac;
  clic_calib::test::NumericalJacobian(&factor, params, block_sizes,
                                      residuals.data(), &num_jac);
  double max_rel = 0.0;
  int worst_r = 0, worst_c = 0;
  double worst_a = 0.0, worst_n = 0.0;
  int col = 0;
  for (int b = 0; b < 4; ++b) {
    for (int d = 0; d < 3; ++d) {
      for (int r = 0; r < num_residuals; ++r) {
        const double a = jac_storage[b][r * 4 + d];
        const double n = num_jac(r, col + d);
        const double denom = std::max(std::abs(a), std::abs(n));
        const double rel =
            denom > 1e-8 ? std::abs(a - n) / denom : std::abs(a - n);
        if (rel > max_rel) {
          max_rel = rel;
          worst_r = r;
          worst_c = col + d;
          worst_a = a;
          worst_n = n;
        }
      }
    }
    col += 3;
  }
  std::cout << "AttitudeFactor Jacobian max rel err=" << max_rel
            << " at (r=" << worst_r << ",c=" << worst_c << ") analytic="
            << worst_a << " numeric=" << worst_n << "\n";
  EXPECT_TRUE(clic_calib::test::CompareJacobians(&factor, params, block_sizes));
}

TEST(TwoStageAttitudeProbe, AttitudeOnlySanity) {
  const RealisticNoiseSpec noise =
      clic_calib::NoiseModel::FromConfigDir(ConfigDirFromExperiments());
  const auto levers =
      clic_calib::LeverArmConfig::from_yaml(ConfigDirFromExperiments() +
                                            "/lever_arms.yaml");
  const SyntheticScenarioBundle sc =
      BuildNearFieldFimNoisyScenarioWithAttitude(kRepSeed, noise);
  const double t_end = NearFieldFlightDurationS(NearFieldFimScenarioGeometry());
  SplineConfig spline_cfg;
  const Stage1Result r =
      FitStage1AttitudeOnly(sc, levers, spline_cfg, 1, 0.0);
  const double rot_rms = TrajRotRmsDeg(*r.trajectory, sc.gt_traj, t_end);
  const TrajRpyRms rpy =
      TrajRpyRmsDeg(*r.trajectory, sc.gt_traj, t_end);
  std::cout << "Attitude-only rot RMS=" << rot_rms << " roll=" << rpy.roll
            << " pitch=" << rpy.pitch << " yaw=" << rpy.yaw << " deg\n";
  EXPECT_LT(rot_rms, 5.0);
  EXPECT_LT(rpy.yaw, 3.0);
}

TEST(TwoStageAttitudeProbe, Step1AttitudeStreamGate) {
  const RealisticNoiseSpec noise =
      clic_calib::NoiseModel::FromConfigDir(ConfigDirFromExperiments());
  AttitudeNoiseSpec att;
  const SyntheticScenarioBundle rep =
      BuildNearFieldFimNoisyScenarioWithAttitude(kRepSeed, noise, att);

  std::cout << "\n=== STEP 1 — attitude stream (50 Hz, RTK clock, anisotropic) ===\n";
  std::cout << "  samples=" << rep.attitude_obs.size()
            << "  σ_roll=pitch=" << att.sigma_roll_deg
            << "°  σ_yaw=" << att.sigma_yaw_deg << "°\n";
  std::cout << "  Time alignment: t_world_ shares RTK/world clock (no offset)\n";

  RunningStats err_roll, err_pitch, err_yaw;
  for (size_t i = 0; i < std::min<size_t>(5, rep.attitude_obs.size()); ++i) {
    const Eigen::Vector3d e =
        AttitudeObsErrorDeg(rep.attitude_obs[i], rep.gt_traj);
    std::cout << "  sample[" << i << "] t=" << rep.attitude_obs[i].t_world_
              << "s  err_rpy=[ " << e.x() << ", " << e.y() << ", " << e.z()
              << " ] deg\n";
  }
  for (const auto& obs : rep.attitude_obs) {
    const Eigen::Vector3d e = AttitudeObsErrorDeg(obs, rep.gt_traj);
    err_roll.Push(e.x());
    err_pitch.Push(e.y());
    err_yaw.Push(e.z());
  }
  PrintBiasStd("obs roll err", err_roll, "deg", 4);
  PrintBiasStd("obs pitch err", err_pitch, "deg", 4);
  PrintBiasStd("obs yaw err", err_yaw, "deg", 4);
  EXPECT_GT(rep.attitude_obs.size(), 100u);
  EXPECT_NEAR(err_roll.Std(), att.sigma_roll_deg, 0.15);
  EXPECT_NEAR(err_pitch.Std(), att.sigma_pitch_deg, 0.15);
  EXPECT_NEAR(err_yaw.Std(), att.sigma_yaw_deg, 0.35);
}

TEST(TwoStageAttitudeProbe, Step2Stage1WithAttitude) {
  const std::string config_dir = ConfigDirFromExperiments();
  const auto levers = clic_calib::LeverArmConfig::from_yaml(
      config_dir + "/lever_arms.yaml");
  const auto spline_cfg = LoadSplineConfig(config_dir + "/spline.yaml");
  const RealisticNoiseSpec noise =
      clic_calib::NoiseModel::FromConfigDir(config_dir);
  const double t_end = NearFieldFlightDurationS(NearFieldFimScenarioGeometry());

  std::cout << "\n=== STEP 2 — Stage 1 RTK + anisotropic attitude (PSDK stream init, N="
            << kNumSeeds << ") ===\n";
  std::cout << "  Attitude factors: 2 Hz (stride=25 on 50 Hz stream)\n";

  {
    const SyntheticScenarioBundle rep =
        BuildNearFieldFimNoisyScenarioWithAttitude(kRepSeed, noise);
    const Stage1Result r = FitStage1WithAttitude(
        rep, levers, spline_cfg, kStage1AlphaP, kStage1AlphaR);
    const EigenReport fim = ComputeStage1FimWithAttitude(
        rep, *r.trajectory, levers, kStage1AlphaP, kStage1AlphaR);
    std::cout << "  Trajectory FIM (rep): rank=" << fim.rank << "/"
              << fim.dim << "  λ_min=" << std::scientific << fim.lambda_min
              << "  cond=" << fim.cond << std::defaultfloat << "\n";
  }

  RunningStats pos_rms, rot_rms, roll_rms, pitch_rms, yaw_rms;
  for (int i = 0; i < kNumSeeds; ++i) {
    const SyntheticScenarioBundle sc = BuildNearFieldFimNoisyScenarioWithAttitude(
        kSeedBase + static_cast<uint32_t>(i), noise);
    const Stage1Result r = FitStage1WithAttitude(
        sc, levers, spline_cfg, kStage1AlphaP, kStage1AlphaR);
    if (!r.summary.IsSolutionUsable()) {
      continue;
    }
    pos_rms.Push(TrajPosRmsMm(*r.trajectory, sc.gt_traj, t_end));
    rot_rms.Push(TrajRotRmsDeg(*r.trajectory, sc.gt_traj, t_end));
    const TrajRpyRms rpy =
        TrajRpyRmsDeg(*r.trajectory, sc.gt_traj, t_end);
    roll_rms.Push(rpy.roll);
    pitch_rms.Push(rpy.pitch);
    yaw_rms.Push(rpy.yaw);
  }

  PrintBiasStd("traj position RMS", pos_rms, "mm", 3);
  PrintBiasStd("traj orientation RMS", rot_rms, "deg", 4);
  PrintBiasStd("traj roll RMS", roll_rms, "deg", 4);
  PrintBiasStd("traj pitch RMS", pitch_rms, "deg", 4);
  PrintBiasStd("traj yaw RMS", yaw_rms, "deg", 4);
  std::cout << "  init: PSDK attitude stream at knots (NOT GT), AttitudeFactorPoseForm\n";

  const bool gate2_ok = rot_rms.mean < 5.0 && roll_rms.mean < 1.0 &&
                        pitch_rms.mean < 1.0 && pos_rms.mean < 100.0;
  std::cout << "\n=== GATE 2 — Stage 1 with attitude ===\n";
  std::cout << (gate2_ok ? "  → orientation sub-degree; proceed\n"
                         : "  → orientation still poor — check factor\n");
  SUCCEED();
}

TEST(TwoStageAttitudeProbe, Step3Stage2WithAttitude) {
  const std::string config_dir = ConfigDirFromExperiments();
  const auto levers = clic_calib::LeverArmConfig::from_yaml(
      config_dir + "/lever_arms.yaml");
  const auto spline_cfg = LoadSplineConfig(config_dir + "/spline.yaml");
  const RealisticNoiseSpec noise =
      clic_calib::NoiseModel::FromConfigDir(config_dir);

  RunningStats lw_pitch, lw_trans_mm, cw_pitch, cw_trans_mm;
  RunningStats t_d_L, t_d_C;
  RunningStats stage2_cond;
  int converged = 0;

  std::cout << "\n=== STEP 3 — Stage 2 on attitude-fixed trajectory, coarse init (N="
            << kNumSeeds << ") ===\n";
  std::cout << "  Coarse init: GT perturbed ~1 m / ~10° (near-field geometry)\n";

  for (int i = 0; i < kNumSeeds; ++i) {
    const uint32_t seed = kSeedBase + static_cast<uint32_t>(i);
    const SyntheticScenarioBundle sc =
        BuildNearFieldFimNoisyScenarioWithAttitude(seed, noise);
    const CoarseExtrinsicInit coarse = MakeCoarseExtrinsicInitPerturbed(
        sc.gt.T_LW, sc.gt.T_CW, sc.gt.t_d_L_s, sc.gt.t_d_C_s, seed);
    const Stage1Result s1 = FitStage1WithAttitude(
        sc, levers, spline_cfg, kStage1AlphaP, kStage1AlphaR);
    const Stage2Result s2 = SolveStage2Extrinsics(
        sc, *s1.trajectory, levers, noise, coarse, spline_cfg.t_d_max_abs_s);
    if (!s2.converged) {
      continue;
    }
    ++converged;
    const ExtrinsicSixDofErrors lw_e =
        ExtrinsicError(StateToSE3(s2.lidar), sc.gt.T_LW);
    const ExtrinsicSixDofErrors cw_e =
        ExtrinsicError(StateToSE3(s2.camera), sc.gt.T_CW);
    lw_pitch.Push(lw_e.rot_err_rad.y() * 180.0 / M_PI);
    lw_trans_mm.Push(lw_e.trans_err_m.norm() * 1e3);
    cw_pitch.Push(cw_e.rot_err_rad.y() * 180.0 / M_PI);
    cw_trans_mm.Push(cw_e.trans_err_m.norm() * 1e3);
    t_d_L.Push((s2.lidar.t_d - sc.gt.t_d_L_s) * 1e3);
    t_d_C.Push((s2.camera.t_d - sc.gt.t_d_C_s) * 1e3);
    const EigenReport fim = Stage2ExtrinsicFim(
        sc, *s1.trajectory, s2.lidar, s2.camera, levers, noise);
    if (fim.is_pd) {
      stage2_cond.Push(std::log10(fim.cond));
    }
  }

  std::cout << "  converged=" << converged << "/" << kNumSeeds << "\n";
  PrintBiasStd("LW pitch err", lw_pitch, "deg", 4);
  PrintBiasStd("|LW trans| err", lw_trans_mm, "mm", 3);
  PrintBiasStd("CW pitch err", cw_pitch, "deg", 4);
  PrintBiasStd("|CW trans| err", cw_trans_mm, "mm", 3);
  PrintBiasStd("t_d_L err", t_d_L, "ms", 3);
  PrintBiasStd("t_d_C err", t_d_C, "ms", 3);
  if (stage2_cond.n > 0) {
    std::cout << "  Stage-2 cond (log10) mean=" << stage2_cond.mean
              << "  (joint ref log10=" << std::log10(kJointCondRef)
              << "  prior RTK-only probe ~11.09)\n";
  }

  const bool cm_level = lw_trans_mm.mean < 50.0 && cw_trans_mm.mean < 50.0 &&
                        std::abs(lw_pitch.mean) < 1.0 && converged >= 45;
  std::cout << "\n=== GATE 3 — two-stage WITH attitude ===\n";
  std::cout << (cm_level ? "  → cm-level extrinsics from coarse init\n"
                         : "  → Stage 2 did NOT reach cm-level\n");
  SUCCEED();
}

TEST(TwoStageAttitudeProbe, Step4YawNoiseSweep) {
  std::cout << "\n=== STEP 4 (legacy) — superseded ===\n";
  std::cout << "  Use TwoStageClosedFormInit.Gate4YawSensitivitySweep "
            << "(closed-form Stage-2 init).\n";
  GTEST_SKIP();
}

TEST(TwoStageClosedFormInit, Gate4YawSensitivitySweep) {
  const std::string config_dir = ConfigDirFromExperiments();
  const auto levers = clic_calib::LeverArmConfig::from_yaml(
      config_dir + "/lever_arms.yaml");
  const auto spline_cfg = LoadSplineConfig(config_dir + "/spline.yaml");
  const CoarseExtrinsicInit t_d_prior =
      LoadCoarseExtrinsicsFromYaml(config_dir);
  const RealisticNoiseSpec noise =
      clic_calib::NoiseModel::FromConfigDir(config_dir);
  const double t_end = NearFieldFlightDurationS(NearFieldFimScenarioGeometry());
  const double L_BG = levers.L_B_to_G.norm();

  std::cout << "\n=== STEP 4 / GATE 4 — σ_yaw sensitivity (closed-form init, N="
            << kNumSeeds << ") ===\n";
  std::cout << "  σ_roll=σ_pitch=0.2°; pipeline: Stage-1 + closed-form Stage-2 + refine\n";
  std::cout << "  σ_yaw sweep: {0.3, 1.0, 1.5, 2.0}°\n\n";

  std::vector<YawSweepRow> rows;
  rows.reserve(4);
  for (double sigma_yaw : {0.3, 1.0, 1.5, 2.0}) {
    rows.push_back(RunYawSensitivityAtSigma(sigma_yaw, levers, spline_cfg,
                                            t_d_prior, noise, t_end));
  }
  PrintYawSweepTable(rows);

  const YawSweepRow* row_15 = nullptr;
  for (const auto& r : rows) {
    if (std::abs(r.sigma_yaw_deg - 1.5) < 1e-6) {
      row_15 = &r;
      break;
    }
  }
  if (row_15) {
    const double pred_lat_mm = L_BG * 1.5 * M_PI / 180.0 * 1e3;
    std::cout << "\n  p_G^W lateral @ σ_yaw=1.5°: |L_B→G|·σ_yaw = " << pred_lat_mm
              << " mm  empirical p_G lat = " << row_15->pG_lat_mm.mean
              << " mm  traj yaw RMS = " << row_15->traj_yaw_deg.mean << "°\n";
  }

  std::string verdict;
  const bool flat = EvaluateGate4YawVerdict(rows, &verdict);
  const YawSweepRow& lo = rows.front();
  const YawSweepRow& hi = rows.back();
  std::cout << "\n=== GATE 4 verdict ===\n";
  std::cout << "  |T_LW| Δ(2.0°−0.3°) = " << (hi.lw_trans_mm.mean - lo.lw_trans_mm.mean)
            << " mm  p_G lat Δ = " << (hi.pG_lat_mm.mean - lo.pG_lat_mm.mean)
            << " mm\n";
  std::cout << "  → " << verdict << "\n";

  for (const auto& r : rows) {
    EXPECT_GE(r.converged, 45);
    EXPECT_GE(r.cm_success, 45);
  }
  EXPECT_TRUE(flat);
}

TEST(TwoStageClosedFormInit, Gate5FimMcClosedFormStage2) {
  const std::string config_dir = ConfigDirFromExperiments();
  const auto levers = clic_calib::LeverArmConfig::from_yaml(
      config_dir + "/lever_arms.yaml");
  const auto spline_cfg = LoadSplineConfig(config_dir + "/spline.yaml");
  const CoarseExtrinsicInit t_d_prior =
      LoadCoarseExtrinsicsFromYaml(config_dir);
  const RealisticNoiseSpec noise =
      clic_calib::NoiseModel::FromConfigDir(config_dir);

  std::cout << "\n=== STEP 5 / GATE 5 — FIM↔MC (closed-form Stage-2, prior-free, N="
            << kNumSeeds << ") ===\n";
  std::cout << "  Pipeline: Stage-1 (RTK+attitude) + closed-form init + refine\n";
  std::cout << "  Theoretical: mean diag(F_ext_obs⁻¹) across converged seeds\n";
  std::cout << "  Band: empirical_std/theoretical_std ∈ [" << kRatioLo << ", "
            << kRatioHi << "]\n\n";

  const FimMcAggregateResult agg =
      CollectClosedFormStage2FimMc(levers, spline_cfg, t_d_prior, noise);
  std::cout << "  MC converged: " << agg.mc.converged << "/" << kNumSeeds << "\n";
  std::cout << std::scientific << std::setprecision(16);
  std::cout << "  F_ext @ rep converged: λ_min=" << agg.fobs_rep.lambda_min
            << "  λ_max=" << agg.fobs_rep.lambda_max
            << "  cond=" << agg.fobs_rep.cond << "  rank=" << agg.fobs_rep.rank
            << "/" << agg.fobs_rep.dim
            << "  PD=" << (agg.fobs_rep.is_pd ? "yes" : "NO") << std::defaultfloat
            << "\n\n";

  const int in_band = PrintFimMcRatioTable(agg.mc, agg.theo_std);

  std::cout << "\n=== GATE 5 verdict ===\n";
  const int in_band_ext = [&]() {
    int n = 0;
    auto ok = [&](double emp, double th) {
      if (th <= 1e-18) {
        return false;
      }
      const double r = emp / th;
      return r >= kRatioLo && r <= kRatioHi;
    };
    if (ok(agg.mc.lw_roll.Std(), agg.theo_std(1))) ++n;
    if (ok(agg.mc.lw_pitch.Std(), agg.theo_std(2))) ++n;
    if (ok(agg.mc.lw_yaw.Std(), agg.theo_std(3))) ++n;
    if (ok(agg.mc.lw_tx.Std(), agg.theo_std(4))) ++n;
    if (ok(agg.mc.lw_ty.Std(), agg.theo_std(5))) ++n;
    if (ok(agg.mc.lw_tz.Std(), agg.theo_std(6))) ++n;
    if (ok(agg.mc.cw_roll.Std(), agg.theo_std(8))) ++n;
    if (ok(agg.mc.cw_pitch.Std(), agg.theo_std(9))) ++n;
    if (ok(agg.mc.cw_yaw.Std(), agg.theo_std(10))) ++n;
    if (ok(agg.mc.cw_tx.Std(), agg.theo_std(11))) ++n;
    if (ok(agg.mc.cw_ty.Std(), agg.theo_std(12))) ++n;
    if (ok(agg.mc.cw_tz.Std(), agg.theo_std(13))) ++n;
    return n;
  }();

  if (in_band >= 10) {
    std::cout << "  → PASS: " << in_band
              << "/14 DoF in band (" << in_band_ext
              << "/12 extrinsic); rank=" << agg.fobs_rep.rank << "/14  cond="
              << agg.fobs_rep.cond << ".\n";
    std::cout << "  → §2 UQ validated on converged Stage-2 sub-problem.\n";
  } else if (in_band >= 6) {
    std::cout << "  → PARTIAL: " << in_band << "/14 in band (" << in_band_ext
              << "/12 extrinsic); rank=" << agg.fobs_rep.rank << "/14  cond="
              << agg.fobs_rep.cond << ".\n";
    std::cout << "  → Rotations + LW_ty + t_d_L agree; camera translations and "
                 "t_d_C show 2–13× mismatch (rank-11 / weak t_d_C observability).\n";
    std::cout << "  → §2 UQ partially supported — sufficient for probe STOP; "
                 "full 14/14 may need joint traj–extrinsic FIM.\n";
  } else {
    std::cout << "  → INCONCLUSIVE: " << in_band
              << "/14 in band — investigate FIM assembly or MC design.\n";
  }

  EXPECT_GE(agg.mc.converged, 45);
  EXPECT_TRUE(agg.fobs_rep.is_pd);
  EXPECT_GE(in_band, 6);
}

TEST(TwoStageClosedFormInit, Gate1FixedFextRankDiagnosis) {
  const std::string config_dir = ConfigDirFromExperiments();
  const auto levers = clic_calib::LeverArmConfig::from_yaml(
      config_dir + "/lever_arms.yaml");
  const auto spline_cfg = LoadSplineConfig(config_dir + "/spline.yaml");
  const CoarseExtrinsicInit t_d_prior =
      LoadCoarseExtrinsicsFromYaml(config_dir);
  const RealisticNoiseSpec noise =
      clic_calib::NoiseModel::FromConfigDir(config_dir);

  SyntheticScenarioBundle sc;
  std::shared_ptr<BodyTrajectory> traj;
  Stage2Result s2;
  ASSERT_TRUE(SolveClosedFormStage2AtSeed(
      kRepSeed, levers, spline_cfg, t_d_prior, noise,
      spline_cfg.t_d_max_abs_s, &sc, &traj, &s2));

  const Eigen::MatrixXd F = Stage2ExtrinsicInformation(
      sc, *traj, s2.lidar, s2.camera, levers, noise);
  ASSERT_EQ(F.rows(), 14);
  ASSERT_EQ(F.cols(), 14);

  const FixedFextEigenReport eigen = DiagnoseFixedFextEigen(F);
  const EigenReport summary = AnalyzeSymmetricF(F);

  std::cout << "\n=== GATE 1 / STEP 1 — Fixed F_ext_obs eigen-diagnosis ===\n";
  std::cout << "  Config C converged @ rep seed " << kRepSeed
            << "  (Stage-1 + closed-form init + Stage-2 refine)\n";
  std::cout << "  F_ext_obs: 14×14, prior-free, trajectory fixed, no regularization\n";
  std::cout << std::scientific << std::setprecision(16);
  std::cout << "  λ_min=" << summary.lambda_min << "  λ_max=" << summary.lambda_max
            << "  cond=" << summary.cond << "  rank=" << summary.rank
            << "/14  PD=" << (summary.is_pd ? "yes" : "NO") << std::defaultfloat
            << "\n\n";

  PrintFixedFextEigenReport(eigen);

  std::cout << "\n  DECISION GATE 1 attribution:\n";
  const int null_count = 14 - eigen.numerical_rank;
  std::cout << "  near-null modes (below rank_tol): " << null_count << "\n";

  // Classify: check if t_d_C and translation DoF dominate null modes.
  auto mode_dominant = [&](int mode_idx) {
    const Eigen::VectorXd v = eigen.evecs_asc.col(mode_idx);
    int best = 0;
    v.cwiseAbs().maxCoeff(&best);
    return best;
  };
  const int dom0 = mode_dominant(0);
  const int dom1 = mode_dominant(1);
  const int dom2 = mode_dominant(2);
  std::cout << "  mode0 dominant DoF: " << FixedFextDofLabel(dom0) << "\n";
  std::cout << "  mode1 dominant DoF: " << FixedFextDofLabel(dom1) << "\n";
  std::cout << "  mode2 dominant DoF: " << FixedFextDofLabel(dom2) << "\n";

  const bool t_d_C_in_null = (dom0 == 7) || (dom1 == 7) || (dom2 == 7);
  const auto is_cw_trans = [](int d) { return d >= 11 && d <= 13; };
  const auto is_lw_trans = [](int d) { return d >= 4 && d <= 6; };
  int cw_trans_modes = 0;
  int lw_trans_modes = 0;
  for (int m = 0; m < 3; ++m) {
    const int d = mode_dominant(m);
    if (is_cw_trans(d)) {
      ++cw_trans_modes;
    }
    if (is_lw_trans(d)) {
      ++lw_trans_modes;
    }
  }

  if (t_d_C_in_null && cw_trans_modes == 0 && lw_trans_modes == 0) {
    std::cout << "  → (a) t_d_C weak observability + numerical rank_tol (benign).\n";
  } else if (eigen.strictly_pd && lw_trans_modes >= 2 && cw_trans_modes == 0) {
    std::cout << "  → (c) Strictly PD; rank-11 is numerical tol only. Near-null modes span "
                 "LW_ty/LW_tx/LW_tz (weak curvature, not true null space).\n";
    std::cout << "  → t_d_C NOT in near-null modes; CW translation MC inflation is separate.\n";
    std::cout << "  → NOT (b): cm-level Config C proves extrinsics are observable.\n";
  } else if (cw_trans_modes + lw_trans_modes >= 2 && !eigen.strictly_pd) {
    std::cout << "  → (b) WARNING: translation DoF in true null space — surprising at cm-level.\n";
  } else {
    std::cout << "  → Mixed: inspect mode composition above.\n";
  }

  EXPECT_EQ(summary.rank, eigen.numerical_rank);
  EXPECT_TRUE(summary.is_pd);
  EXPECT_EQ(null_count, 3);
}

void PrintTranslationInfoComparison(const Eigen::MatrixXd& F_fixed,
                                    const Eigen::MatrixXd& F_marg) {
  const int trans_idx[] = {4, 5, 6, 11, 12, 13};
  std::cout << std::setw(12) << "DoF" << std::setw(18) << "F_fixed_ii"
            << std::setw(18) << "F_marg_ii" << std::setw(14) << "marg/fixed"
            << std::setw(18) << "σ_fixed" << std::setw(18) << "σ_marg"
            << std::setw(10) << "σ↑?" << "\n";
  std::cout << std::scientific << std::setprecision(16);
  int n_sigma_up = 0;
  int n_info_down = 0;
  for (int idx : trans_idx) {
    const double ff = F_fixed(idx, idx);
    const double fm = F_marg(idx, idx);
    const double ratio = ff > 0.0 ? fm / ff : 0.0;
    const double sf = ff > 0.0 ? 1.0 / std::sqrt(ff) : 0.0;
    const double sm = fm > 0.0 ? 1.0 / std::sqrt(fm) : 0.0;
    const bool info_down = fm < ff;
    const bool sigma_up = sm > sf;
    if (info_down) {
      ++n_info_down;
    }
    if (sigma_up) {
      ++n_sigma_up;
    }
    std::cout << std::setw(12) << FixedFextDofLabel(idx) << std::setw(18) << ff
              << std::setw(18) << fm << std::setw(14) << ratio
              << std::setw(18) << sf << std::setw(18) << sm << std::setw(10)
              << (sigma_up ? "yes" : "NO") << "\n";
  }
  const auto block_trace = [](const Eigen::MatrixXd& F, int start) {
    return F.block<3, 3>(start, start).trace();
  };
  const double tr_lw_fixed = block_trace(F_fixed, 4);
  const double tr_lw_marg = block_trace(F_marg, 4);
  const double tr_cw_fixed = block_trace(F_fixed, 11);
  const double tr_cw_marg = block_trace(F_marg, 11);
  std::cout << "\n  Translation 3×3 block trace(Information):\n";
  std::cout << "    LW: fixed=" << tr_lw_fixed << "  marginal=" << tr_lw_marg
            << "  ratio=" << (tr_lw_fixed > 0 ? tr_lw_marg / tr_lw_fixed : 0)
            << "\n";
  std::cout << "    CW: fixed=" << tr_cw_fixed << "  marginal=" << tr_cw_marg
            << "  ratio=" << (tr_cw_fixed > 0 ? tr_cw_marg / tr_cw_fixed : 0)
            << "\n";
  std::cout << "  per-DoF: F_marg<F_fixed (info↓): " << n_info_down << "/6"
            << "  σ_marg>σ_fixed: " << n_sigma_up << "/6\n";
  std::cout << std::defaultfloat;
}

TEST(TwoStageClosedFormInit, Gate2MarginalFimAssembly) {
  const std::string config_dir = ConfigDirFromExperiments();
  const auto levers = clic_calib::LeverArmConfig::from_yaml(
      config_dir + "/lever_arms.yaml");
  const auto spline_cfg = LoadSplineConfig(config_dir + "/spline.yaml");
  const CoarseExtrinsicInit t_d_prior =
      LoadCoarseExtrinsicsFromYaml(config_dir);
  const RealisticNoiseSpec noise =
      clic_calib::NoiseModel::FromConfigDir(config_dir);

  SyntheticScenarioBundle sc;
  std::shared_ptr<BodyTrajectory> traj;
  Stage2Result s2;
  ASSERT_TRUE(SolveClosedFormStage2AtSeed(
      kRepSeed, levers, spline_cfg, t_d_prior, noise,
      spline_cfg.t_d_max_abs_s, &sc, &traj, &s2));

  const Eigen::MatrixXd F_fixed = Stage2ExtrinsicInformation(
      sc, *traj, s2.lidar, s2.camera, levers, noise);
  const JointFimResult joint = JointInformationWithMarginal(
      sc, *traj, s2.lidar, s2.camera, levers, noise, kStage1AlphaP,
      kStage1AlphaR, /*joint_refine_before_fim=*/false);

  const FixedFextEigenReport marg_eigen =
      DiagnoseFixedFextEigen(joint.marginal.F_marg);
  const EigenReport fixed_summary = AnalyzeSymmetricF(F_fixed);

  std::cout << "\n=== GATE 2 / STEP 2 — Marginal FIM assembly ===\n";
  std::cout << "  Config C converged @ rep seed " << kRepSeed << "\n";
  std::cout << "  Trajectory: Stage-1 estimate (not GT); prior-free; no regularization\n";
  std::cout << "  Joint F full dim=" << joint.F_full.rows()
            << "  traj block dim=" << joint.marginal.f_rr_dim << "\n";
  std::cout << std::scientific << std::setprecision(16);
  std::cout << "  F_rr: SVD rank=" << joint.marginal.f_rr_svd_rank << "/"
            << joint.marginal.f_rr_dim << "  eigen rank="
            << joint.marginal.f_rr_eigen_rank << "  λ_min="
            << joint.marginal.f_rr_lambda_min << "  λ_max="
            << joint.marginal.f_rr_lambda_max << "\n";
  std::cout << "  Schur: F_marg = F_ee − F_er·pinv(F_rr)·F_re  (pinv via positive "
               "eigenmodes; SVD rank reported separately)\n\n";

  std::cout << "  F_marg (14×14 extrinsic+t_d marginal information):\n";
  std::cout << "    λ_min=" << joint.marginal.f_marg.lambda_min
            << "  λ_max=" << joint.marginal.f_marg.lambda_max
            << "  cond=" << joint.marginal.f_marg.cond
            << "  rank=" << joint.marginal.f_marg.rank << "/14  PD="
            << (joint.marginal.f_marg.is_pd ? "yes" : "NO") << std::defaultfloat
            << "\n\n";

  PrintFixedFextEigenReport(marg_eigen);

  std::cout << "\n  Fixed F_ext vs F_marg — translation information (diag + σ≈1/√F_ii):\n";
  PrintTranslationInfoComparison(F_fixed, joint.marginal.F_marg);

  std::cout << "\n  DECISION GATE 2:\n";
  std::cout << "  Fixed F_ext: rank=" << fixed_summary.rank << "/14  PD="
            << (fixed_summary.is_pd ? "yes" : "NO") << "\n";
  std::cout << "  F_marg:      rank=" << joint.marginal.f_marg.rank << "/14  PD="
            << (joint.marginal.f_marg.is_pd ? "yes" : "NO") << "\n";

  const double tr_lw_r =
      F_fixed.block<3, 3>(4, 4).trace() > 0
          ? joint.marginal.F_marg.block<3, 3>(4, 4).trace() /
                F_fixed.block<3, 3>(4, 4).trace()
          : 0.0;
  const double tr_cw_r =
      F_fixed.block<3, 3>(11, 11).trace() > 0
          ? joint.marginal.F_marg.block<3, 3>(11, 11).trace() /
                F_fixed.block<3, 3>(11, 11).trace()
          : 0.0;

  if (tr_cw_r < 1.0 && tr_lw_r < 1.0) {
    std::cout << "  → CONFIRMED: marginal translation information < fixed "
                 "(trajectory uncertainty propagated).\n";
  } else if (tr_cw_r < 1.0) {
    std::cout << "  → PARTIAL: CW translation block info reduced (trace ratio="
              << tr_cw_r << "); LW block info INCREASED (ratio=" << tr_lw_r
              << ") — indefinite F_rr at decoupled point.\n";
  } else if (tr_lw_r < 1.0 || tr_cw_r < 1.0) {
    std::cout << "  → PARTIAL: one sensor block shows info reduction (LW ratio="
              << tr_lw_r << "  CW ratio=" << tr_cw_r << ").\n";
  } else {
    std::cout << "  → UNEXPECTED: both translation blocks show marginal info ≥ fixed.\n";
  }

  EXPECT_GT(joint.marginal.f_rr_svd_rank, 0);
  EXPECT_LT(joint.marginal.f_rr_svd_rank, joint.marginal.f_rr_dim);
  EXPECT_TRUE(joint.marginal.f_marg.is_pd);
  EXPECT_EQ(joint.marginal.F_marg.rows(), 14);
}

TEST(TwoStageClosedFormInit, Gate5MarginalFimRepSeedTiming) {
  const std::string config_dir = ConfigDirFromExperiments();
  const auto levers = clic_calib::LeverArmConfig::from_yaml(
      config_dir + "/lever_arms.yaml");
  const auto spline_cfg = LoadSplineConfig(config_dir + "/spline.yaml");
  const CoarseExtrinsicInit t_d_prior =
      LoadCoarseExtrinsicsFromYaml(config_dir);
  const RealisticNoiseSpec noise =
      clic_calib::NoiseModel::FromConfigDir(config_dir);

  SyntheticScenarioBundle sc;
  std::shared_ptr<BodyTrajectory> traj;
  Stage2Result s2;
  const auto t0 = std::chrono::steady_clock::now();
  ASSERT_TRUE(SolveClosedFormStage2AtSeed(
      kRepSeed, levers, spline_cfg, t_d_prior, noise,
      spline_cfg.t_d_max_abs_s, &sc, &traj, &s2));
  const auto t1 = std::chrono::steady_clock::now();
  JointFimResult joint = JointInformationWithMarginal(
      sc, *traj, s2.lidar, s2.camera, levers, noise, kStage1AlphaP,
      kStage1AlphaR, /*joint_refine_before_fim=*/true);
  const auto t2 = std::chrono::steady_clock::now();
  const double solve_s =
      std::chrono::duration<double>(t1 - t0).count();
  const double joint_s =
      std::chrono::duration<double>(t2 - t1).count();

  std::cout << "\n=== GATE 5b rep-seed timing (seed=" << kRepSeed << ") ===\n";
  std::cout << std::scientific << std::setprecision(16);
  std::cout << "  solve_s=" << solve_s << "  joint_FIM_s=" << joint_s << "\n";
  std::cout << "  joint dim=" << joint.F_full.rows()
            << "  F_rr SVD rank=" << joint.marginal.f_rr_svd_rank << "/"
            << joint.marginal.f_rr_dim
            << "  eigen rank=" << joint.marginal.f_rr_eigen_rank << "\n";
  std::cout << "  F_marg rank=" << joint.marginal.f_marg.rank
            << "/14  PD=" << (joint.marginal.f_marg.is_pd ? "yes" : "NO")
            << std::defaultfloat << "\n";

  EXPECT_GT(joint.marginal.f_rr_svd_rank, 0);
  EXPECT_EQ(joint.marginal.F_marg.rows(), 14);
}

TEST(TwoStageClosedFormInit, Gate5MarginalFimMcComparison) {
  const std::string config_dir = ConfigDirFromExperiments();
  const auto levers = clic_calib::LeverArmConfig::from_yaml(
      config_dir + "/lever_arms.yaml");
  const auto spline_cfg = LoadSplineConfig(config_dir + "/spline.yaml");
  const CoarseExtrinsicInit t_d_prior =
      LoadCoarseExtrinsicsFromYaml(config_dir);
  const RealisticNoiseSpec noise =
      clic_calib::NoiseModel::FromConfigDir(config_dir);

  const int num_seeds = []() {
    if (const char* env = std::getenv("CLIC_MARGINAL_N")) {
      return std::max(1, std::atoi(env));
    }
    return kNumSeeds;
  }();

  std::cout << "\n=== GATE 5b — Fixed vs Marginal FIM↔MC (N=" << num_seeds
            << ") ===\n";
  std::cout << "  (a) fixed-trajectory F_ext_obs⁻¹ — omits Stage-1 traj uncertainty\n";
  std::cout << "  (b) Schur-marginal F_ext⁻¹ — traj marginalized via SVD pinv(F_rr)\n";
  std::cout << "  Band: empirical/theoretical ∈ [" << kRatioLo << ", " << kRatioHi
            << "]\n\n";

  const FimMcAggregateResult agg =
      CollectClosedFormStage2FimMc(levers, spline_cfg, t_d_prior, noise,
                                   num_seeds, /*include_marginal=*/true,
                                   /*joint_refine_before_fim=*/true);
  std::cout << "  MC converged: " << agg.mc.converged << "/" << num_seeds << "\n";
  std::cout << std::scientific << std::setprecision(16);
  std::cout << "  (a) F_ext fixed @ rep: rank=" << agg.fobs_rep.rank << "/"
            << agg.fobs_rep.dim << "  cond=" << agg.fobs_rep.cond << "  PD="
            << (agg.fobs_rep.is_pd ? "yes" : "NO") << "\n";
  std::cout << "  Joint F full dim=" << agg.joint_full_dim
            << "  traj rest dim=" << agg.marginal_rep.f_rr_dim << "\n";
  std::cout << "  F_rr @ rep: SVD rank=" << agg.marginal_rep.f_rr_svd_rank << "/"
            << agg.marginal_rep.f_rr_dim << "  eigen rank="
            << agg.marginal_rep.f_rr_eigen_rank << "  λ_min="
            << agg.marginal_rep.f_rr_lambda_min << "  λ_max="
            << agg.marginal_rep.f_rr_lambda_max << "\n";
  std::cout << "  (b) F_ext marginal @ rep: rank=" << agg.marginal_rep.f_marg.rank
            << "/14  cond=" << agg.marginal_rep.f_marg.cond << "  PD="
            << (agg.marginal_rep.f_marg.is_pd ? "yes" : "NO") << std::defaultfloat
            << "\n\n";

  const DualRatioSummary dual = PrintFimMcDualRatioTable(
      agg.mc, agg.theo_std, agg.theo_std_marginal);

  std::cout << "\n=== GATE 5b verdict ===\n";
  std::cout << "  (a) fixed-trajectory: " << dual.in_band_fixed
            << "/14 in band (" << dual.in_band_ext_fixed
            << "/12 extrinsic) — sub-optimality gap (traj uncertainty omitted)\n";
  std::cout << "  (b) marginal FIM:     " << dual.in_band_marginal
            << "/14 in band (" << dual.in_band_ext_marginal
            << "/12 extrinsic) — valid §2 UQ reference\n";
  const int trans_fixed_in = [&]() {
    int n = 0;
    auto ok = [&](double emp, double th) {
      return th > 1e-18 && emp / th >= kRatioLo && emp / th <= kRatioHi;
    };
    if (ok(agg.mc.lw_tx.Std(), agg.theo_std(4))) ++n;
    if (ok(agg.mc.lw_tz.Std(), agg.theo_std(6))) ++n;
    if (ok(agg.mc.cw_tx.Std(), agg.theo_std(11))) ++n;
    if (ok(agg.mc.cw_ty.Std(), agg.theo_std(12))) ++n;
    if (ok(agg.mc.cw_tz.Std(), agg.theo_std(13))) ++n;
    return n;
  }();
  const int trans_marg_in = [&]() {
    int n = 0;
    auto ok = [&](double emp, double th) {
      return th > 1e-18 && emp / th >= kRatioLo && emp / th <= kRatioHi;
    };
    if (ok(agg.mc.lw_tx.Std(), agg.theo_std_marginal(4))) ++n;
    if (ok(agg.mc.lw_tz.Std(), agg.theo_std_marginal(6))) ++n;
    if (ok(agg.mc.cw_tx.Std(), agg.theo_std_marginal(11))) ++n;
    if (ok(agg.mc.cw_ty.Std(), agg.theo_std_marginal(12))) ++n;
    if (ok(agg.mc.cw_tz.Std(), agg.theo_std_marginal(13))) ++n;
    return n;
  }();
  std::cout << "  Translation DoF in band: fixed " << trans_fixed_in
            << "/5 → marginal " << trans_marg_in
            << "/5 (decoupling cost quantified by fixed/marg theo ratio)\n";

  if (dual.in_band_marginal >= dual.in_band_fixed &&
      trans_marg_in >= trans_fixed_in) {
    std::cout << "  → CONFIRMED: marginal FIM closes §2 UQ; fixed-trajectory gap "
                 "is two-stage decoupling artifact.\n";
  } else if (trans_marg_in > trans_fixed_in ||
             dual.in_band_marginal >= dual.in_band_fixed - 2) {
    std::cout << "  → PARTIAL: marginal improves some DoF (notably CW_tz); MC measures "
                 "two-stage estimator, not joint MAP — decoupling gap quantified.\n";
    std::cout << "  → F_rr SVD rank=" << agg.marginal_rep.f_rr_svd_rank << "/"
              << agg.marginal_rep.f_rr_dim
              << " at decoupled linearization (λ_min(F_rr) may be < 0).\n";
  } else {
    std::cout << "  → INVESTIGATE: marginal did not improve as expected at decoupled "
                 "linearization point.\n";
  }

  EXPECT_GE(agg.mc.converged, num_seeds >= 50 ? 45 : num_seeds - 1);
  if (num_seeds > static_cast<int>(kRepSeed - kSeedBase)) {
    EXPECT_GT(agg.marginal_rep.f_rr_svd_rank, 0);
    EXPECT_TRUE(agg.marginal_rep.f_marg.is_pd);
  }
  if (num_seeds >= 50) {
    EXPECT_GE(dual.in_band_fixed, 6);
  }
}

TEST(TwoStageClosedFormInit, Gate3FimMcSideBySideComparison) {
  const std::string config_dir = ConfigDirFromExperiments();
  const auto levers = clic_calib::LeverArmConfig::from_yaml(
      config_dir + "/lever_arms.yaml");
  const auto spline_cfg = LoadSplineConfig(config_dir + "/spline.yaml");
  const CoarseExtrinsicInit t_d_prior =
      LoadCoarseExtrinsicsFromYaml(config_dir);
  const RealisticNoiseSpec noise =
      clic_calib::NoiseModel::FromConfigDir(config_dir);

  const int num_seeds = []() {
    if (const char* env = std::getenv("CLIC_MARGINAL_N")) {
      return std::max(1, std::atoi(env));
    }
    return kNumSeeds;
  }();

  std::cout << "\n=== STEP 3 / DECISION GATE 3 — Fixed vs Marginal FIM↔MC (N="
            << num_seeds << ") ===\n";
  std::cout << "  Same MC as Gate 5: Config C closed-form Stage-2, prior-free\n";
  std::cout << "  (a) empirical_std / fixed F_ext⁻¹  (fixed trajectory)\n";
  std::cout << "  (b) empirical_std / marginal F_marg⁻¹  (Schur @ decoupled "
               "point, no joint refine)\n";
  std::cout << "  Band: [" << kRatioLo << ", " << kRatioHi << "]\n\n";

  const FimMcAggregateResult agg =
      CollectClosedFormStage2FimMc(levers, spline_cfg, t_d_prior, noise,
                                   num_seeds, /*include_marginal=*/true,
                                   /*joint_refine_before_fim=*/false);
  std::cout << "  MC converged: " << agg.mc.converged << "/" << num_seeds << "\n";
  std::cout << std::scientific << std::setprecision(16);
  std::cout << "  (a) F_ext fixed @ rep: rank=" << agg.fobs_rep.rank << "/"
            << agg.fobs_rep.dim << "  PD=" << (agg.fobs_rep.is_pd ? "yes" : "NO")
            << "\n";
  std::cout << "  (b) F_marg @ rep: rank=" << agg.marginal_rep.f_marg.rank
            << "/14  PD=" << (agg.marginal_rep.f_marg.is_pd ? "yes" : "NO")
            << "  F_rr SVD rank=" << agg.marginal_rep.f_rr_svd_rank << "/"
            << agg.marginal_rep.f_rr_dim << "  λ_min(F_rr)="
            << agg.marginal_rep.f_rr_lambda_min << std::defaultfloat << "\n\n";

  const DualRatioSummary dual = PrintFimMcDualRatioTable(
      agg.mc, agg.theo_std, agg.theo_std_marginal);
  const int trans_fixed_in = CountTranslationInBand(agg.mc, agg.theo_std);
  const int trans_marg_in =
      CountTranslationInBand(agg.mc, agg.theo_std_marginal);

  std::cout << "\n=== DECISION GATE 3 — §2 UQ verdict ===\n";
  std::cout << "  (a) fixed:    " << dual.in_band_fixed << "/14 in band ("
            << dual.in_band_ext_fixed << "/12 extrinsic); translations "
            << trans_fixed_in << "/5\n";
  std::cout << "  (b) marginal: " << dual.in_band_marginal << "/14 in band ("
            << dual.in_band_ext_marginal << "/12 extrinsic); translations "
            << trans_marg_in << "/5\n";
  std::cout << "  (a)−(b) gap quantifies two-stage sub-optimality (fixed omits "
               "trajectory uncertainty).\n";

  const bool marginal_validates =
      dual.in_band_marginal >= 10 && trans_marg_in >= 4;
  if (marginal_validates) {
    std::cout << "  → CONFIRMED: marginal FIM validates §2 UQ. Fixed-trajectory "
                 "mismatch was omitted trajectory uncertainty; MARGINAL FIM is the "
                 "theoretically correct covariance predictor.\n";
  } else {
    std::cout << "  → NOT CONFIRMED: marginal FIM does not bring most DoF into "
                 "band.\n";
    PrintMarginalOutOfBandDofs(agg.mc, agg.theo_std_marginal);
    PrintMcBiasTable(agg.mc);
    std::cout << "\n  Diagnosis:\n";
    std::cout << "  • MC measures two-stage estimator variance (Stage-1 traj "
                 "noise → Stage-2 extrinsics).\n";
    std::cout << "  • Marginal FIM @ decoupled Config C point (no joint refine) "
                 "may not match this estimand.\n";
    if (agg.marginal_rep.f_rr_lambda_min < 0.0) {
      std::cout << "  • F_rr indefinite (λ_min=" << std::scientific
                << std::setprecision(6) << agg.marginal_rep.f_rr_lambda_min
                << std::defaultfloat
                << ") — Schur at decoupled linearization can mis-state LW block.\n";
    }
    std::cout << "  • Check bias table: |mean| ≪ emp_std supports zero-mean "
                 "covariance assumption.\n";
    std::cout << "  → STOP: §2 UQ not closed by marginal Schur alone; do not "
                 "claim 14/14 validation.\n";
  }

  EXPECT_GE(agg.mc.converged, num_seeds >= 50 ? 45 : num_seeds - 1);
  if (num_seeds >= 50) {
    EXPECT_GE(dual.in_band_fixed, 6);
  }
}

TEST(TwoStageClosedFormInit, Gate1TLWUmeyamaPreIteration) {
  const std::string config_dir = ConfigDirFromExperiments();
  const auto levers = clic_calib::LeverArmConfig::from_yaml(
      config_dir + "/lever_arms.yaml");
  const auto spline_cfg = LoadSplineConfig(config_dir + "/spline.yaml");
  const CoarseExtrinsicInit t_d_nominal =
      LoadCoarseExtrinsicsFromYaml(config_dir);
  const RealisticNoiseSpec noise =
      clic_calib::NoiseModel::FromConfigDir(config_dir);

  RunningStats coarse_rot_deg, coarse_trans_mm;
  RunningStats umeyama_rot_deg, umeyama_trans_mm;
  RunningStats td0_rot_deg, td0_trans_mm;
  RunningStats umeyama_rms_mm, lw_pairs;
  int ok_count = 0;

  std::cout << "\n=== GATE 1 — closed-form T_LW (Umeyama, pre-iteration, N="
            << kNumSeeds << ") ===\n";
  std::cout << "  Per-scan sphere center in L + p_G^W from Stage-1 traj;\n";
  std::cout << "  pairing t_d = nominal yaml (rig spec); init t_d^L := 0\n";

  for (int i = 0; i < kNumSeeds; ++i) {
    const uint32_t seed = kSeedBase + static_cast<uint32_t>(i);
    const SyntheticScenarioBundle sc =
        BuildNearFieldFimNoisyScenarioWithAttitude(seed, noise);
    const Stage1Result s1 = FitStage1WithAttitude(
        sc, levers, spline_cfg, kStage1AlphaP, kStage1AlphaR);

    const CoarseExtrinsicInit coarse = MakeCoarseExtrinsicInitPerturbed(
        sc.gt.T_LW, sc.gt.T_CW, sc.gt.t_d_L_s, sc.gt.t_d_C_s, seed);
    const ExtrinsicSixDofErrors coarse_e =
        ExtrinsicError(coarse.T_LW, sc.gt.T_LW);
    coarse_rot_deg.Push(coarse_e.rot_err_rad.norm() * 180.0 / M_PI);
    coarse_trans_mm.Push(coarse_e.trans_err_m.norm() * 1e3);

    SE3d T_LW;
    UmeyamaRigid u;
    int np = 0;
    if (!InitTLWFromLidarSphereCenters(*s1.trajectory, sc.lidar_obs,
                                       levers.L_B_to_G, t_d_nominal.t_d_L_s,
                                       kSphereRadiusM, &T_LW, &u, &np)) {
      continue;
    }
    ++ok_count;
    const ExtrinsicSixDofErrors geo_e = ExtrinsicError(T_LW, sc.gt.T_LW);
    umeyama_rot_deg.Push(geo_e.rot_err_rad.norm() * 180.0 / M_PI);
    umeyama_trans_mm.Push(geo_e.trans_err_m.norm() * 1e3);
    umeyama_rms_mm.Push(u.rms_m * 1e3);
    lw_pairs.Push(static_cast<double>(np));

    SE3d T_td0;
    if (InitTLWFromLidarSphereCenters(*s1.trajectory, sc.lidar_obs,
                                      levers.L_B_to_G, 0.0, kSphereRadiusM,
                                      &T_td0, nullptr, nullptr)) {
      const ExtrinsicSixDofErrors e0 = ExtrinsicError(T_td0, sc.gt.T_LW);
      td0_rot_deg.Push(e0.rot_err_rad.norm() * 180.0 / M_PI);
      td0_trans_mm.Push(e0.trans_err_m.norm() * 1e3);
    }
  }

  std::cout << "  Umeyama succeeded=" << ok_count << "/" << kNumSeeds << "\n";
  PrintBiasStd("coarse rot |LW| err", coarse_rot_deg, "deg", 4);
  PrintBiasStd("coarse |LW trans| err", coarse_trans_mm, "mm", 3);
  PrintBiasStd("Umeyama rot |LW| err", umeyama_rot_deg, "deg", 4);
  PrintBiasStd("Umeyama |LW trans| err", umeyama_trans_mm, "mm", 3);
  PrintBiasStd("pair residual rms", umeyama_rms_mm, "mm", 3);
  PrintBiasStd("LiDAR pairs used", lw_pairs, "", 1);
  if (td0_trans_mm.n > 0) {
    std::cout << "  (diagnostic: pairing t_d=0 → trans bias ~"
              << td0_trans_mm.mean << " mm from motion×delay)\n";
    PrintBiasStd("  [diag] t_d=0 rot err", td0_rot_deg, "deg", 4);
    PrintBiasStd("  [diag] t_d=0 trans err", td0_trans_mm, "mm", 3);
  }

  const bool gate_pass = ok_count >= 45 && umeyama_rot_deg.mean < 5.0 &&
                         umeyama_trans_mm.mean < 50.0 &&
                         coarse_rot_deg.mean > 5.0;
  std::cout << "\n=== GATE 1 verdict ===\n";
  std::cout << (gate_pass ? "  → PASS: Umeyama T_LW init within few deg/cm\n"
                          : "  → FAIL: init not sufficiently better than coarse\n");

  EXPECT_GE(ok_count, 45);
  EXPECT_LT(umeyama_rot_deg.mean, 5.0);
  EXPECT_LT(umeyama_trans_mm.mean, 50.0);
  EXPECT_GT(coarse_rot_deg.mean, 5.0);
}

TEST(TwoStageClosedFormInit, Gate2TCWPnPPreIteration) {
  const std::string config_dir = ConfigDirFromExperiments();
  const auto levers = clic_calib::LeverArmConfig::from_yaml(
      config_dir + "/lever_arms.yaml");
  const auto spline_cfg = LoadSplineConfig(config_dir + "/spline.yaml");
  const CoarseExtrinsicInit t_d_nominal =
      LoadCoarseExtrinsicsFromYaml(config_dir);
  const RealisticNoiseSpec noise =
      clic_calib::NoiseModel::FromConfigDir(config_dir);
  const PinholeIntrinsics K{600.0, 600.0, 320.0, 240.0};
  const RadtanDistortion dist;
  const Eigen::Vector3d L_G_to_M = levers.L_G_to_M.at(0);

  RunningStats coarse_rot_deg, coarse_trans_mm;
  RunningStats pnp_rot_deg, pnp_trans_mm;
  RunningStats best_frame_reproj_px, mean_frame_reproj_px, frames_ok;
  int ok_count = 0;

  std::cout << "\n=== GATE 2 — closed-form T_CW (per-frame PnP, pre-iteration, N="
            << kNumSeeds << ") ===\n";
  std::cout << "  Per-frame tag-local IPPE/EPnP + SE(3) mean + batch ITERATIVE;\n";
  std::cout << "  aggregate = lowest global reproj; pairing t_d = nominal yaml\n";

  for (int i = 0; i < kNumSeeds; ++i) {
    const uint32_t seed = kSeedBase + static_cast<uint32_t>(i);
    const SyntheticScenarioBundle sc =
        BuildNearFieldFimNoisyScenarioWithAttitude(seed, noise);
    const Stage1Result s1 = FitStage1WithAttitude(
        sc, levers, spline_cfg, kStage1AlphaP, kStage1AlphaR);
    const std::vector<AprilTagObservation> tags =
        SynthesizeTagObsForTrajectory(
            sc.tag_obs, *s1.trajectory, sc.gt.T_CW, levers.L_B_to_G, L_G_to_M,
            t_d_nominal.t_d_C_s, K, dist, noise, seed);

    const CoarseExtrinsicInit coarse = MakeCoarseExtrinsicInitPerturbed(
        sc.gt.T_LW, sc.gt.T_CW, sc.gt.t_d_L_s, sc.gt.t_d_C_s, seed);
    const ExtrinsicSixDofErrors coarse_e =
        ExtrinsicError(coarse.T_CW, sc.gt.T_CW);
    coarse_rot_deg.Push(coarse_e.rot_err_rad.norm() * 180.0 / M_PI);
    coarse_trans_mm.Push(coarse_e.trans_err_m.norm() * 1e3);

    SE3d T_CW;
    int n_ok = 0;
    int n_total = 0;
    double best_reproj = 0.0;
    double mean_reproj = 0.0;
    if (!InitTCWFromAprilTagPnPPerFrame(
            *s1.trajectory, tags, levers.L_B_to_G, L_G_to_M,
            t_d_nominal.t_d_C_s, K, dist, &T_CW, &n_ok, &n_total, &best_reproj,
            &mean_reproj)) {
      continue;
    }
    ++ok_count;
    const ExtrinsicSixDofErrors pnp_e = ExtrinsicError(T_CW, sc.gt.T_CW);
    pnp_rot_deg.Push(pnp_e.rot_err_rad.norm() * 180.0 / M_PI);
    pnp_trans_mm.Push(pnp_e.trans_err_m.norm() * 1e3);
    best_frame_reproj_px.Push(best_reproj);
    mean_frame_reproj_px.Push(mean_reproj);
    frames_ok.Push(static_cast<double>(n_ok));
  }

  std::cout << "  PnP aggregate succeeded=" << ok_count << "/" << kNumSeeds
            << "\n";
  PrintBiasStd("coarse rot |CW| err", coarse_rot_deg, "deg", 4);
  PrintBiasStd("coarse |CW trans| err", coarse_trans_mm, "mm", 3);
  PrintBiasStd("PnP rot |CW| err", pnp_rot_deg, "deg", 4);
  PrintBiasStd("PnP |CW trans| err", pnp_trans_mm, "mm", 3);
  PrintBiasStd("best-frame self reproj", best_frame_reproj_px, "px", 3);
  PrintBiasStd("global reproj (picked)", mean_frame_reproj_px, "px", 3);
  PrintBiasStd("PnP frames solved", frames_ok, "", 1);

  const bool gate_pass = ok_count >= 45 && pnp_rot_deg.mean < 5.0 &&
                         pnp_trans_mm.mean < 50.0 &&
                         coarse_rot_deg.mean > 5.0;
  std::cout << "\n=== GATE 2 verdict ===\n";
  std::cout << (gate_pass ? "  → PASS: per-frame PnP T_CW init within few deg/cm\n"
                          : "  → FAIL: init not sufficiently better than coarse\n");

  EXPECT_GE(ok_count, 45);
  EXPECT_LT(pnp_rot_deg.mean, 5.0);
  EXPECT_LT(pnp_trans_mm.mean, 50.0);
  EXPECT_GT(coarse_rot_deg.mean, 5.0);
}

TEST(TwoStageClosedFormInit, Gate3BasinDiscriminationLadder) {
  const std::string config_dir = ConfigDirFromExperiments();
  const auto levers = clic_calib::LeverArmConfig::from_yaml(
      config_dir + "/lever_arms.yaml");
  const auto spline_cfg = LoadSplineConfig(config_dir + "/spline.yaml");
  const CoarseExtrinsicInit t_d_prior =
      LoadCoarseExtrinsicsFromYaml(config_dir);
  const RealisticNoiseSpec noise =
      clic_calib::NoiseModel::FromConfigDir(config_dir);

  std::cout << "\n=== STEP 3 / GATE 3 — basin discrimination ladder (N="
            << kNumSeeds << ") ===\n";
  std::cout << "  A Oracle: GT traj + GT init (residual/Jacobian sanity)\n";
  std::cout << "  B GT + closed-form init (basin given perfect trajectory)\n";
  std::cout << "  C Stage-1 + closed-form init (production path)\n";
  std::cout << "  D Stage-1 + coarse ±10° (prior failure reproduction)\n";
  std::cout << "  cm-level: |trans|<" << kCmTransMm << " mm, |LW pitch|<"
            << kCmPitchDeg << "°; wrong-basin: |trans|>" << kWrongBasinTransMm
            << " mm\n";

  const LadderExtrinsicStats cfg_a = RunBasinLadderConfig(
      {"A Oracle", LadderTraj::kGt, LadderInit::kOracle, false}, levers,
      spline_cfg, t_d_prior, noise);
  const LadderExtrinsicStats cfg_b = RunBasinLadderConfig(
      {"B GT+closed-form", LadderTraj::kGt, LadderInit::kClosedForm, false},
      levers, spline_cfg, t_d_prior, noise);
  const LadderExtrinsicStats cfg_c = RunBasinLadderConfig(
      {"C Stage-1+closed-form", LadderTraj::kStage1, LadderInit::kClosedForm,
       true},
      levers, spline_cfg, t_d_prior, noise);
  const LadderExtrinsicStats cfg_d = RunBasinLadderConfig(
      {"D Stage-1+coarse", LadderTraj::kStage1, LadderInit::kCoarse, false},
      levers, spline_cfg, t_d_prior, noise);

  PrintLadderConfigBlock("A — Oracle (GT traj, GT init)", cfg_a);
  PrintLadderConfigBlock("B — GT traj, closed-form init", cfg_b);
  PrintLadderConfigBlock("C — Stage-1 traj, closed-form init", cfg_c);
  PrintLadderConfigBlock("D — Stage-1 traj, coarse ±10°", cfg_d);

  const auto cm_rate = [](const LadderExtrinsicStats& s) {
    return static_cast<double>(s.cm_success) / static_cast<double>(kNumSeeds);
  };
  const auto wrong_rate = [](const LadderExtrinsicStats& s) {
    return static_cast<double>(s.wrong_basin) / static_cast<double>(kNumSeeds);
  };

  const bool b_cm = cfg_b.cm_success >= 45 && cfg_b.lw_trans_mm.mean < kCmTransMm &&
                    cfg_b.cw_trans_mm.mean < kCmTransMm;
  const bool c_cm = cfg_c.cm_success >= 45 && cfg_c.lw_trans_mm.mean < kCmTransMm &&
                    cfg_c.cw_trans_mm.mean < kCmTransMm;
  const bool d_fails = cfg_d.wrong_basin >= 30 || cfg_d.lw_trans_mm.mean > 500.0;
  const bool a_oracle = cfg_a.cm_success >= 45 &&
                        cfg_a.lw_trans_mm.mean < 5.0 &&
                        cfg_a.cw_trans_mm.mean < 5.0;

  std::cout << "\n=== GATE 3 verdict ===\n";
  if (b_cm && c_cm && d_fails) {
    std::cout << "  → CONFIRMED: purely a basin problem; closed-form init fixes production path (C).\n";
    std::cout << "  → Proceed to STEP 4.\n";
  } else if (!b_cm) {
    std::cout << "  → STOP: B failed (GT traj + closed-form still wrong) — residual Stage-2 bug beyond basin.\n";
    std::cout << "  → Inspect diverging DoF / worst eigenvector before trusting closed-form init.\n";
  } else if (b_cm && !c_cm) {
    std::cout << "  → STOP: C failed but B succeeded — Stage-1 trajectory error spoils init/refinement.\n";
    std::cout << "  → Re-open yaw-bottleneck; AprilTag-constrained yaw refinement likely needed.\n";
  } else {
    std::cout << "  → INCONCLUSIVE: check cm-level / wrong-basin rates above.\n";
  }
  std::cout << "  basin success rate: A=" << cm_rate(cfg_a) * 100.0 << "%  B="
            << cm_rate(cfg_b) * 100.0 << "%  C=" << cm_rate(cfg_c) * 100.0
            << "%  D=" << cm_rate(cfg_d) * 100.0 << "%\n";
  std::cout << "  wrong-basin rate:   A=" << wrong_rate(cfg_a) * 100.0 << "%  B="
            << wrong_rate(cfg_b) * 100.0 << "%  C=" << wrong_rate(cfg_c) * 100.0
            << "%  D=" << wrong_rate(cfg_d) * 100.0 << "%\n";

  EXPECT_TRUE(a_oracle);
  EXPECT_GE(cfg_a.converged, 45);
  EXPECT_GE(cfg_b.cm_success, 45);
  EXPECT_GE(cfg_c.cm_success, 45);
  EXPECT_LT(cfg_b.lw_trans_mm.mean, kCmTransMm);
  EXPECT_LT(cfg_b.cw_trans_mm.mean, kCmTransMm);
  EXPECT_LT(cfg_c.lw_trans_mm.mean, kCmTransMm);
  EXPECT_LT(cfg_c.cw_trans_mm.mean, kCmTransMm);
  EXPECT_GE(cfg_d.wrong_basin, 30);
  EXPECT_GT(cfg_d.lw_trans_mm.mean, 500.0);
}

TEST(TwoStageClosedFormInit, Layer1InitOnlyGtTrajectory) {
  const std::string config_dir = ConfigDirFromExperiments();
  const auto levers = clic_calib::LeverArmConfig::from_yaml(
      config_dir + "/lever_arms.yaml");
  const CoarseExtrinsicInit t_d_prior =
      LoadCoarseExtrinsicsFromYaml(config_dir);
  const RealisticNoiseSpec noise =
      clic_calib::NoiseModel::FromConfigDir(config_dir);

  RunningStats coarse_lw_mm, geo_lw_mm, coarse_cw_mm, geo_cw_mm;
  RunningStats umeyama_rms_mm, pnp_reproj_px;

  std::cout << "\n=== LAYER 1 — init-only error (GT trajectory, N=" << kNumSeeds
            << ") ===\n";
  std::cout << "  Compare coarse ±1m/10° vs closed-form (no Ceres polish)\n";

  for (int i = 0; i < kNumSeeds; ++i) {
    const uint32_t seed = kSeedBase + static_cast<uint32_t>(i);
    const SyntheticScenarioBundle sc =
        BuildNearFieldFimNoisyScenarioWithAttitude(seed, noise);

    const CoarseExtrinsicInit coarse = MakeCoarseExtrinsicInitPerturbed(
        sc.gt.T_LW, sc.gt.T_CW, sc.gt.t_d_L_s, sc.gt.t_d_C_s, seed);
    const GeometricInitReport geo = MakeGeometricExtrinsicInit(
        sc.gt_traj, sc.lidar_obs, sc.tag_obs, levers, kSphereRadiusM, t_d_prior);

    coarse_lw_mm.Push(
        ExtrinsicError(coarse.T_LW, sc.gt.T_LW).trans_err_m.norm() * 1e3);
    coarse_cw_mm.Push(
        ExtrinsicError(coarse.T_CW, sc.gt.T_CW).trans_err_m.norm() * 1e3);
    geo_lw_mm.Push(
        ExtrinsicError(geo.init.T_LW, sc.gt.T_LW).trans_err_m.norm() * 1e3);
    geo_cw_mm.Push(
        ExtrinsicError(geo.init.T_CW, sc.gt.T_CW).trans_err_m.norm() * 1e3);
    umeyama_rms_mm.Push(geo.lw_umeyama.rms_m * 1e3);
    pnp_reproj_px.Push(geo.pnp_reproj_px_rms);
  }

  PrintBiasStd("coarse |LW| err", coarse_lw_mm, "mm", 3);
  PrintBiasStd("coarse |CW| err", coarse_cw_mm, "mm", 3);
  PrintBiasStd("geo init |LW| err", geo_lw_mm, "mm", 3);
  PrintBiasStd("geo init |CW| err", geo_cw_mm, "mm", 3);
  PrintBiasStd("LiDAR Umeyama fit rms", umeyama_rms_mm, "mm", 3);
  PrintBiasStd("PnP reproj rms", pnp_reproj_px, "px", 3);

  EXPECT_LT(geo_lw_mm.mean, 200.0);
  EXPECT_LT(geo_cw_mm.mean, 200.0);
}

TEST(TwoStageClosedFormInit, Layer2GtTrajGeometricPlusPolish) {
  const std::string config_dir = ConfigDirFromExperiments();
  const auto levers = clic_calib::LeverArmConfig::from_yaml(
      config_dir + "/lever_arms.yaml");
  const auto spline_cfg = LoadSplineConfig(config_dir + "/spline.yaml");
  const CoarseExtrinsicInit t_d_prior =
      LoadCoarseExtrinsicsFromYaml(config_dir);
  const RealisticNoiseSpec noise =
      clic_calib::NoiseModel::FromConfigDir(config_dir);

  RunningStats lw_mm, cw_mm;
  int converged = 0;

  std::cout << "\n=== LAYER 2 — GT traj + geometric init + Stage-2 polish (N="
            << kNumSeeds << ") ===\n";

  for (int i = 0; i < kNumSeeds; ++i) {
    const SyntheticScenarioBundle sc =
        BuildNearFieldFimNoisyScenarioWithAttitude(
            kSeedBase + static_cast<uint32_t>(i), noise);
    const GeometricInitReport geo = MakeGeometricExtrinsicInit(
        sc.gt_traj, sc.lidar_obs, sc.tag_obs, levers, kSphereRadiusM, t_d_prior);
    const Stage2Result s2 = SolveStage2Extrinsics(
        sc, sc.gt_traj, levers, noise, geo.init, spline_cfg.t_d_max_abs_s);
    if (!s2.converged) {
      continue;
    }
    ++converged;
    lw_mm.Push(
        ExtrinsicError(StateToSE3(s2.lidar), sc.gt.T_LW).trans_err_m.norm() *
        1e3);
    cw_mm.Push(
        ExtrinsicError(StateToSE3(s2.camera), sc.gt.T_CW).trans_err_m.norm() *
        1e3);
  }

  std::cout << "  converged=" << converged << "/" << kNumSeeds << "\n";
  PrintBiasStd("|LW trans| err", lw_mm, "mm", 3);
  PrintBiasStd("|CW trans| err", cw_mm, "mm", 3);
  EXPECT_GE(converged, 45);
  EXPECT_LT(lw_mm.mean, 50.0);
  EXPECT_LT(cw_mm.mean, 50.0);
}

TEST(TwoStageClosedFormInit, Layer3Stage1TrajGeometricGate3) {
  const std::string config_dir = ConfigDirFromExperiments();
  const auto levers = clic_calib::LeverArmConfig::from_yaml(
      config_dir + "/lever_arms.yaml");
  const auto spline_cfg = LoadSplineConfig(config_dir + "/spline.yaml");
  const CoarseExtrinsicInit t_d_prior =
      LoadCoarseExtrinsicsFromYaml(config_dir);
  const RealisticNoiseSpec noise =
      clic_calib::NoiseModel::FromConfigDir(config_dir);

  RunningStats init_lw_mm, init_cw_mm;
  RunningStats lw_pitch, lw_trans_mm, cw_pitch, cw_trans_mm;
  RunningStats t_d_L, t_d_C;
  RunningStats stage2_cond;
  int converged = 0;

  std::cout << "\n=== LAYER 3 / GATE 3 — Stage-1 traj + geometric init (N="
            << kNumSeeds << ") ===\n";

  for (int i = 0; i < kNumSeeds; ++i) {
    const SyntheticScenarioBundle sc =
        BuildNearFieldFimNoisyScenarioWithAttitude(
            kSeedBase + static_cast<uint32_t>(i), noise);
    const Stage1Result s1 = FitStage1WithAttitude(
        sc, levers, spline_cfg, kStage1AlphaP, kStage1AlphaR);
    const PinholeIntrinsics K{600.0, 600.0, 320.0, 240.0};
    const RadtanDistortion dist;
    SyntheticScenarioBundle sc_s1 = sc;
    sc_s1.tag_obs = SynthesizeTagObsForTrajectory(
        sc.tag_obs, *s1.trajectory, sc.gt.T_CW, levers.L_B_to_G,
        levers.L_G_to_M.at(0), t_d_prior.t_d_C_s, K, dist, noise,
        kSeedBase + static_cast<uint32_t>(i));
    const GeometricInitReport geo = MakeGeometricExtrinsicInit(
        *s1.trajectory, sc_s1.lidar_obs, sc_s1.tag_obs, levers, kSphereRadiusM,
        t_d_prior);
    init_lw_mm.Push(
        ExtrinsicError(geo.init.T_LW, sc.gt.T_LW).trans_err_m.norm() * 1e3);
    init_cw_mm.Push(
        ExtrinsicError(geo.init.T_CW, sc.gt.T_CW).trans_err_m.norm() * 1e3);

    const Stage2Result s2 = SolveStage2Extrinsics(
        sc_s1, *s1.trajectory, levers, noise, geo.init, spline_cfg.t_d_max_abs_s);
    if (!s2.converged) {
      continue;
    }
    ++converged;
    const ExtrinsicSixDofErrors lw_e =
        ExtrinsicError(StateToSE3(s2.lidar), sc.gt.T_LW);
    const ExtrinsicSixDofErrors cw_e =
        ExtrinsicError(StateToSE3(s2.camera), sc.gt.T_CW);
    lw_pitch.Push(lw_e.rot_err_rad.y() * 180.0 / M_PI);
    lw_trans_mm.Push(lw_e.trans_err_m.norm() * 1e3);
    cw_pitch.Push(cw_e.rot_err_rad.y() * 180.0 / M_PI);
    cw_trans_mm.Push(cw_e.trans_err_m.norm() * 1e3);
    t_d_L.Push((s2.lidar.t_d - sc.gt.t_d_L_s) * 1e3);
    t_d_C.Push((s2.camera.t_d - sc.gt.t_d_C_s) * 1e3);
    const EigenReport fim = Stage2ExtrinsicFim(
        sc_s1, *s1.trajectory, s2.lidar, s2.camera, levers, noise);
    if (fim.is_pd) {
      stage2_cond.Push(std::log10(fim.cond));
    }
  }

  std::cout << "  converged=" << converged << "/" << kNumSeeds << "\n";
  PrintBiasStd("pre-polish |LW| err", init_lw_mm, "mm", 3);
  PrintBiasStd("pre-polish |CW| err", init_cw_mm, "mm", 3);
  PrintBiasStd("LW pitch err", lw_pitch, "deg", 4);
  PrintBiasStd("|LW trans| err", lw_trans_mm, "mm", 3);
  PrintBiasStd("CW pitch err", cw_pitch, "deg", 4);
  PrintBiasStd("|CW trans| err", cw_trans_mm, "mm", 3);
  PrintBiasStd("t_d_L err", t_d_L, "ms", 3);
  PrintBiasStd("t_d_C err", t_d_C, "ms", 3);
  if (stage2_cond.n > 0) {
    std::cout << "  Stage-2 cond (log10) mean=" << stage2_cond.mean << "\n";
  }

  const bool cm_level = lw_trans_mm.mean < 50.0 && cw_trans_mm.mean < 50.0 &&
                        std::abs(lw_pitch.mean) < 1.0 && converged >= 45;
  std::cout << "\n=== GATE 3 — closed-form two-stage ===\n";
  std::cout << (cm_level ? "  → cm-level extrinsics from geometric init\n"
                         : "  → did NOT reach cm-level\n");
  SUCCEED();
}

TEST(TwoStageClosedFormInit, GateUqNoiseDecompositionMc) {
  const std::string config_dir = ConfigDirFromExperiments();
  const auto levers =
      clic_calib::LeverArmConfig::from_yaml(config_dir + "/lever_arms.yaml");
  const auto spline_cfg = LoadSplineConfig(config_dir + "/spline.yaml");
  const CoarseExtrinsicInit t_d_prior =
      LoadCoarseExtrinsicsFromYaml(config_dir);
  const RealisticNoiseSpec noise =
      clic_calib::NoiseModel::FromConfigDir(config_dir);

  const int num_seeds = []() {
    if (const char* env = std::getenv("CLIC_UQ_N")) {
      return std::max(1, std::atoi(env));
    }
    return 100;
  }();

  std::cout << "\n=== GATE UQ — Noise-source decomposition MC (N=" << num_seeds
            << ") ===\n";
  std::cout << "  TOTAL:     seed_traj=seed_obs=i\n";
  std::cout << "  FIXED:     traj@rep=" << kRepSeed
            << ", vary Stage-2 obs noise only\n";
  std::cout << "  TRAJ-PROP: seed_traj=i, seed_obs=rep=" << kRepSeed << "\n";
  std::cout << "  Additive band: Var_sum/Var_total ∈ [" << kUqAdditiveLo << ", "
            << kUqAdditiveHi << "]\n\n";

  const UqDecompositionMcResult uq = CollectUqDecompositionMc(
      levers, spline_cfg, t_d_prior, noise, num_seeds);
  ASSERT_TRUE(uq.rep_traj_ok);
  EXPECT_GE(uq.total.converged, num_seeds - 5);
  EXPECT_GE(uq.fixed_traj.converged, num_seeds - 5);
  EXPECT_GE(uq.traj_prop.converged, num_seeds - 5);

  const int in_band = PrintUqVarianceDecompositionTable(uq);
  PrintMcBiasTable(uq.total);
  std::cout << "\n  Additive in-band: " << in_band << "/14 DoF\n";

  if (in_band >= 10) {
    std::cout << "  → CONFIRMED: Σ_total ≈ Σ_fixed + Σ_traj (empirical UQ validated)\n";
  } else if (in_band >= 6) {
    std::cout << "  → PARTIAL: per-DoF decomposition mixed; inspect failed DoFs\n";
  } else {
    std::cout << "  → FAIL: systematic additive mismatch — check arm isolation\n";
  }

  EXPECT_GE(in_band, 10)
      << "Need ≥10/14 DoF in additive band for §2 UQ validation";
}

ExtrinsicState ExtrinsicStateFromGt(const SyntheticScenarioBundle& sc,
                                    bool lidar) {
  ExtrinsicState s;
  const SE3d T = lidar ? sc.gt.T_LW : sc.gt.T_CW;
  s.q = T.so3().unit_quaternion();
  s.t = T.translation();
  s.t_d = lidar ? sc.gt.t_d_L_s : sc.gt.t_d_C_s;
  return s;
}

Stage2Result GtStage2Linearization(const SyntheticScenarioBundle& sc) {
  Stage2Result s2;
  s2.lidar = ExtrinsicStateFromGt(sc, true);
  s2.camera = ExtrinsicStateFromGt(sc, false);
  s2.converged = true;
  return s2;
}

int CountTranslationInBandFixed(const FimMcSampleStats& mc,
                              const Eigen::VectorXd& theo) {
  int n = 0;
  auto ok = [&](const RunningStats& s, int idx) {
    const double th = idx < theo.size() ? theo(idx) : 0.0;
    if (th <= 1e-18) {
      return false;
    }
    const double r = s.Std() / th;
    return r >= kRatioLo && r <= kRatioHi;
  };
  if (ok(mc.lw_tx, 4)) ++n;
  if (ok(mc.lw_ty, 5)) ++n;
  if (ok(mc.lw_tz, 6)) ++n;
  if (ok(mc.cw_tx, 11)) ++n;
  if (ok(mc.cw_ty, 12)) ++n;
  return n;
}

void RunGate1FixedTrajBlock(
    const BodyTrajectory& fixed_traj, const char* traj_label,
    const Stage2Result* fim_lin,
    const clic_calib::LeverArmConfig& levers, const SplineConfig& spline_cfg,
    const CoarseExtrinsicInit& t_d_prior, const RealisticNoiseSpec& noise,
    int num_seeds) {
  std::cout << "\n--- " << traj_label << " ---\n";
  const FixedTrajMcFimResult result = CollectFixedTrajMcVsFim(
      fixed_traj, traj_label, levers, spline_cfg, t_d_prior, noise, num_seeds,
      fim_lin);
  PrintFixedMcStdTable(result.mc);
  const int in_band = PrintFimMcRatioTable(result.mc, result.theo_std);
  const int trans_in = CountTranslationInBandFixed(result.mc, result.theo_std);
  std::cout << "  → " << in_band << "/14 in band (" << trans_in
            << "/5 translations)\n";
}

TEST(TwoStageClosedFormInit, Gate1FixedTrajMcVsFixedFim) {
  const std::string config_dir = ConfigDirFromExperiments();
  const auto levers =
      clic_calib::LeverArmConfig::from_yaml(config_dir + "/lever_arms.yaml");
  const auto spline_cfg = LoadSplineConfig(config_dir + "/spline.yaml");
  const CoarseExtrinsicInit t_d_prior =
      LoadCoarseExtrinsicsFromYaml(config_dir);
  const RealisticNoiseSpec noise =
      clic_calib::NoiseModel::FromConfigDir(config_dir);

  const int num_seeds = []() {
    if (const char* env = std::getenv("CLIC_UQ_N")) {
      return std::max(100, std::atoi(env));
    }
    return 100;
  }();

  const SyntheticScenarioBundle rep_template =
      BuildNearFieldFimNoisyScenarioWithAttitude(kRepSeed, kRepSeed, noise);
  const Stage1Result s1_rep = FitStage1WithAttitude(
      rep_template, levers, spline_cfg, kStage1AlphaP, kStage1AlphaR);
  ASSERT_TRUE(s1_rep.summary.IsSolutionUsable());

  const Stage2Result gt_lin = GtStage2Linearization(rep_template);

  std::cout << "\n=== GATE 1 — Σ_fixed_MC vs fixed FIM⁻¹ (N=" << num_seeds
            << ") ===\n";
  std::cout << "  Vary: Stage-2 obs noise only (LiDAR σ_r, camera σ_pix)\n";
  std::cout << "  Pipeline: closed-form init + prior-free Stage-2 refine\n";
  std::cout << "  Band: emp_std/theo_std ∈ [" << kRatioLo << ", " << kRatioHi
            << "]\n";

  RunGate1FixedTrajBlock(rep_template.gt_traj, "A: GT traj, FIM @ rep converged",
                       nullptr, levers, spline_cfg, t_d_prior, noise, num_seeds);
  RunGate1FixedTrajBlock(rep_template.gt_traj, "B: GT traj, FIM @ GT extrinsics",
                       &gt_lin, levers, spline_cfg, t_d_prior, noise, num_seeds);
  RunGate1FixedTrajBlock(*s1_rep.trajectory,
                       "C: rep-fit Stage-1 traj, FIM @ rep converged", nullptr,
                       levers, spline_cfg, t_d_prior, noise, num_seeds);

  const FixedTrajMcFimResult primary = CollectFixedTrajMcVsFim(
      *s1_rep.trajectory, "C", levers, spline_cfg, t_d_prior, noise, num_seeds);
  EXPECT_GE(primary.mc.converged, num_seeds - 5);
  const int trans_in =
      CountTranslationInBandFixed(primary.mc, primary.theo_std);
  int in_band_all = 0;
  {
    struct Row {
      const RunningStats* s;
      int idx;
    };
    const Row rows[] = {
        {&primary.mc.t_d_L, 0},   {&primary.mc.lw_roll, 1},
        {&primary.mc.lw_pitch, 2}, {&primary.mc.lw_yaw, 3},
        {&primary.mc.lw_tx, 4},   {&primary.mc.lw_ty, 5},
        {&primary.mc.lw_tz, 6},   {&primary.mc.t_d_C, 7},
        {&primary.mc.cw_roll, 8}, {&primary.mc.cw_pitch, 9},
        {&primary.mc.cw_yaw, 10},   {&primary.mc.cw_tx, 11},
        {&primary.mc.cw_ty, 12},  {&primary.mc.cw_tz, 13},
    };
    for (const Row& r : rows) {
      const double th =
          r.idx < primary.theo_std.size() ? primary.theo_std(r.idx) : 0.0;
      if (th > 1e-18) {
        const double ratio = r.s->Std() / th;
        if (ratio >= kRatioLo && ratio <= kRatioHi) {
          ++in_band_all;
        }
      }
    }
  }
  const int in_band_final = in_band_all;

  std::cout << "\n=== DECISION GATE 1 — fixed-trajectory UQ ===\n";
  std::cout << "  Primary comparison: **C (rep-fit traj)** matches Gate 5 FIM "
               "linearization.\n";
  std::cout << "  GT traj (A/B) is cleaner isolation but MC spread is lower "
               "(obs synthesized on GT path).\n";
  std::cout << "  C: Σ_fixed_MC vs F_ext⁻¹ = " << in_band_final << "/14 ("
            << trans_in << "/5 translations)\n";
  if (in_band_final >= 10 && trans_in >= 4) {
    std::cout << "  → CONFIRMED: fixed FIM correct for fixed-traj sub-problem;\n";
    std::cout << "    Gate-5 6/14 mismatch was missing trajectory-prop term.\n";
  } else if (in_band_final >= 6) {
    std::cout << "  → PARTIAL: inspect outlier DoFs.\n";
  } else {
    std::cout << "  → FAIL: check FIM assembly or linearization point.\n";
  }

  EXPECT_GE(in_band_final, 10);
  EXPECT_GE(trans_in, 4);
}

TEST(TwoStageClosedFormInit, Gate2TrajPropMc) {
  const std::string config_dir = ConfigDirFromExperiments();
  const auto levers =
      clic_calib::LeverArmConfig::from_yaml(config_dir + "/lever_arms.yaml");
  const auto spline_cfg = LoadSplineConfig(config_dir + "/spline.yaml");
  const CoarseExtrinsicInit t_d_prior =
      LoadCoarseExtrinsicsFromYaml(config_dir);
  const RealisticNoiseSpec noise =
      clic_calib::NoiseModel::FromConfigDir(config_dir);

  const int num_seeds = []() {
    if (const char* env = std::getenv("CLIC_UQ_N")) {
      return std::max(100, std::atoi(env));
    }
    return 100;
  }();

  std::cout << "\n=== GATE 2 — Σ_traj-prop_MC (N=" << num_seeds << ") ===\n";
  std::cout << "  Fixed Stage-2 obs: single realization @ seed_obs=" << kRepSeed
            << " (LiDAR + camera noise frozen)\n";
  std::cout << "  Vary: RTK position + anisotropic attitude (Stage-1 only)\n";
  std::cout << "  Pipeline: Stage-1 → closed-form init → prior-free refine\n\n";

  const FimMcSampleStats traj_mc =
      CollectTrajPropMc(levers, spline_cfg, t_d_prior, noise, num_seeds);
  EXPECT_GE(traj_mc.converged, num_seeds - 5);

  PrintMcStdTable(traj_mc, "Σ_traj-prop_MC");

  const TrajPropDominanceReport dom = ComputeTrajPropDominance(traj_mc);
  const double trans_rot_ratio =
      dom.rot_var_sum > 1e-24 ? dom.trans_var_sum / dom.rot_var_sum : 0.0;
  const double lw_trans_rot =
      dom.lw_rot_var > 1e-24 ? dom.lw_trans_var / dom.lw_rot_var : 0.0;

  std::cout << std::fixed << std::setprecision(2);
  std::cout << "\n  Variance dominance (traj-prop arm):\n";
  std::cout << "    Σ trans var / Σ rot var (6+6 extrinsic) = " << trans_rot_ratio
            << "×\n";
  std::cout << "    LW trans var / LW rot var = " << lw_trans_rot << "×\n";
  std::cout << "    LW_tx std = " << traj_mc.lw_tx.Std() * 1e3
            << " mm  LW_tz std = " << traj_mc.lw_tz.Std() * 1e3 << " mm\n";
  std::cout << "    LW_roll std = " << traj_mc.lw_roll.Std() * 180e3 / M_PI
            << " mrad  CW_roll std = " << traj_mc.cw_roll.Std() * 180e3 / M_PI
            << " mrad\n";

  std::cout << "\n=== DECISION GATE 2 — trajectory-propagation UQ ===\n";
  const bool trans_dominates = trans_rot_ratio >= 5.0 && lw_trans_rot >= 3.0 &&
                               traj_mc.lw_tz.Std() > traj_mc.lw_roll.Std();
  const bool gate5_pattern =
      traj_mc.lw_tx.Std() > 1.5e-3 && traj_mc.lw_tz.Std() > 5.0e-3;
  if (trans_dominates && gate5_pattern) {
    std::cout << "  → CONFIRMED: translation DoFs dominate traj-prop variance;\n";
    std::cout << "    qualitatively matches Gate-5 inflation on LW translations.\n";
  } else if (trans_rot_ratio >= 2.0) {
    std::cout << "  → PARTIAL: translation-heavy but below strict dominance "
                 "threshold.\n";
  } else {
    std::cout << "  → FAIL: traj-prop not translation-dominant — check arm "
                 "isolation.\n";
  }

  EXPECT_GE(traj_mc.converged, num_seeds - 5);
  EXPECT_GE(trans_rot_ratio, 5.0);
  EXPECT_GE(lw_trans_rot, 3.0);
  EXPECT_GT(traj_mc.lw_tz.Std(), traj_mc.lw_roll.Std());
}

TEST(TwoStageClosedFormInit, Gate3DecompositionValidation) {
  const std::string config_dir = ConfigDirFromExperiments();
  const auto levers =
      clic_calib::LeverArmConfig::from_yaml(config_dir + "/lever_arms.yaml");
  const auto spline_cfg = LoadSplineConfig(config_dir + "/spline.yaml");
  const CoarseExtrinsicInit t_d_prior =
      LoadCoarseExtrinsicsFromYaml(config_dir);
  const RealisticNoiseSpec noise =
      clic_calib::NoiseModel::FromConfigDir(config_dir);

  const int num_seeds = []() {
    if (const char* env = std::getenv("CLIC_UQ_N")) {
      return std::max(100, std::atoi(env));
    }
    return 100;
  }();

  std::cout << "\n=== STEP 3 / GATE 3 — Covariance decomposition (N="
            << num_seeds << ") ===\n";
  std::cout << "  Σ_total:     all noise (Gate-5 MC @ N=" << num_seeds << ")\n";
  std::cout << "  Σ_fixed:     rep-fit traj @ " << kRepSeed
            << ", Stage-2 obs noise only\n";
  std::cout << "  Σ_traj-prop: Stage-2 obs @ " << kRepSeed
            << ", Stage-1 RTK+attitude only\n";
  std::cout << "  Validation:  Var_sum/Var_total ∈ [" << kDecompLo << ", "
            << kDecompHi << "]\n\n";

  const UqDecompositionMcResult uq = CollectUqDecompositionMc(
      levers, spline_cfg, t_d_prior, noise, num_seeds);
  ASSERT_TRUE(uq.rep_traj_ok);
  EXPECT_GE(uq.total.converged, num_seeds - 5);
  EXPECT_GE(uq.fixed_traj.converged, num_seeds - 5);
  EXPECT_GE(uq.traj_prop.converged, num_seeds - 5);

  PrintMcBiasTable(uq.total);
  const DecompositionGateReport gate = PrintDecompositionValidationTable(
      uq, kDecompLo, kDecompHi);

  std::cout << "\n=== DECISION GATE 3 — §2 UQ VERDICT ===\n";
  std::cout << "  Additive closure: " << gate.in_band << "/14 DoF in band ("
            << gate.trans_in_band << "/5 translations)\n";
  if (!gate.failed_dofs.empty()) {
    std::cout << "  Out of band: ";
    for (size_t i = 0; i < gate.failed_dofs.size(); ++i) {
      std::cout << gate.failed_dofs[i]
                << (i + 1 < gate.failed_dofs.size() ? ", " : "");
    }
    std::cout << "\n";
  }

  if (gate.in_band >= 10) {
    std::cout << "  → CONFIRMED: extrinsic uncertainty decomposes into fixed-traj\n";
    std::cout << "    information + trajectory-propagation; empirical sum matches\n";
    std::cout << "    total covariance. Fixed FIM under-predicts translation variance\n";
    std::cout << "    because it omits the propagation term (quantified per DoF).\n";
    std::cout << "  → Proceed to STEP 4.\n";
  } else if (gate.in_band >= 6) {
    std::cout << "  → PARTIAL: inspect failed DoFs for shared-noise path or\n";
    std::cout << "    nonlinear propagation. STOP pending diagnosis.\n";
  } else {
    std::cout << "  → FAIL: decomposition does not close. STOP.\n";
  }

  EXPECT_GE(gate.in_band, 10)
      << "§2 UQ requires ≥10/14 DoF additive closure @ [" << kDecompLo << ", "
      << kDecompHi << "]";
  EXPECT_GE(gate.trans_in_band, 4)
      << "Translation DoFs should predominantly close";
}

TEST(TwoStageClosedFormInit, Gate4DeltaMethodCrossCheck) {
  const std::string config_dir = ConfigDirFromExperiments();
  const auto levers =
      clic_calib::LeverArmConfig::from_yaml(config_dir + "/lever_arms.yaml");
  const auto spline_cfg = LoadSplineConfig(config_dir + "/spline.yaml");
  const CoarseExtrinsicInit t_d_prior =
      LoadCoarseExtrinsicsFromYaml(config_dir);
  const RealisticNoiseSpec noise =
      clic_calib::NoiseModel::FromConfigDir(config_dir);

  const int num_seeds = []() {
    if (const char* env = std::getenv("CLIC_UQ_N")) {
      return std::max(100, std::atoi(env));
    }
    return 100;
  }();

  std::cout << "\n=== STEP 4 / GATE 4 — Delta-method analytic cross-check (N="
            << num_seeds << ") ===\n";
  std::cout << "  Σ_analytic = Σ_fixed_FIM⁻¹ + Σ_k J_k Σ_traj,k J_k^T\n";
  std::cout << "  Σ_traj,k: Stage-1 FIM⁻¹ block per knot (well-conditioned)\n";
  std::cout << "  J_k:      FD ∂θ_ext/∂(knot k) @ rep-fit trajectory\n";
  std::cout << "  Compare analytic_std / mc_std ∈ [" << kDecompLo << ", "
            << kDecompHi << "]\n\n";

  const UqDecompositionMcResult uq = CollectUqDecompositionMc(
      levers, spline_cfg, t_d_prior, noise, num_seeds);
  ASSERT_TRUE(uq.rep_traj_ok);
  EXPECT_GE(uq.total.converged, num_seeds - 5);

  const DeltaMethodReport dm = RunDeltaMethodCrossCheck(
      levers, spline_cfg, t_d_prior, noise, uq.total);

  std::cout << "  Stage-1 FIM @ rep-fit traj: rank=" << dm.stage1_fim.rank
            << " cond=" << dm.stage1_fim.cond << " pd="
            << (dm.stage1_fim.is_pd ? "yes" : "NO") << "\n";
  std::cout << "  Knot FD sensitivity: mid knot " << dm.mid_knot << " (1 of "
            << dm.num_knots << " spline storage knots)\n\n";

  const int in_band =
      CompareAnalyticMcRatio(dm, kDecompLo, kDecompHi);

  std::cout << "\n=== DECISION GATE 4 — Analytic cross-check ===\n";
  std::cout << "  analytic/MC ratio in band: " << in_band << "/14\n";
  if (in_band >= 10) {
    std::cout << "  → CONFIRMED: delta-method (Stage-1 FIM + FD Jacobian)\n";
    std::cout << "    matches MC total covariance; avoids singular joint F_θθ.\n";
  } else if (in_band >= 6) {
    std::cout << "  → PARTIAL: FD linearization or knot-sum approximation\n";
    std::cout << "    limits analytic match; MC decomposition (STEP 3) remains primary.\n";
  } else {
    std::cout << "  → Analytic cross-check weak; MC decomposition (STEP 3) is\n";
    std::cout << "    the authoritative §2 UQ result.\n";
  }
  std::cout << "  Primary result: STEP 3 MC decomposition (Gate 3).\n";

  EXPECT_GT(dm.knots_used, 0) << "Mid-knot FD must contribute to propagation";
  EXPECT_GT(dm.stage1_fim.rank, 0)
      << "Stage-1 FIM must have positive numerical rank";
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
