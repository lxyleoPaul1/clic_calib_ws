#include <clic_calib/estimator/stage1_trajectory_fitter.h>

#include <clic_calib/estimator/ceres_so3_scope.h>
#include <clic_calib/estimator/trajectory_support.h>
#include <clic_calib/factor/attitude_factor_pose_form.h>
#include <clic_calib/factor/rtk_position_factor.h>
#include <clic_calib/factor/rtk_position_prais_winsten_factor.h>
#include <clic_calib/factor/trajectory_smoothness_factor.h>
#include <clic_calib/utils/temporal_correlation.h>
#include <clic_calib/spline/spline_segment.h>

#include <algorithm>
#include <functional>
#include <set>
#include <stdexcept>

namespace clic_calib {
namespace {

using trajectory_support::GetActiveKnotPointers;
using trajectory_support::InitStage1TrajectoryFromAttitudeStream;
using trajectory_support::MinSpacingInSortedTimes;
using trajectory_support::ReseedKnotPositionsFromRtkLeverArm;
using trajectory_support::Stage1KnotDt;
using trajectory_support::TrimTrajectoryToObservedSupport;

SplineSegmentMeta<SplineOrder> TrajectoryMeta(const BodyTrajectory& traj) {
  return SplineSegmentMeta<SplineOrder>(traj.minTimeNs(), traj.getDtNs(),
                                        traj.numKnots());
}

void AddStage1RtkPraisWinstenFactors(
    ceres::Problem* problem,
    std::vector<std::unique_ptr<ceres::CostFunction>>* owned,
    const std::vector<RTKMeasurement>& rtk, double rho,
    const BodyTrajectory& traj, const Eigen::Vector3d& L_B_to_A,
    const SplineSegmentMeta<SplineOrder>& meta,
    const std::function<void(const std::array<double*, SplineOrder>&)>&
        register_so3) {
  std::vector<const RTKMeasurement*> fixed;
  fixed.reserve(rtk.size());
  for (const auto& m : rtk) {
    if (m.fix_status_ == RTKMeasurement::FixStatus::FIXED) {
      fixed.push_back(&m);
    }
  }
  if (fixed.empty()) {
    return;
  }
  std::sort(fixed.begin(), fixed.end(),
            [](const RTKMeasurement* a, const RTKMeasurement* b) {
              return a->t_world_ < b->t_world_;
            });

  for (size_t i = 0; i < fixed.size(); ++i) {
    const RTKMeasurement& m = *fixed[i];
    const int64_t t_ns = static_cast<int64_t>(m.t_world_ * S_TO_NS);
    std::array<double*, SplineOrder> rot_knots{};
    std::array<double*, SplineOrder> pos_knots{};
    if (!GetActiveKnotPointers(traj, t_ns, &rot_knots, &pos_knots)) {
      continue;
    }
    if (i == 0) {
      owned->push_back(
          std::make_unique<analytic_derivative::RTKPositionPraisWinstenFirstFactor>(
              t_ns, m.p_A_W_observed_, m.covariance_, L_B_to_A, rho, meta));
      std::vector<double*> blocks;
      blocks.insert(blocks.end(), rot_knots.begin(), rot_knots.end());
      blocks.insert(blocks.end(), pos_knots.begin(), pos_knots.end());
      problem->AddResidualBlock(owned->back().get(), nullptr, blocks);
      register_so3(rot_knots);
      continue;
    }

    const RTKMeasurement& m_prev = *fixed[i - 1];
    const int64_t t_prev_ns =
        static_cast<int64_t>(m_prev.t_world_ * S_TO_NS);
    std::array<double*, SplineOrder> rot_prev{};
    std::array<double*, SplineOrder> pos_prev{};
    if (!GetActiveKnotPointers(traj, t_prev_ns, &rot_prev, &pos_prev)) {
      continue;
    }
    const analytic_derivative::RtkSplineKnotUnion knot_union =
        analytic_derivative::BuildRtkSplineKnotUnion(rot_prev, pos_prev,
                                                     rot_knots, pos_knots);
    owned->push_back(
        std::make_unique<
            analytic_derivative::RTKPositionPraisWinstenInnovationFactor>(
            t_prev_ns, t_ns, m_prev.p_A_W_observed_, m.p_A_W_observed_,
            m.covariance_, L_B_to_A, rho, meta, knot_union));
    std::vector<double*> blocks;
    blocks.insert(blocks.end(), knot_union.rot_blocks.begin(),
                 knot_union.rot_blocks.end());
    blocks.insert(blocks.end(), knot_union.pos_blocks.begin(),
                 knot_union.pos_blocks.end());
    problem->AddResidualBlock(owned->back().get(), nullptr, blocks);
    register_so3(rot_prev);
    register_so3(rot_knots);
  }
}

void AddStage1AttitudeFactors(
    ceres::Problem* problem,
    std::vector<std::unique_ptr<ceres::CostFunction>>* owned,
    const std::vector<AttitudeObservation>& attitude,
    const BodyTrajectory& traj,
    const SplineSegmentMeta<SplineOrder>& meta,
    const std::function<void(const std::array<double*, SplineOrder>&)>&
        register_so3,
    int stride) {
  for (size_t idx = 0; idx < attitude.size();
       idx += static_cast<size_t>(stride)) {
    const auto& obs = attitude[idx];
    const int64_t t_ns = static_cast<int64_t>(obs.t_world_ * S_TO_NS);
    std::array<double*, SplineOrder> rot_knots{};
    std::array<double*, SplineOrder> pos_knots{};
    if (!GetActiveKnotPointers(traj, t_ns, &rot_knots, &pos_knots)) {
      continue;
    }
    owned->push_back(
        std::make_unique<analytic_derivative::AttitudeFactorPoseForm>(
            t_ns, obs.R_WB_observed_, obs.covariance_, meta));
    std::vector<double*> blocks(rot_knots.begin(), rot_knots.end());
    problem->AddResidualBlock(owned->back().get(), nullptr, blocks);
    register_so3(rot_knots);
  }
}

}  // namespace

Stage1TrajectoryInput Stage1TrajectoryInput::FromRtkAttitudeStreams(
    std::vector<RTKMeasurement> rtk,
    std::vector<AttitudeObservation> attitude) {
  if (rtk.empty()) {
    throw std::runtime_error(
        "Stage1TrajectoryInput::FromRtkAttitudeStreams: no RTK");
  }
  std::sort(rtk.begin(), rtk.end(),
            [](const RTKMeasurement& a, const RTKMeasurement& b) {
              return a.t_world_ < b.t_world_;
            });
  std::sort(attitude.begin(), attitude.end(),
            [](const AttitudeObservation& a, const AttitudeObservation& b) {
              return a.t_world_ < b.t_world_;
            });
  Stage1TrajectoryInput out;
  out.rtk = std::move(rtk);
  out.attitude = std::move(attitude);
  out.t_obs_lo = out.rtk.front().t_world_;
  out.t_obs_hi = out.rtk.back().t_world_;
  return out;
}

Stage1TrajectoryInput Stage1TrajectoryInput::FromObservationStreams(
    const std::vector<RTKMeasurement>& rtk,
    const std::vector<AttitudeObservation>& attitude,
    const std::vector<double>& extra_bar_times_s) {
  if (rtk.empty()) {
    throw std::runtime_error(
        "Stage1TrajectoryInput::FromObservationStreams: no RTK");
  }
  Stage1TrajectoryInput out = FromRtkAttitudeStreams(rtk, attitude);
  double t_lo = out.t_obs_lo;
  double t_hi = out.t_obs_hi;
  for (const auto& a : attitude) {
    t_lo = std::min(t_lo, a.t_world_);
    t_hi = std::max(t_hi, a.t_world_);
  }
  for (double t : extra_bar_times_s) {
    t_lo = std::min(t_lo, t);
    t_hi = std::max(t_hi, t);
  }
  out.t_obs_lo = t_lo;
  out.t_obs_hi = t_hi;
  return out;
}

Stage1TrajectoryResult Stage1TrajectoryFitter::Fit(
    const Stage1TrajectoryInput& input, const LeverArmConfig& levers,
    const Stage1TrajectoryConfig& cfg) {
  Stage1TrajectoryResult out;
  if (input.rtk.empty()) {
    throw std::runtime_error("Stage1TrajectoryFitter::Fit: no RTK");
  }
  if (input.attitude.empty()) {
    throw std::runtime_error("Stage1TrajectoryFitter::Fit: no attitude stream");
  }
  if (input.t_obs_hi <= input.t_obs_lo) {
    throw std::runtime_error("Stage1TrajectoryFitter::Fit: invalid time window");
  }

  const double knot_dt =
      Stage1KnotDt(input.rtk, input.attitude, cfg.knot_interval_s);
  auto traj = InitStage1TrajectoryFromAttitudeStream(
      input.t_obs_lo, input.t_obs_hi, knot_dt, input.rtk, levers.L_B_to_A,
      input.attitude);

  struct Stage1BuildOpts {
    bool include_rtk = true;
    bool include_attitude = true;
    bool include_smoothness = true;
    bool rotation_constant = false;
  };

  double rtk_rho = cfg.rtk_ar1_rho;
  bool use_pw = cfg.use_rtk_prais_winsten;
  if (use_pw) {
    std::vector<double> rtk_times;
    rtk_times.reserve(input.rtk.size());
    for (const auto& m : input.rtk) {
      if (m.fix_status_ == RTKMeasurement::FixStatus::FIXED) {
        rtk_times.push_back(m.t_world_);
      }
    }
    const double min_rtk_dt = MinSpacingInSortedTimes(rtk_times);
    if (min_rtk_dt >= 0.2) {
      use_pw = false;
    }
  }

  auto build_and_solve = [&](int max_iters, const Stage1BuildOpts& opts,
                             ceres::Solver::Summary* summary,
                             double pw_rho) {
    CeresSo3ProblemScope scope;
    ceres::Problem& problem = *scope.problem;
    std::set<double*> so3_registered;
    std::set<double*> rot_blocks_all;
    auto register_so3_knots =
        [&](const std::array<double*, SplineOrder>& rot_knots) {
          for (double* q : rot_knots) {
            rot_blocks_all.insert(q);
            if (so3_registered.insert(q).second) {
              scope.SetLocalParamSO3(q);
            }
          }
        };

    const SplineSegmentMeta<SplineOrder> meta = TrajectoryMeta(*traj);

    if (opts.include_rtk) {
      if (use_pw) {
        AddStage1RtkPraisWinstenFactors(
            &problem, &scope.owned_costs, input.rtk, pw_rho, *traj,
            levers.L_B_to_A, meta, register_so3_knots);
      } else {
        for (const auto& m : input.rtk) {
          if (m.fix_status_ != RTKMeasurement::FixStatus::FIXED) {
            continue;
          }
          const int64_t t_ns = static_cast<int64_t>(m.t_world_ * S_TO_NS);
          std::array<double*, SplineOrder> rot_knots{};
          std::array<double*, SplineOrder> pos_knots{};
          if (!GetActiveKnotPointers(*traj, t_ns, &rot_knots, &pos_knots)) {
            continue;
          }
          scope.owned_costs.push_back(
              std::make_unique<analytic_derivative::RTKPositionFactor>(
                  t_ns, m.p_A_W_observed_, m.covariance_, levers.L_B_to_A,
                  meta));
          std::vector<double*> blocks;
          blocks.insert(blocks.end(), rot_knots.begin(), rot_knots.end());
          blocks.insert(blocks.end(), pos_knots.begin(), pos_knots.end());
          problem.AddResidualBlock(scope.owned_costs.back().get(), nullptr,
                                   blocks);
          register_so3_knots(rot_knots);
        }
      }
    }

    if (opts.include_attitude) {
      AddStage1AttitudeFactors(&problem, &scope.owned_costs, input.attitude,
                               *traj, meta, register_so3_knots,
                               cfg.attitude_stride);
    }

    if (opts.include_smoothness && (cfg.alpha_p > 0.0 || cfg.alpha_R > 0.0)) {
      const double dt_s = traj->getDt();
      for (size_t seg = 0; seg + SplineOrder <= traj->numKnots(); ++seg) {
        const int64_t t_mid_ns =
            meta.t0_ns + static_cast<int64_t>((static_cast<double>(seg) + 0.5) *
                                              meta.dt_ns);
        if (t_mid_ns < meta.MinTimeNs() || t_mid_ns >= meta.MaxTimeNs()) {
          continue;
        }
        std::array<double*, SplineOrder> rot_knots{};
        std::array<double*, SplineOrder> pos_knots{};
        if (!GetActiveKnotPointers(*traj, t_mid_ns, &rot_knots, &pos_knots)) {
          continue;
        }
        scope.owned_costs.push_back(
            std::make_unique<analytic_derivative::TrajectorySmoothnessFactor>(
                t_mid_ns, cfg.alpha_p, cfg.alpha_R, dt_s, meta));
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
    opts_solver.num_threads = 1;
    ceres::Solve(opts_solver, &problem, summary);
  };

  ceres::Solver::Summary attitude_pass;
  build_and_solve(120, Stage1BuildOpts{false, true, true, false}, &attitude_pass,
                  0.0);
  ReseedKnotPositionsFromRtkLeverArm(traj.get(), input.rtk, levers.L_B_to_A);

  if (use_pw && rtk_rho < 0.0) {
    rtk_rho = EstimateRtkResidualAr1Rho(*traj, input.rtk, levers.L_B_to_A);
  }
  rtk_rho = std::clamp(rtk_rho, 0.0, 0.995);

  ceres::Solver::Summary rtk_pass;
  build_and_solve(120, Stage1BuildOpts{true, false, true, true}, &rtk_pass,
                  rtk_rho);

  if (cfg.trim_to_observation_support) {
    out.trajectory = TrimTrajectoryToObservedSupport(*traj, input.t_obs_lo,
                                                     input.t_obs_hi, knot_dt);
  } else {
    out.trajectory = traj;
  }
  out.summary = rtk_pass;
  return out;
}

}  // namespace clic_calib
