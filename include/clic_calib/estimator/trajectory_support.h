#pragma once

#include <clic_calib/sensor_data/attitude_observation.h>
#include <clic_calib/sensor_data/rtk_measurement.h>
#include <clic_calib/spline/trajectory.h>
#include <clic_calib/utils/sophus_utils.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

namespace clic_calib {
namespace trajectory_support {

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

inline std::shared_ptr<BodyTrajectory> InitStage1TrajectoryFromAttitudeStream(
    double t_obs_lo, double t_obs_hi, double knot_dt,
    const std::vector<RTKMeasurement>& rtk, const Eigen::Vector3d& L_B_to_A,
    const std::vector<AttitudeObservation>& attitude) {
  const double margin = static_cast<double>(SplineOrder - 1) * knot_dt;
  const double t_lo = std::max(0.0, t_obs_lo - margin);
  const double t_hi = t_obs_hi + margin;
  if (t_hi <= t_lo + knot_dt) {
    throw std::runtime_error(
        "InitStage1TrajectoryFromAttitudeStream: invalid span");
  }
  const int num_knots =
      static_cast<int>(std::ceil((t_hi - t_lo) / knot_dt)) + (SplineOrder - 1);
  auto traj = std::make_shared<BodyTrajectory>(knot_dt, t_lo);
  const SE3d k0(SO3d(Eigen::Quaterniond::Identity()), Eigen::Vector3d::Zero());
  traj->setKnots(k0, num_knots);
  for (int i = 0; i < num_knots; ++i) {
    const double t_k = t_lo + static_cast<double>(i) * knot_dt;
    const Eigen::Vector3d p_A = InterpolateRtkPosition(rtk, t_k);
    const SO3d R_init = InterpolateAttitudeObservation(attitude, t_k);
    traj->setKnot(SE3d(R_init, p_A - R_init * L_B_to_A), i);
  }
  return traj;
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

  const double ref_dt = reference.getDt();
  if (std::abs(ref_dt - knot_dt) > 1e-9) {
    throw std::runtime_error("TrimTrajectoryToObservedSupport: knot_dt mismatch");
  }

  const int i_lo = static_cast<int>(std::floor((t_lo - ref_t0) / ref_dt + 1e-9));
  const int i_hi = static_cast<int>(std::ceil((t_hi - ref_t0) / ref_dt)) +
                   (SplineOrder - 1);
  const int ref_n = static_cast<int>(reference.numKnots());
  const int start = std::max(0, i_lo);
  const int end = std::min(ref_n, i_hi);
  if (end - start < static_cast<int>(SplineOrder)) {
    throw std::runtime_error(
        "TrimTrajectoryToObservedSupport: insufficient knots");
  }

  const double new_t0 = ref_t0 + static_cast<double>(start) * ref_dt;
  auto traj = std::make_shared<BodyTrajectory>(ref_dt, new_t0);
  const int num_knots = end - start;
  traj->setKnots(
      SE3d(reference.getKnotSO3(start), reference.getKnotPos(start)), num_knots);
  for (int i = 0; i < num_knots; ++i) {
    traj->setKnot(
        SE3d(reference.getKnotSO3(start + i), reference.getKnotPos(start + i)),
        i);
  }
  return traj;
}

}  // namespace trajectory_support
}  // namespace clic_calib
