#pragma once

#include <clic_calib/sensor_data/rtk_measurement.h>
#include <clic_calib/spline/trajectory.h>

#include <Eigen/Core>

#include <vector>

namespace clic_calib {

/** AR(1) innovation or exponential-kernel decorrelation for evenly spaced streams. */
struct TemporalDecorrelationConfig {
  bool enabled = false;
  /** Lag-1 autocorrelation; < 0 ⇒ estimate from @p aspect_series. */
  double ar1_rho = -1.0;
  /** Exponential kernel τ [s] when mode = kExponentialKernel. */
  double kernel_tau_s = 0.35;
  enum class Mode { kAr1Innovation, kExponentialKernel };
  Mode mode = Mode::kAr1Innovation;
};

/** Lag-1 sample autocorrelation (zero-mean); clamped to (-0.995, 0.995). */
double EstimateLag1Autocorrelation(const std::vector<double>& series);

/**
 * Per-observation multipliers for sqrt_information (3×3 left-multiply scalar).
 * Normalized so Σ s_i² equals AR(1) effective sample count n·(1−ρ)/(1+ρ).
 */
std::vector<double> ComputeTemporalDecorrelationScales(
    const std::vector<double>& times_s,
    const std::vector<double>& aspect_series_rad,
    const TemporalDecorrelationConfig& cfg);

/** Uniform per-frame √( (1−ρ)/(1+ρ) ) from AR(1) effective sample count. */
std::vector<double> UniformAr1DecorrelationScales(size_t count, double rho);

/**
 * Estimate AR(1) ρ from consecutive RTK antenna residuals ‖p_obs−p_pred‖.
 */
double EstimateRtkResidualAr1Rho(
    const BodyTrajectory& traj, const std::vector<RTKMeasurement>& rtk,
    const Eigen::Vector3d& L_B_to_A);

}  // namespace clic_calib
