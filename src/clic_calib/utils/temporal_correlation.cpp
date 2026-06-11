#include <clic_calib/utils/temporal_correlation.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace clic_calib {
namespace {

double ClampRho(double rho) {
  return std::clamp(rho, -0.995, 0.995);
}

void NormalizeToEffectiveSampleCount(std::vector<double>* scales, double rho) {
  if (!scales || scales->empty()) {
    return;
  }
  const double n = static_cast<double>(scales->size());
  const double n_eff = n * (1.0 - rho) / (1.0 + rho);
  double sum_sq = 0.0;
  for (double s : *scales) {
    sum_sq += s * s;
  }
  if (sum_sq < 1e-18 || n_eff < 1e-6) {
    return;
  }
  const double norm = std::sqrt(n_eff / sum_sq);
  for (double& s : *scales) {
    s *= norm;
  }
}

}  // namespace

double EstimateLag1Autocorrelation(const std::vector<double>& series) {
  if (series.size() < 3) {
    return 0.0;
  }
  const double n = static_cast<double>(series.size());
  double mean = 0.0;
  for (double v : series) {
    mean += v;
  }
  mean /= n;
  double var = 0.0;
  double cov = 0.0;
  for (size_t i = 0; i + 1 < series.size(); ++i) {
    const double a = series[i] - mean;
    const double b = series[i + 1] - mean;
    var += a * a;
    cov += a * b;
  }
  var /= std::max(1.0, n - 1.0);
  if (var < 1e-18) {
    return 0.0;
  }
  return ClampRho(cov / ((n - 1.0) * var));
}

std::vector<double> ComputeTemporalDecorrelationScales(
    const std::vector<double>& times_s,
    const std::vector<double>& aspect_series_rad,
    const TemporalDecorrelationConfig& cfg) {
  const size_t n = times_s.size();
  std::vector<double> scales(n, 1.0);
  if (!cfg.enabled || n == 0) {
    return scales;
  }

  double rho = cfg.ar1_rho;
  if (rho < 0.0 && aspect_series_rad.size() == n) {
    rho = EstimateLag1Autocorrelation(aspect_series_rad);
  }
  rho = ClampRho(std::min(rho, 0.92));

  if (cfg.mode == TemporalDecorrelationConfig::Mode::kExponentialKernel &&
      cfg.kernel_tau_s > 1e-6) {
    const double tau = cfg.kernel_tau_s;
    for (size_t i = 0; i < n; ++i) {
      double kernel_sum = 0.0;
      for (size_t j = 0; j < n; ++j) {
        kernel_sum += std::exp(-std::abs(times_s[i] - times_s[j]) / tau);
      }
      scales[i] = 1.0 / std::sqrt(std::max(kernel_sum, 1e-6));
    }
    NormalizeToEffectiveSampleCount(&scales, rho);
    return scales;
  }

  const double innov = std::sqrt(std::max(0.0, 1.0 - rho * rho));
  scales[0] = 1.0;
  for (size_t i = 1; i < n; ++i) {
    scales[i] = innov;
  }
  NormalizeToEffectiveSampleCount(&scales, rho);
  return scales;
}

double EstimateRtkResidualAr1Rho(const BodyTrajectory& traj,
                                const std::vector<RTKMeasurement>& rtk,
                                const Eigen::Vector3d& L_B_to_A) {
  std::vector<const RTKMeasurement*> fixed;
  fixed.reserve(rtk.size());
  for (const auto& m : rtk) {
    if (m.fix_status_ == RTKMeasurement::FixStatus::FIXED) {
      fixed.push_back(&m);
    }
  }
  if (fixed.size() < 3) {
    return 0.0;
  }
  std::sort(fixed.begin(), fixed.end(),
            [](const RTKMeasurement* a, const RTKMeasurement* b) {
              return a->t_world_ < b->t_world_;
            });

  std::array<std::vector<double>, 3> axis_series;
  for (const RTKMeasurement* m : fixed) {
    const Eigen::Vector3d r =
        m->p_A_W_observed_ - traj.antenna_position_w(m->t_world_, L_B_to_A);
    for (int k = 0; k < 3; ++k) {
      axis_series[static_cast<size_t>(k)].push_back(r[k]);
    }
  }
  double rho_sum = 0.0;
  int rho_count = 0;
  for (const auto& series : axis_series) {
    const double rho = EstimateLag1Autocorrelation(series);
    if (std::isfinite(rho)) {
      rho_sum += rho;
      ++rho_count;
    }
  }
  return rho_count > 0 ? rho_sum / static_cast<double>(rho_count) : 0.0;
}

std::vector<double> UniformAr1DecorrelationScales(size_t count, double rho) {
  std::vector<double> scales(count, 1.0);
  if (count == 0) {
    return scales;
  }
  rho = ClampRho(rho);
  const double s = std::sqrt(std::max(0.0, (1.0 - rho) / (1.0 + rho)));
  for (double& v : scales) {
    v = s;
  }
  return scales;
}

}  // namespace clic_calib
