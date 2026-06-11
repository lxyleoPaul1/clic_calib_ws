#include <clic_calib/estimator/relative_extrinsic.h>

namespace clic_calib {

SE3d ComposeRelativeExtrinsic(const SE3d& T_iW, const SE3d& T_jW) {
  return T_jW * T_iW.inverse();
}

RelativeExtrinsicError RelativeExtrinsicErrorVsGt(const SE3d& T_rel_est,
                                                  const SE3d& T_rel_gt) {
  RelativeExtrinsicError e;
  const SE3d T_err = T_rel_gt.inverse() * T_rel_est;
  e.rot_deg = T_err.so3().log().norm() * 180.0 / M_PI;
  e.trans_mm = T_err.translation().norm() * 1e3;
  return e;
}

Eigen::Matrix3d SampleTranslationCovariance(
    const std::vector<Eigen::Vector3d>& samples_mm) {
  Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
  if (samples_mm.size() < 2) {
    return cov;
  }
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (const auto& s : samples_mm) {
    mean += s;
  }
  mean /= static_cast<double>(samples_mm.size());
  for (const auto& s : samples_mm) {
    const Eigen::Vector3d d = s - mean;
    cov += d * d.transpose();
  }
  cov /= static_cast<double>(samples_mm.size() - 1);
  return cov;
}

RelativeAbsCovarianceRatio CovarianceRelOverMeanAbs(
    const std::vector<Eigen::Vector3d>& err_a_mm,
    const std::vector<Eigen::Vector3d>& err_b_mm,
    const std::vector<Eigen::Vector3d>& err_rel_mm) {
  RelativeAbsCovarianceRatio r;
  const Eigen::Matrix3d cov_a = SampleTranslationCovariance(err_a_mm);
  const Eigen::Matrix3d cov_b = SampleTranslationCovariance(err_b_mm);
  const Eigen::Matrix3d cov_rel = SampleTranslationCovariance(err_rel_mm);
  r.trace_rel_mm2 = cov_rel.trace();
  r.mean_trace_abs_mm2 = 0.5 * (cov_a.trace() + cov_b.trace());
  if (r.mean_trace_abs_mm2 > 1e-12) {
    r.rel_over_abs = r.trace_rel_mm2 / r.mean_trace_abs_mm2;
  }
  return r;
}

double CenterRegistrationTransMm(const SE3d& T_rel_est, const SE3d& T_rel_gt) {
  return RelativeExtrinsicErrorVsGt(T_rel_est, T_rel_gt).trans_mm;
}

}  // namespace clic_calib
