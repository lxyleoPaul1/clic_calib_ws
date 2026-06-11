#pragma once

#include <clic_calib/utils/sophus_utils.hpp>

#include <Eigen/Core>

#include <vector>

namespace clic_calib {

/** T_{j\leftarrow i} = T_{jW} · T_{Wi} with world W. */
SE3d ComposeRelativeExtrinsic(const SE3d& T_iW, const SE3d& T_jW);

struct RelativeExtrinsicError {
  double rot_deg = 0.0;
  double trans_mm = 0.0;
};

RelativeExtrinsicError RelativeExtrinsicErrorVsGt(const SE3d& T_rel_est,
                                                  const SE3d& T_rel_gt);

/** Sample covariance trace ratio: rel / mean(abs per sensor). */
struct RelativeAbsCovarianceRatio {
  double rel_over_abs = 0.0;
  double trace_rel_mm2 = 0.0;
  double mean_trace_abs_mm2 = 0.0;
};

Eigen::Matrix3d SampleTranslationCovariance(
    const std::vector<Eigen::Vector3d>& samples_mm);

RelativeAbsCovarianceRatio CovarianceRelOverMeanAbs(
    const std::vector<Eigen::Vector3d>& err_a_mm,
    const std::vector<Eigen::Vector3d>& err_b_mm,
    const std::vector<Eigen::Vector3d>& err_rel_mm);

/** Center registration: rotation-only relative metric (no baseline leverage). */
double CenterRegistrationTransMm(const SE3d& T_rel_est, const SE3d& T_rel_gt);

}  // namespace clic_calib
