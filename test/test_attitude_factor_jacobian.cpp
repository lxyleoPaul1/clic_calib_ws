#include <clic_calib/estimator/trajectory_support.h>
#include <clic_calib/factor/attitude_factor_pose_form.h>
#include <clic_calib/spline/spline_segment.h>
#include <clic_calib/utils/sophus_utils.hpp>
#include "gtest_ceres_guard.hpp"


#include <gtest/gtest.h>

#include "factor_test_utils.h"

#include <array>
#include <vector>

namespace {

using clic_calib::trajectory_support::GetActiveKnotPointers;

clic_calib::SplineSegmentMeta<clic_calib::SplineOrder> TrajectoryMeta(
    const clic_calib::BodyTrajectory& traj) {
  return clic_calib::SplineSegmentMeta<clic_calib::SplineOrder>(
      traj.minTimeNs(), traj.getDtNs(), traj.numKnots());
}

Eigen::Matrix3d AnisotropicBodyCovarianceRad2() {
  const double d2r = M_PI / 180.0;
  Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
  cov(0, 0) = std::pow(0.2 * d2r, 2);
  cov(1, 1) = std::pow(0.2 * d2r, 2);
  cov(2, 2) = std::pow(1.5 * d2r, 2);
  return cov;
}

}  // namespace

TEST(AttitudeFactorPoseForm, JacobianMatchesNumeric) {
  const double knot_dt = 0.05;
  const double t_query = 2.0;
  clic_calib::BodyTrajectory traj(knot_dt, 0.0);
  traj.setKnots(
      clic_calib::SE3d(clic_calib::SO3d::rotZ(0.3), Eigen::Vector3d::Zero()),
      80);

  const clic_calib::SO3d R_obs =
      clic_calib::SO3d::rotY(0.1) * clic_calib::SO3d::rotX(-0.05);
  const Eigen::Matrix3d cov = AnisotropicBodyCovarianceRad2();
  const int64_t t_ns = static_cast<int64_t>(t_query * clic_calib::S_TO_NS);

  const clic_calib::SplineSegmentMeta<clic_calib::SplineOrder> meta =
      TrajectoryMeta(traj);
  std::array<double*, clic_calib::SplineOrder> rot_knots{};
  std::array<double*, clic_calib::SplineOrder> pos_knots{};
  ASSERT_TRUE(GetActiveKnotPointers(traj, t_ns, &rot_knots, &pos_knots));

  clic_calib::analytic_derivative::AttitudeFactorPoseForm factor(
      t_ns, R_obs, cov, meta);
  std::vector<double*> params(rot_knots.begin(), rot_knots.end());
  std::vector<int> block_sizes(4, 4);

  EXPECT_TRUE(clic_calib::test::CompareJacobians(&factor, params, block_sizes,
                                                   1e-5, 1e-5));
}

int main(int argc, char** argv) { return ClicGTestRunAll(argc, argv); }
