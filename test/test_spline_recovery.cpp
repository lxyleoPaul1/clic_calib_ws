#include <clic_calib/spline/trajectory.h>
#include "gtest_ceres_guard.hpp"


#include <gtest/gtest.h>

namespace {

TEST(BodyTrajectory, ExtendsAndEvaluatesFinitePose) {
  clic_calib::BodyTrajectory traj(0.1, 0.0);
  const int64_t t_ns = static_cast<int64_t>(0.5 * clic_calib::BodyTrajectory::kSToNs);
  traj.extendKnotsTo(
      t_ns, clic_calib::SO3d(Eigen::Quaterniond::Identity()),
      Eigen::Vector3d(1.0, 2.0, 3.0));

  const auto p = traj.position_wb(0.25);
  const auto T = traj.pose_wb(0.25);
  EXPECT_TRUE(p.allFinite());
  EXPECT_TRUE(T.translation().allFinite());

  const Eigen::Vector3d lever(0.0, 0.0, -0.55);
  const auto p_a = traj.antenna_position_w(0.25, lever);
  EXPECT_TRUE(p_a.allFinite());
}

}  // namespace

int main(int argc, char** argv) { return ClicGTestRunAll(argc, argv); }
