#include <clic_calib/factor/prior_factor.h>
#include <clic_calib/factor/trajectory_smoothness_factor.h>
#include <clic_calib/spline/spline_segment.h>
#include "gtest_ceres_guard.hpp"


#include <gtest/gtest.h>

#include <factor_test_utils.h>

#include <random>
#include <vector>

namespace {

constexpr int64_t kT0Ns = 0;
constexpr int64_t kDtNs = static_cast<int64_t>(0.1 * clic_calib::S_TO_NS);

}  // namespace

TEST(ExtrinsicPriorFactor, JacobianMatchesNumeric) {
  const clic_calib::SE3d T_prior(
      clic_calib::SO3d::rotZ(0.1),
      Eigen::Vector3d(1.0, 0.5, -0.2));
  const clic_calib::SE3d T_actual =
      T_prior * clic_calib::SE3d(clic_calib::SO3d::exp(Eigen::Vector3d(0.01, -0.02, 0.005)),
                                 Eigen::Vector3d(0.02, -0.01, 0.03));
  Eigen::Matrix<double, 6, 1> sqrt_info;
  sqrt_info << 1.0, 1.0, 1.0, 0.5, 0.5, 0.5;

  clic_calib::analytic_derivative::ExtrinsicPriorFactor factor(T_prior,
                                                             sqrt_info);

  Eigen::Quaterniond q_actual = T_actual.unit_quaternion();
  Eigen::Vector3d t_actual = T_actual.translation();
  std::vector<double> q_storage(q_actual.coeffs().data(),
                                q_actual.coeffs().data() + 4);
  std::vector<double> t_storage(t_actual.data(), t_actual.data() + 3);
  std::vector<double*> params = {q_storage.data(), t_storage.data()};

  std::vector<int> block_sizes = {4, 3};
  // Decoupled SE(3) log Jacobian is first-order; allow small absolute slack.
  EXPECT_TRUE(clic_calib::test::CompareJacobians(&factor, params, block_sizes,
                                                 1e-5, 0.05));
}

TEST(TrajectorySmoothnessFactor, JacobianMatchesNumeric) {
  clic_calib::SplineSegmentMeta<clic_calib::SplineOrder> meta(kT0Ns, kDtNs, 4);
  std::vector<double> rot(16, 0);
  std::vector<double> pos(12, 0);
  for (int k = 0; k < 4; ++k) {
    rot[k * 4 + 3] = 1.0;
    pos[k * 3 + 0] = k * 0.2;
  }

  std::vector<double*> params;
  for (int k = 0; k < 4; ++k) {
    params.push_back(&rot[k * 4]);
  }
  for (int k = 0; k < 4; ++k) {
    params.push_back(&pos[k * 3]);
  }

  const int64_t t_ns = kT0Ns + static_cast<int64_t>(0.4 * kDtNs);
  clic_calib::analytic_derivative::TrajectorySmoothnessFactor factor(
      t_ns, 0.01, 0.01, 0.1, meta);

  std::vector<int> block_sizes = {4, 4, 4, 4, 3, 3, 3, 3};
  EXPECT_TRUE(clic_calib::test::CompareJacobians(&factor, params, block_sizes,
                                                 1e-5));
}

int main(int argc, char** argv) { return ClicGTestRunAll(argc, argv); }
