#include <clic_calib/factor/rtk_position_factor.h>
#include <clic_calib/spline/spline_segment.h>
#include <clic_calib/utils/sophus_utils.hpp>
#include "gtest_ceres_guard.hpp"


#include <gtest/gtest.h>

#include <factor_test_utils.h>

#include <random>
#include <vector>

namespace {

using clic_calib::SplineOrder;
using clic_calib::SO3d;
using clic_calib::analytic_derivative::RTKPositionFactor;

constexpr int64_t kT0Ns = 0;
constexpr int64_t kDtNs = static_cast<int64_t>(0.1 * clic_calib::S_TO_NS);
constexpr int kNumKnots = 4;

void InitSplineMeta(clic_calib::SplineSegmentMeta<SplineOrder>* meta) {
  *meta = clic_calib::SplineSegmentMeta<SplineOrder>(kT0Ns, kDtNs, kNumKnots);
}

void RandomUnitQuaternion(std::mt19937* rng, double q[4]) {
  std::normal_distribution<double> nd(0.0, 1.0);
  Eigen::Vector4d v(nd(*rng), nd(*rng), nd(*rng), nd(*rng));
  v.normalize();
  q[0] = v[0];
  q[1] = v[1];
  q[2] = v[2];
  q[3] = v[3];
}

}  // namespace

TEST(RTKPositionFactor, JacobianMatchesNumericOver20Trials) {
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> pos_dist(-5.0, 5.0);

  clic_calib::SplineSegmentMeta<SplineOrder> meta(kT0Ns, kDtNs, kNumKnots);

  std::vector<double> rot_storage(kNumKnots * 4);
  std::vector<double> pos_storage(kNumKnots * 3);
  std::vector<double*> params(SplineOrder * 2);

  for (int trial = 0; trial < 20; ++trial) {
    for (int k = 0; k < kNumKnots; ++k) {
      RandomUnitQuaternion(&rng, &rot_storage[k * 4]);
      for (int d = 0; d < 3; ++d) {
        pos_storage[k * 3 + d] = pos_dist(rng);
      }
    }

    const int64_t t_ns = kT0Ns + static_cast<int64_t>(0.37 * kDtNs);
    for (int k = 0; k < SplineOrder; ++k) {
      params[k] = &rot_storage[k * 4];
      params[SplineOrder + k] = &pos_storage[k * 3];
    }

    const Eigen::Vector3d L_B_to_A(0.12, 0.05, -0.58);
    const SO3d R =
        clic_calib::analytic_derivative::So3SplineView::EvaluateRotation(
            t_ns, meta, params.data());
    const Eigen::Vector3d p =
        clic_calib::analytic_derivative::RdSplineView::evaluate(t_ns, meta,
                                                                params.data() +
                                                                    SplineOrder);
    const Eigen::Vector3d p_obs = p + R * L_B_to_A + Eigen::Vector3d(0.1, -0.2, 0.05);

    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    cov.diagonal() << 0.02, 0.02, 0.04;

    RTKPositionFactor factor(t_ns, p_obs, cov, L_B_to_A, meta);

    std::vector<int> block_sizes = {4, 4, 4, 4, 3, 3, 3, 3};
    EXPECT_TRUE(clic_calib::test::CompareJacobians(&factor, params, block_sizes,
                                                   1e-5))
        << "trial " << trial;
  }
}

int main(int argc, char** argv) { return ClicGTestRunAll(argc, argv); }
