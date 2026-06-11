#include <clic_calib/factor/body_centroid_factor.h>
#include <clic_calib/spline/spline_segment.h>
#include <clic_calib/utils/sophus_utils.hpp>
#include "gtest_ceres_guard.hpp"


#include <gtest/gtest.h>

#include <factor_test_utils.h>

#include <random>
#include <vector>

namespace {

constexpr int64_t kT0Ns = 0;
constexpr int64_t kDtNs = static_cast<int64_t>(0.1 * clic_calib::S_TO_NS);
constexpr int kNumKnots = 4;

Eigen::Quaterniond RandomQuat(std::mt19937* rng) {
  std::normal_distribution<double> nd(0.0, 1.0);
  Eigen::Vector4d v(nd(*rng), nd(*rng), nd(*rng), nd(*rng));
  v.normalize();
  return Eigen::Quaterniond(v[3], v[0], v[1], v[2]);
}

}  // namespace

TEST(BodyCentroidFactor, JacobianMatchesNumericOver20Trials) {
  std::mt19937 rng(11);
  std::uniform_real_distribution<double> unif(-2.0, 2.0);
  const Eigen::Vector3d L_B_to_body(0.0, 0.0, -0.12);
  const Eigen::Matrix3d sqrt_info = Eigen::Matrix3d::Identity() * 10.0;

  clic_calib::SplineSegmentMeta<clic_calib::SplineOrder> meta(kT0Ns, kDtNs,
                                                              kNumKnots);
  std::vector<double> rot(kNumKnots * 4);
  std::vector<double> pos(kNumKnots * 3);

  for (int trial = 0; trial < 20; ++trial) {
    for (int k = 0; k < kNumKnots; ++k) {
      const auto q = RandomQuat(&rng);
      rot[k * 4 + 0] = q.x();
      rot[k * 4 + 1] = q.y();
      rot[k * 4 + 2] = q.z();
      rot[k * 4 + 3] = q.w();
      for (int d = 0; d < 3; ++d) {
        pos[k * 3 + d] = unif(rng);
      }
    }

    double t_d = unif(rng) * 0.01;
    Eigen::Quaterniond q_LW = RandomQuat(&rng);
    Eigen::Vector3d t_LW(unif(rng), unif(rng), unif(rng));

    std::vector<double*> params = {&t_d, q_LW.coeffs().data(), t_LW.data()};
    for (int k = 0; k < 4; ++k) {
      params.push_back(&rot[k * 4]);
    }
    for (int k = 0; k < 4; ++k) {
      params.push_back(&pos[k * 3]);
    }

    const int64_t bar_t_ns = kT0Ns + static_cast<int64_t>(0.4 * kDtNs);
    const Eigen::Vector3d c_L(unif(rng), unif(rng), unif(rng));
    clic_calib::analytic_derivative::BodyCentroidFactor factor(
        bar_t_ns, c_L, L_B_to_body, sqrt_info, meta);

    const std::vector<int> block_sizes = {1, 4, 3, 4, 4, 4, 4, 3, 3, 3, 3};
    EXPECT_TRUE(clic_calib::test::CompareJacobians(&factor, params, block_sizes,
                                                   1e-5))
        << "trial " << trial;
  }
}

int main(int argc, char** argv) { return ClicGTestRunAll(argc, argv); }
