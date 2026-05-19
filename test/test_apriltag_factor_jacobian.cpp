#include <clic_calib/factor/apriltag_reproj_factor.h>
#include <clic_calib/spline/spline_segment.h>
#include <clic_calib/utils/camera_projection.h>
#include <clic_calib/utils/sophus_utils.hpp>

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

TEST(AprilTagReprojFactor, ZeroResidualAtSyntheticProjection) {
  clic_calib::SplineSegmentMeta<clic_calib::SplineOrder> meta(kT0Ns, kDtNs,
                                                              kNumKnots);

  std::vector<double> rot(kNumKnots * 4, 0);
  std::vector<double> pos(kNumKnots * 3, 0);
  for (int k = 0; k < kNumKnots; ++k) {
    rot[k * 4 + 3] = 1.0;
    pos[k * 3 + 0] = 0.0;
    pos[k * 3 + 1] = 0.0;
    pos[k * 3 + 2] = 2.0;
  }

  const int64_t bar_t_ns = kT0Ns + static_cast<int64_t>(0.4 * kDtNs);
  double t_d = 0.0;
  Eigen::Quaterniond q_CW = Eigen::Quaterniond::Identity();
  Eigen::Vector3d t_CW(0.0, 0.0, 0.0);
  const Eigen::Vector3d L_B_to_G_M(0.1, 0.0, -0.55);

  std::vector<double*> params = {&t_d, q_CW.coeffs().data(), t_CW.data()};
  for (int k = 0; k < 4; ++k) {
    params.push_back(&rot[k * 4]);
  }
  for (int k = 0; k < 4; ++k) {
    params.push_back(&pos[k * 3]);
  }

  const clic_calib::SO3d R_WB =
      clic_calib::analytic_derivative::So3SplineView::EvaluateRotation(
          bar_t_ns, meta, params.data() + 3);
  const Eigen::Vector3d p_WB =
      clic_calib::analytic_derivative::RdSplineView::evaluate(
          bar_t_ns, meta, params.data() + 7);
  const clic_calib::SE3d T_CW(q_CW, t_CW);
  const Eigen::Vector3d p_M_C = T_CW * (p_WB + R_WB * L_B_to_G_M);

  clic_calib::PinholeIntrinsics K{500.0, 500.0, 320.0, 240.0};
  clic_calib::RadtanDistortion dist;
  const Eigen::Vector2d u_obs =
      clic_calib::ProjectRadtan(p_M_C, K, dist, nullptr);

  clic_calib::analytic_derivative::AprilTagReprojFactor factor(
      bar_t_ns, u_obs, L_B_to_G_M, K, dist, 1.0, meta);
  double residuals[2] = {0.0, 0.0};
  factor.Evaluate(params.data(), residuals, nullptr);
  EXPECT_NEAR(residuals[0], 0.0, 1e-6);
  EXPECT_NEAR(residuals[1], 0.0, 1e-6);
}

TEST(AprilTagReprojFactor, JacobianMatchesNumericOver20Trials) {
  std::mt19937 rng(11);
  std::uniform_real_distribution<double> unif(-1.5, 1.5);

  clic_calib::SplineSegmentMeta<clic_calib::SplineOrder> meta(kT0Ns, kDtNs,
                                                              kNumKnots);
  std::vector<double> rot(kNumKnots * 4);
  std::vector<double> pos(kNumKnots * 3);
  clic_calib::PinholeIntrinsics K{600.0, 600.0, 320.0, 240.0};
  clic_calib::RadtanDistortion dist{0.01, -0.002, 0.0, 0.0};

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
    Eigen::Quaterniond q_CW = RandomQuat(&rng);
    Eigen::Vector3d t_CW(unif(rng), unif(rng), unif(rng));
    std::vector<double*> params = {&t_d, q_CW.coeffs().data(), t_CW.data()};
    for (int k = 0; k < 4; ++k) {
      params.push_back(&rot[k * 4]);
    }
    for (int k = 0; k < 4; ++k) {
      params.push_back(&pos[k * 3]);
    }

    const int64_t bar_t_ns = kT0Ns + static_cast<int64_t>(0.4 * kDtNs);
    const Eigen::Vector2d u_obs(unif(rng) * 100 + 320, unif(rng) * 100 + 240);

    clic_calib::analytic_derivative::AprilTagReprojFactor factor(
        bar_t_ns, u_obs, Eigen::Vector3d(0.1, 0, -0.55), K, dist, 1.0, meta);

    std::vector<int> block_sizes = {1, 4, 3, 4, 4, 4, 4, 3, 3, 3, 3};
    EXPECT_TRUE(clic_calib::test::CompareJacobians(&factor, params, block_sizes,
                                                   1e-5))
        << "trial " << trial;
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
