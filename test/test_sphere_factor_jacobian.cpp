#include <clic_calib/factor/sphere_implicit_factor.h>
#include <clic_calib/spline/spline_segment.h>
#include <clic_calib/utils/sophus_utils.hpp>
#include "gtest_ceres_guard.hpp"


#include <gtest/gtest.h>

#include <factor_test_utils.h>

#include <cmath>
#include <random>
#include <vector>

namespace {

constexpr int64_t kT0Ns = 0;
constexpr int64_t kDtNs = static_cast<int64_t>(0.1 * clic_calib::S_TO_NS);
constexpr int kNumKnots = 4;

void InitMeta(clic_calib::SplineSegmentMeta<clic_calib::SplineOrder>* meta) {
  *meta = clic_calib::SplineSegmentMeta<clic_calib::SplineOrder>(kT0Ns, kDtNs,
                                                                  kNumKnots);
}

Eigen::Quaterniond RandomQuat(std::mt19937* rng) {
  std::normal_distribution<double> nd(0.0, 1.0);
  Eigen::Vector4d v(nd(*rng), nd(*rng), nd(*rng), nd(*rng));
  v.normalize();
  return Eigen::Quaterniond(v[3], v[0], v[1], v[2]);
}

}  // namespace

TEST(SphereImplicitFactor, ZeroResidualOnSurface) {
  const double R_ball = 0.10;
  const Eigen::Vector3d L_B_to_G(0.0, 0.0, -0.55);
  const clic_calib::SE3d T_LW;

  clic_calib::SplineSegmentMeta<clic_calib::SplineOrder> meta(kT0Ns, kDtNs,
                                                              kNumKnots);

  std::vector<double> rot(kNumKnots * 4, 0);
  std::vector<double> pos(kNumKnots * 3, 0);
  for (int k = 0; k < kNumKnots; ++k) {
    const auto q = Eigen::Quaterniond::Identity();
    rot[k * 4 + 0] = q.x();
    rot[k * 4 + 1] = q.y();
    rot[k * 4 + 2] = q.z();
    rot[k * 4 + 3] = q.w();
    pos[k * 3 + 0] = k * 0.1;
    pos[k * 3 + 1] = 0.0;
    pos[k * 3 + 2] = 3.0;
  }

  const int64_t bar_t_ns = kT0Ns + static_cast<int64_t>(0.4 * kDtNs);
  double t_d = 0.0;
  Eigen::Quaterniond q_LW = Eigen::Quaterniond::Identity();
  Eigen::Vector3d t_LW = Eigen::Vector3d::Zero();

  std::vector<double*> params = {&t_d, q_LW.coeffs().data(), t_LW.data()};
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
  const Eigen::Vector3d p_G_W = p_WB + R_WB * L_B_to_G;
  const Eigen::Vector3d p_G_L = T_LW * p_G_W;
  const Eigen::Vector3d q_L = p_G_L + R_ball * Eigen::Vector3d::UnitX();

  clic_calib::analytic_derivative::SphereImplicitFactor factor(
      bar_t_ns, q_L, L_B_to_G, R_ball, 0.02, meta);
  double residual = 0.0;
  factor.Evaluate(params.data(), &residual, nullptr);
  EXPECT_NEAR(residual, 0.0, 1e-6);
}

TEST(SphereImplicitFactor, FiveCmOffSurface) {
  const double R_ball = 0.10;
  const Eigen::Vector3d L_B_to_G(0.0, 0.0, -0.55);
  clic_calib::SplineSegmentMeta<clic_calib::SplineOrder> meta(kT0Ns, kDtNs,
                                                              kNumKnots);

  std::vector<double> rot(kNumKnots * 4, 0);
  std::vector<double> pos(kNumKnots * 3, 0);
  for (int k = 0; k < kNumKnots; ++k) {
    rot[k * 4 + 3] = 1.0;
    pos[k * 3 + 0] = 0.0;
    pos[k * 3 + 1] = 0.0;
    pos[k * 3 + 2] = 0.0;
  }

  const int64_t bar_t_ns = kT0Ns + static_cast<int64_t>(0.4 * kDtNs);
  double t_d = 0.0;
  Eigen::Quaterniond q_LW = Eigen::Quaterniond::Identity();
  Eigen::Vector3d t_LW = Eigen::Vector3d::Zero();
  std::vector<double*> params = {&t_d, q_LW.coeffs().data(), t_LW.data()};
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
  const Eigen::Vector3d p_G_L = p_WB + R_WB * L_B_to_G;
  const Eigen::Vector3d q_L = p_G_L + (R_ball + 0.05) * Eigen::Vector3d::UnitX();

  clic_calib::analytic_derivative::SphereImplicitFactor factor(
      bar_t_ns, q_L, L_B_to_G, R_ball, 0.02, meta);
  double residual = 0.0;
  factor.Evaluate(params.data(), &residual, nullptr);
  EXPECT_NEAR(residual, 0.05 / 0.02, 1e-4);
}

TEST(SphereImplicitFactor, JacobianMatchesNumericOver20Trials) {
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> unif(-2.0, 2.0);

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
    const Eigen::Vector3d q_L(unif(rng), unif(rng), unif(rng));
    clic_calib::analytic_derivative::SphereImplicitFactor factor(
        bar_t_ns, q_L, Eigen::Vector3d(0, 0, -0.55), 0.10, 0.02, meta);

    std::vector<int> block_sizes = {1, 4, 3, 4, 4, 4, 4, 3, 3, 3, 3};
    EXPECT_TRUE(clic_calib::test::CompareJacobians(&factor, params, block_sizes,
                                                   1e-5))
        << "trial " << trial;
  }
}

int main(int argc, char** argv) { return ClicGTestRunAll(argc, argv); }
