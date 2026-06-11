#include <clic_calib/factor/fixed_traj_body_centroid_joint_lever_factor.h>
#include <clic_calib/spline/trajectory.h>
#include <clic_calib/utils/noise_model.h>
#include <clic_calib/utils/sophus_utils.hpp>

#include <gtest/gtest.h>

#include <factor_test_utils.h>

#include <random>
#include <vector>

namespace {

constexpr double kRelTol = 1e-5;

}  // namespace

TEST(FixedTrajBodyCentroidJointLeverFactor, JacobianMatchesNumericOver20Trials) {
  clic_calib::BodyTrajectory traj(0.1, 0.0);
  traj.setKnots(clic_calib::SE3d(clic_calib::SO3d::rotY(0.1),
                                 Eigen::Vector3d(5.0, 0.0, 12.0)),
                  8);

  const clic_calib::NoiseModel noise;
  const Eigen::Matrix3d sqrt_info =
      noise.BodyCentroidSqrtInformation(20.0, 50);

  std::mt19937 rng(42);
  std::uniform_real_distribution<double> unif(-0.05, 0.05);

  for (int trial = 0; trial < 20; ++trial) {
    const double t_bar = 0.12 + 0.005 * trial;
    const double t_d = unif(rng) * 0.01;
    Eigen::Quaterniond q_LW = Eigen::Quaterniond::Identity();
    q_LW.coeffs() += 0.01 * Eigen::Vector4d(unif(rng), unif(rng), unif(rng), unif(rng));
    q_LW.normalize();
    Eigen::Vector3d t_LW(unif(rng), unif(rng), unif(rng));
    Eigen::Vector3d L_B(0.0, 0.0, -0.12);
    L_B += 0.02 * Eigen::Vector3d(unif(rng), unif(rng), unif(rng));

    const clic_calib::SE3d T_LW(q_LW, t_LW);
    const Eigen::Vector3d p_W =
        traj.position_wb(t_bar - t_d) +
        traj.rotation_wb(t_bar - t_d).matrix() * L_B;
    const Eigen::Vector3d c_L = T_LW * p_W + 0.02 * Eigen::Vector3d::Random();

    clic_calib::FixedTrajBodyCentroidJointLeverFactor factor(traj, t_bar, c_L,
                                                             sqrt_info);

    double t_d_var = t_d;
    std::vector<double*> params = {&t_d_var, q_LW.coeffs().data(), t_LW.data(),
                                   L_B.data()};
    const std::vector<int> block_sizes = {1, 4, 3, 3};
    EXPECT_TRUE(clic_calib::test::CompareJacobians(&factor, params, block_sizes,
                                                   kRelTol))
        << "trial " << trial;
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
