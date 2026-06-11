/*
 * STEP 1 diagnostic — single-variable t_d^L observability (no confounders).
 *
 * Ceres problem: SphereImplicitFactor only; trajectory knots and T_LW fixed;
 * t_d^L is the sole free variable.
 */

#include <clic_calib/factor/apriltag_reproj_factor.h>
#include <clic_calib/factor/ceres_local_param.h>
#include <clic_calib/factor/sphere_implicit_factor.h>
#include <clic_calib/spline/spline_segment.h>
#include <clic_calib/spline/trajectory.h>
#include <clic_calib/utils/camera_projection.h>
#include <clic_calib/utils/lever_arm.h>
#include <clic_calib/utils/sophus_utils.hpp>
#include "gtest_ceres_guard.hpp"


#include <gtest/gtest.h>

#include <ceres/ceres.h>

#include <cmath>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace {

constexpr double kRBall = 0.10;
constexpr double kSigmaR = 0.02;
constexpr double kTdBoundLo = -0.5;
constexpr double kTdBoundHi = 0.5;
constexpr int kPointsPerScan = 24;

std::string ConfigDir() {
  std::filesystem::path root = std::filesystem::path(__FILE__).parent_path();
  root = root.parent_path().parent_path();
  const std::filesystem::path candidate = root / "config";
  if (std::filesystem::exists(candidate / "lever_arms.yaml")) {
    return candidate.string();
  }
  return "config";
}

clic_calib::BodyTrajectory MakeDynamicTrajectory() {
  clic_calib::BodyTrajectory traj(0.1, 0.0);
  const int num_knots = 12;
  const clic_calib::SE3d k0(clic_calib::SO3d::rotZ(0.0), Eigen::Vector3d::Zero());
  traj.setKnots(k0, num_knots);
  for (int i = 0; i < num_knots; ++i) {
    const double s = static_cast<double>(i) * 0.1;
    const clic_calib::SO3d R =
        clic_calib::SO3d::rotZ(0.05 * s) *
        clic_calib::SO3d::rotY(0.12 * std::sin(s));
    const Eigen::Vector3d p(0.5 * s, 0.3 * std::sin(s), 2.0 + 0.4 * std::sin(s));
    traj.setKnot(clic_calib::SE3d(R, p), i);
  }
  const int64_t t_end_ns = static_cast<int64_t>(5.0 * clic_calib::S_TO_NS);
  traj.extendKnotsTo(t_end_ns, traj.getKnotSO3(num_knots - 1),
                     traj.getKnotPos(num_knots - 1));
  return traj;
}

clic_calib::BodyTrajectory MakeStaticTrajectory() {
  clic_calib::BodyTrajectory traj(0.1, 0.0);
  const clic_calib::SE3d pose(clic_calib::SO3d::rotZ(0.1),
                              Eigen::Vector3d(1.0, 0.5, 2.0));
  const int num_knots = 12;
  traj.setKnots(pose, num_knots);
  const int64_t t_end_ns = static_cast<int64_t>(5.0 * clic_calib::S_TO_NS);
  traj.extendKnotsTo(t_end_ns, pose.so3(), pose.translation());
  return traj;
}

bool GetActiveKnotPointers(const clic_calib::BodyTrajectory& traj, int64_t t_ns,
                           std::array<double*, clic_calib::SplineOrder>* rot_knots,
                           std::array<double*, clic_calib::SplineOrder>* pos_knots) {
  if (traj.numKnots() < static_cast<size_t>(clic_calib::SplineOrder)) {
    return false;
  }
  clic_calib::SplineSegmentMeta<clic_calib::SplineOrder> meta(
      traj.minTimeNs(), traj.getDtNs(), traj.numKnots());
  if (t_ns < meta.MinTimeNs() || t_ns >= meta.MaxTimeNs()) {
    return false;
  }
  const auto ui = meta.computeTIndexNs(t_ns);
  const size_t s = ui.second;
  if (s + clic_calib::SplineOrder > traj.numKnots()) {
    return false;
  }
  for (int i = 0; i < clic_calib::SplineOrder; ++i) {
    (*rot_knots)[i] = const_cast<double*>(
        traj.getKnotSO3(static_cast<int>(s + i)).data());
    (*pos_knots)[i] = const_cast<double*>(
        traj.getKnotPos(static_cast<int>(s + i)).data());
  }
  return true;
}

struct ScanObservation {
  double bar_t_s;
  std::vector<Eigen::Vector3d> points_L;
};

std::vector<ScanObservation> SynthesizeLidarScans(
    const clic_calib::BodyTrajectory& gt_traj, const clic_calib::SE3d& T_LW_gt,
    const Eigen::Vector3d& L_B_to_G, double t_d_gt) {
  std::vector<ScanObservation> scans;
  for (double t = 0.5; t <= 4.5; t += 0.4) {
    ScanObservation scan;
    scan.bar_t_s = t + t_d_gt;
    const Eigen::Vector3d p_G_W = gt_traj.sphere_center_w(t, L_B_to_G);
    const Eigen::Vector3d p_G_L = T_LW_gt * p_G_W;
    for (int k = 0; k < kPointsPerScan; ++k) {
      const double phi = 2.0 * M_PI * k / kPointsPerScan;
      const Eigen::Vector3d dir(std::cos(phi), std::sin(phi), 0.0);
      scan.points_L.push_back(p_G_L + kRBall * dir.normalized());
    }
    scans.push_back(std::move(scan));
  }
  return scans;
}

struct TdSingleVarResult {
  double t_d_est = 0.0;
  double t_d_init = 0.0;
  ceres::Solver::Summary summary;
};

TdSingleVarResult SolveTdOnly(clic_calib::BodyTrajectory& gt_traj,
                              const clic_calib::SE3d& T_LW_gt,
                              const Eigen::Vector3d& L_B_to_G,
                              double t_d_gt, double t_d_init) {
  const std::vector<ScanObservation> scans =
      SynthesizeLidarScans(gt_traj, T_LW_gt, L_B_to_G, t_d_gt);

  clic_calib::SplineSegmentMeta<clic_calib::SplineOrder> meta(
      gt_traj.minTimeNs(), gt_traj.getDtNs(), gt_traj.numKnots());

  double t_d = t_d_init;
  Eigen::Quaterniond q_LW = T_LW_gt.unit_quaternion();
  Eigen::Vector3d t_LW = T_LW_gt.translation();

  auto so3_local =
      std::make_unique<clic_calib::LieLocalParameterization<clic_calib::SO3d>>();
  std::vector<std::unique_ptr<ceres::CostFunction>> owned_costs;
  owned_costs.reserve(scans.size() * kPointsPerScan);

  ceres::Problem::Options problem_options;
  problem_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  problem_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  problem_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  ceres::Problem problem(problem_options);

  problem.AddParameterBlock(q_LW.coeffs().data(), 4, so3_local.get());
  problem.AddParameterBlock(t_LW.data(), 3);
  problem.SetParameterBlockConstant(q_LW.coeffs().data());
  problem.SetParameterBlockConstant(t_LW.data());

  for (const auto& scan : scans) {
    const int64_t bar_t_ns = static_cast<int64_t>(scan.bar_t_s * clic_calib::S_TO_NS);
    const int64_t t_eval_ns =
        bar_t_ns - static_cast<int64_t>(t_d * clic_calib::S_TO_NS);
    std::array<double*, clic_calib::SplineOrder> rot_knots{};
    std::array<double*, clic_calib::SplineOrder> pos_knots{};
    if (!GetActiveKnotPointers(gt_traj, t_eval_ns, &rot_knots, &pos_knots)) {
      continue;
    }
    for (int i = 0; i < clic_calib::SplineOrder; ++i) {
      if (!problem.HasParameterBlock(rot_knots[i])) {
        problem.AddParameterBlock(rot_knots[i], 4, so3_local.get());
        problem.SetParameterBlockConstant(rot_knots[i]);
      }
      if (!problem.HasParameterBlock(pos_knots[i])) {
        problem.AddParameterBlock(pos_knots[i], 3);
        problem.SetParameterBlockConstant(pos_knots[i]);
      }
    }
    for (const auto& q_L : scan.points_L) {
      owned_costs.push_back(
          std::make_unique<clic_calib::analytic_derivative::SphereImplicitFactor>(
              bar_t_ns, q_L, L_B_to_G, kRBall, kSigmaR, meta));
      std::vector<double*> blocks = {&t_d, q_LW.coeffs().data(), t_LW.data()};
      blocks.insert(blocks.end(), rot_knots.begin(), rot_knots.end());
      blocks.insert(blocks.end(), pos_knots.begin(), pos_knots.end());
      problem.AddResidualBlock(owned_costs.back().get(), nullptr, blocks);
    }
  }

  problem.SetParameterLowerBound(&t_d, 0, kTdBoundLo);
  problem.SetParameterUpperBound(&t_d, 0, kTdBoundHi);

  ceres::Solver::Options opts;
  opts.max_num_iterations = 100;
  opts.function_tolerance = 1e-12;
  opts.gradient_tolerance = 1e-14;
  opts.minimizer_progress_to_stdout = false;

  TdSingleVarResult result;
  result.t_d_init = t_d_init;
  ceres::Solve(opts, &problem, &result.summary);
  result.t_d_est = t_d;
  return result;
}

void PrintSummary(const char* label, double t_d_gt, const TdSingleVarResult& r) {
  std::cout << "=== " << label << " ===\n"
            << "  t_d_gt:     " << t_d_gt << " s\n"
            << "  t_d_init:   " << r.t_d_init << " s\n"
            << "  t_d_est:    " << r.t_d_est << " s\n"
            << "  error:      " << (r.t_d_est - t_d_gt) * 1000.0 << " ms\n"
            << "  initial_cost: " << r.summary.initial_cost << "\n"
            << "  final_cost:   " << r.summary.final_cost << "\n"
            << "  iterations:   " << r.summary.num_successful_steps << "\n"
            << "  termination:  "
            << ceres::TerminationTypeToString(r.summary.termination_type) << "\n"
            << r.summary.FullReport() << "\n";
}

}  // namespace

TEST(TdSingleVariable, RecoversPositive30msWithDynamicMotion) {
  const auto levers =
      clic_calib::LeverArmConfig::from_yaml(ConfigDir() + "/lever_arms.yaml");
  clic_calib::BodyTrajectory gt_traj = MakeDynamicTrajectory();
  const clic_calib::SE3d T_LW_gt(clic_calib::SO3d::rotY(-0.15),
                                 Eigen::Vector3d(0.0, 0.0, 0.5));
  constexpr double kTdGt = 0.030;

  const TdSingleVarResult result =
      SolveTdOnly(gt_traj, T_LW_gt, levers.L_B_to_G, kTdGt, 0.0);
  PrintSummary("RecoversPositive30ms", kTdGt, result);

  EXPECT_TRUE(result.summary.IsSolutionUsable());
  EXPECT_NEAR(result.t_d_est, kTdGt, 0.002)
      << "t_d should recover to within ±2 ms of 30 ms";
}

TEST(TdSingleVariable, RecoversNegative15msWithDynamicMotion) {
  const auto levers =
      clic_calib::LeverArmConfig::from_yaml(ConfigDir() + "/lever_arms.yaml");
  clic_calib::BodyTrajectory gt_traj = MakeDynamicTrajectory();
  const clic_calib::SE3d T_LW_gt(clic_calib::SO3d::rotY(-0.15),
                                 Eigen::Vector3d(0.0, 0.0, 0.5));
  constexpr double kTdGt = -0.015;

  const TdSingleVarResult result =
      SolveTdOnly(gt_traj, T_LW_gt, levers.L_B_to_G, kTdGt, 0.0);
  PrintSummary("RecoversNegative15ms", kTdGt, result);

  EXPECT_TRUE(result.summary.IsSolutionUsable());
  EXPECT_NEAR(result.t_d_est, kTdGt, 0.002)
      << "t_d should recover to within ±2 ms of -15 ms";
}

TEST(TdSingleVariable, StaticMotionTdIsUnobservable) {
  const auto levers =
      clic_calib::LeverArmConfig::from_yaml(ConfigDir() + "/lever_arms.yaml");
  clic_calib::BodyTrajectory gt_traj = MakeStaticTrajectory();
  const clic_calib::SE3d T_LW_gt(clic_calib::SO3d::rotY(-0.15),
                                 Eigen::Vector3d(0.0, 0.0, 0.5));
  constexpr double kTdGt = 0.030;

  const TdSingleVarResult result =
      SolveTdOnly(gt_traj, T_LW_gt, levers.L_B_to_G, kTdGt, 0.0);
  PrintSummary("StaticMotionUnobservable", kTdGt, result);

  const double recovery_error_ms = std::abs(result.t_d_est - kTdGt) * 1000.0;
  const double motion_from_init_ms =
      std::abs(result.t_d_est - result.t_d_init) * 1000.0;

  EXPECT_GT(recovery_error_ms, 2.0)
      << "static motion: t_d must NOT recover gt within 2 ms";
  EXPECT_LT(motion_from_init_ms, 1.0)
      << "static motion: optimizer should not move t_d from init";
  EXPECT_NEAR(result.summary.initial_cost, result.summary.final_cost, 1e-6)
      << "static motion: cost should be unchanged (t_d unobservable)";
}

struct TagScanObservation {
  double bar_t_s;
  Eigen::Vector2d u_obs;
};

std::vector<TagScanObservation> SynthesizeTagScans(
    const clic_calib::BodyTrajectory& gt_traj, const clic_calib::SE3d& T_CW_gt,
    const Eigen::Vector3d& L_B_to_G_M, double t_d_gt) {
  const clic_calib::PinholeIntrinsics K{600.0, 600.0, 320.0, 240.0};
  const clic_calib::RadtanDistortion dist;
  std::vector<TagScanObservation> scans;
  for (double t = 0.5; t <= 4.5; t += 0.35) {
    TagScanObservation scan;
    scan.bar_t_s = t + t_d_gt;
    const clic_calib::SE3d T_WB = gt_traj.pose_wb(t);
    const Eigen::Vector3d p_M_W = T_WB * L_B_to_G_M;
    const Eigen::Vector3d p_M_C = T_CW_gt * p_M_W;
    scan.u_obs = clic_calib::ProjectRadtan(p_M_C, K, dist, nullptr);
    scans.push_back(scan);
  }
  return scans;
}

TdSingleVarResult SolveTdCameraOnly(clic_calib::BodyTrajectory& gt_traj,
                                    const clic_calib::SE3d& T_CW_gt,
                                    const Eigen::Vector3d& L_B_to_G_M,
                                    double t_d_gt, double t_d_init) {
  const std::vector<TagScanObservation> scans =
      SynthesizeTagScans(gt_traj, T_CW_gt, L_B_to_G_M, t_d_gt);

  clic_calib::SplineSegmentMeta<clic_calib::SplineOrder> meta(
      gt_traj.minTimeNs(), gt_traj.getDtNs(), gt_traj.numKnots());

  double t_d = t_d_init;
  Eigen::Quaterniond q_CW = T_CW_gt.unit_quaternion();
  Eigen::Vector3d t_CW = T_CW_gt.translation();
  const clic_calib::PinholeIntrinsics K{600.0, 600.0, 320.0, 240.0};
  const clic_calib::RadtanDistortion dist;
  constexpr double kSigmaPix = 1.0;

  auto so3_local =
      std::make_unique<clic_calib::LieLocalParameterization<clic_calib::SO3d>>();
  std::vector<std::unique_ptr<ceres::CostFunction>> owned_costs;
  owned_costs.reserve(scans.size());

  ceres::Problem::Options problem_options;
  problem_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  problem_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  problem_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  ceres::Problem problem(problem_options);

  problem.AddParameterBlock(q_CW.coeffs().data(), 4, so3_local.get());
  problem.AddParameterBlock(t_CW.data(), 3);
  problem.SetParameterBlockConstant(q_CW.coeffs().data());
  problem.SetParameterBlockConstant(t_CW.data());

  for (const auto& scan : scans) {
    const int64_t bar_t_ns = static_cast<int64_t>(scan.bar_t_s * clic_calib::S_TO_NS);
    const int64_t t_eval_ns =
        bar_t_ns - static_cast<int64_t>(t_d * clic_calib::S_TO_NS);
    std::array<double*, clic_calib::SplineOrder> rot_knots{};
    std::array<double*, clic_calib::SplineOrder> pos_knots{};
    if (!GetActiveKnotPointers(gt_traj, t_eval_ns, &rot_knots, &pos_knots)) {
      continue;
    }
    for (int i = 0; i < clic_calib::SplineOrder; ++i) {
      if (!problem.HasParameterBlock(rot_knots[i])) {
        problem.AddParameterBlock(rot_knots[i], 4, so3_local.get());
        problem.SetParameterBlockConstant(rot_knots[i]);
      }
      if (!problem.HasParameterBlock(pos_knots[i])) {
        problem.AddParameterBlock(pos_knots[i], 3);
        problem.SetParameterBlockConstant(pos_knots[i]);
      }
    }
    owned_costs.push_back(
        std::make_unique<clic_calib::analytic_derivative::AprilTagReprojFactor>(
            bar_t_ns, scan.u_obs, L_B_to_G_M, K, dist, kSigmaPix, meta));
    std::vector<double*> blocks = {&t_d, q_CW.coeffs().data(), t_CW.data()};
    blocks.insert(blocks.end(), rot_knots.begin(), rot_knots.end());
    blocks.insert(blocks.end(), pos_knots.begin(), pos_knots.end());
    problem.AddResidualBlock(owned_costs.back().get(), nullptr, blocks);
  }

  problem.SetParameterLowerBound(&t_d, 0, kTdBoundLo);
  problem.SetParameterUpperBound(&t_d, 0, kTdBoundHi);

  ceres::Solver::Options opts;
  opts.max_num_iterations = 100;
  opts.function_tolerance = 1e-12;
  opts.gradient_tolerance = 1e-14;

  TdSingleVarResult result;
  result.t_d_init = t_d_init;
  ceres::Solve(opts, &problem, &result.summary);
  result.t_d_est = t_d;
  return result;
}

TEST(TdSingleVariableCamera, RecoversNegative15msWithDynamicMotion) {
  const auto levers =
      clic_calib::LeverArmConfig::from_yaml(ConfigDir() + "/lever_arms.yaml");
  clic_calib::BodyTrajectory gt_traj = MakeDynamicTrajectory();
  const clic_calib::SE3d T_CW_gt(clic_calib::SO3d::rotX(0.1),
                                 Eigen::Vector3d(2.0, 1.5, 0.2));
  const Eigen::Vector3d L_B_to_G_M =
      levers.L_B_to_G + levers.L_G_to_M.at(0);
  constexpr double kTdGt = -0.015;

  const TdSingleVarResult result =
      SolveTdCameraOnly(gt_traj, T_CW_gt, L_B_to_G_M, kTdGt, 0.0);
  PrintSummary("CameraRecoversNegative15ms", kTdGt, result);

  EXPECT_TRUE(result.summary.IsSolutionUsable());
  EXPECT_NEAR(result.t_d_est, kTdGt, 0.002);
}

int main(int argc, char** argv) { return ClicGTestRunAll(argc, argv); }
