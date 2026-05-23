#include <clic_calib/estimator/extrinsic_refiner.h>

#include <clic_calib/estimator/ceres_so3_scope.h>
#include <clic_calib/factor/fixed_traj_apriltag_factor.h>
#include <clic_calib/factor/fixed_traj_sphere_factor.h>

#include <memory>
#include <stdexcept>

namespace clic_calib {
namespace {

void AddStage2FixedFactors(
    ceres::Problem* problem,
    std::vector<std::unique_ptr<ceres::CostFunction>>* owned,
    const BodyTrajectory& fixed_traj,
    const std::vector<LiDARTargetObservation>& lidar,
    const std::vector<AprilTagObservation>& tags,
    const LeverArmConfig& levers, const NoiseModel& noise,
    const ExtrinsicRefinerConfig& cfg, ExtrinsicOptimizeState* lidar_state,
    ExtrinsicOptimizeState* camera_state,
    ceres::LossFunction* lidar_loss, ceres::LossFunction* camera_loss) {
  const double inv_sigma_r = 1.0 / noise.lidar_ranging_sigma_m;
  const double inv_sigma_pix = 1.0 / noise.camera_pixel_sigma;

  for (const auto& scan : lidar) {
    const double t_bar = scan.t_sensor_;
    for (const auto& q_L : scan.points_L_) {
      owned->push_back(std::make_unique<FixedTrajSphereFactor>(
          fixed_traj, t_bar, q_L, levers.L_B_to_G, cfg.sphere_radius_m,
          inv_sigma_r));
      problem->AddResidualBlock(
          owned->back().get(), lidar_loss, &lidar_state->t_d,
          lidar_state->q.coeffs().data(), lidar_state->t.data());
    }
  }

  const auto lg_it = levers.L_G_to_M.find(cfg.marker_id);
  if (lg_it == levers.L_G_to_M.end()) {
    throw std::runtime_error("ExtrinsicRefiner: missing L_G_to_M");
  }
  const Eigen::Vector3d L_G_to_M = lg_it->second;

  for (const auto& det : tags) {
    const double t_bar = det.t_sensor_;
    for (int c = 0; c < 4; ++c) {
      owned->push_back(std::make_unique<FixedTrajAprilTagFactor>(
          fixed_traj, t_bar, det.corners_pixel_[c], levers.L_B_to_G, L_G_to_M,
          cfg.camera_K, cfg.camera_dist, inv_sigma_pix));
      problem->AddResidualBlock(
          owned->back().get(), camera_loss, &camera_state->t_d,
          camera_state->q.coeffs().data(), camera_state->t.data());
    }
  }
}

}  // namespace

ExtrinsicRefinerResult ExtrinsicRefiner::Refine(
    const BodyTrajectory& fixed_traj,
    const std::vector<LiDARTargetObservation>& lidar,
    const std::vector<AprilTagObservation>& tags,
    const LeverArmConfig& levers, const NoiseModel& noise,
    const CoarseExtrinsicInit& init, const ExtrinsicRefinerConfig& cfg) {
  ExtrinsicRefinerResult out;
  out.lidar.SetFromInit(init, true);
  out.camera.SetFromInit(init, false);

  CeresSo3ProblemScope scope;
  std::unique_ptr<ceres::CauchyLoss> cauchy;
  std::unique_ptr<ceres::HuberLoss> huber;
  ceres::LossFunction* lidar_loss = nullptr;
  ceres::LossFunction* camera_loss = nullptr;
  if (cfg.lidar_cauchy_scale > 0.0) {
    cauchy = std::make_unique<ceres::CauchyLoss>(cfg.lidar_cauchy_scale);
    lidar_loss = cauchy.get();
  }
  if (cfg.camera_huber_delta_px > 0.0) {
    huber = std::make_unique<ceres::HuberLoss>(cfg.camera_huber_delta_px);
    camera_loss = huber.get();
  }

  AddStage2FixedFactors(scope.problem.get(), &scope.owned_costs, fixed_traj,
                        lidar, tags, levers, noise, cfg, &out.lidar, &out.camera,
                        lidar_loss, camera_loss);
  scope.SetLocalParamSO3(out.lidar.q.coeffs().data());
  scope.SetLocalParamSO3(out.camera.q.coeffs().data());

  scope.problem->SetParameterLowerBound(&out.lidar.t_d, 0, -cfg.t_d_max_abs_s);
  scope.problem->SetParameterUpperBound(&out.lidar.t_d, 0, cfg.t_d_max_abs_s);
  scope.problem->SetParameterLowerBound(&out.camera.t_d, 0, -cfg.t_d_max_abs_s);
  scope.problem->SetParameterUpperBound(&out.camera.t_d, 0, cfg.t_d_max_abs_s);

  ceres::Solver::Options opts;
  opts.max_num_iterations = cfg.max_iterations;
  opts.minimizer_progress_to_stdout = false;
  ceres::Solve(opts, scope.problem.get(), &out.summary);
  out.converged = out.summary.IsSolutionUsable();
  return out;
}

}  // namespace clic_calib
