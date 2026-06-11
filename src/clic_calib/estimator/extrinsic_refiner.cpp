#include <clic_calib/estimator/extrinsic_refiner.h>

#include <clic_calib/estimator/ceres_so3_scope.h>
#include <clic_calib/factor/fixed_traj_apriltag_factor.h>
#include <clic_calib/factor/fixed_traj_body_centroid_factor.h>
#include <clic_calib/factor/fixed_traj_body_centroid_joint_lever_factor.h>
#include <clic_calib/factor/fixed_traj_sphere_factor.h>

#include <memory>
#include <stdexcept>

namespace clic_calib {

struct Stage2FactorCounts {
  int body_centroid_fixed = 0;
  int body_centroid_joint = 0;
  int sphere = 0;
  int apriltag = 0;
};

void AddStage2FixedFactors(
    ceres::Problem* problem,
    std::vector<std::unique_ptr<ceres::CostFunction>>* owned,
    const BodyTrajectory& fixed_traj,
    const std::vector<LiDARTargetObservation>& lidar,
    const std::vector<BodyClusterObservation>& body_cluster,
    const std::vector<AprilTagObservation>& tags,
    const LeverArmConfig& levers, const NoiseModel& noise,
    const ExtrinsicRefinerConfig& cfg, ExtrinsicOptimizeState* lidar_state,
    ExtrinsicOptimizeState* camera_state, Eigen::Vector3d* L_B_joint,
    ceres::LossFunction* lidar_loss, ceres::LossFunction* camera_loss,
    Stage2FactorCounts* counts = nullptr,
    const std::vector<Eigen::Vector3d>* L_B_per_obs = nullptr) {
  const double inv_sigma_r = 1.0 / noise.lidar_ranging_sigma_m;
  const double inv_sigma_pix = 1.0 / noise.camera_pixel_sigma;

  if (cfg.lidar_target_mode == LidarTargetMode::kBodyCluster) {
    const bool joint_L =
        cfg.body_lever_arm_mode == BodyLeverArmMode::kJointOptimize;
    if (joint_L && !L_B_joint) {
      throw std::runtime_error(
          "ExtrinsicRefiner: joint p_B mode requires L_B_joint state");
    }
    for (size_t obs_i = 0; obs_i < body_cluster.size(); ++obs_i) {
      const auto& obs = body_cluster[obs_i];
      const double t_bar = obs.t_sensor_;
      const Eigen::Matrix3d sqrt_info =
          cfg.use_centroid_cov_whitening
              ? noise.BodyCentroidSqrtInformationFromObservation(obs)
              : noise.BodyCentroidSqrtInformation(
                    obs.mean_range_m_,
                    static_cast<int>(obs.point_count_));
      const Eigen::Vector3d L_B =
          (L_B_per_obs && obs_i < L_B_per_obs->size())
              ? (*L_B_per_obs)[obs_i]
              : levers.L_B_to_body_centroid;
      if (joint_L) {
        owned->push_back(
            std::make_unique<FixedTrajBodyCentroidJointLeverFactor>(
                fixed_traj, t_bar, obs.centroid_L_, sqrt_info));
        problem->AddResidualBlock(
            owned->back().get(), lidar_loss, &lidar_state->t_d,
            lidar_state->q.coeffs().data(), lidar_state->t.data(),
            L_B_joint->data());
        if (counts) {
          ++counts->body_centroid_joint;
        }
      } else {
        owned->push_back(std::make_unique<FixedTrajBodyCentroidFactor>(
            fixed_traj, t_bar, obs.centroid_L_, L_B, sqrt_info));
        problem->AddResidualBlock(
            owned->back().get(), lidar_loss, &lidar_state->t_d,
            lidar_state->q.coeffs().data(), lidar_state->t.data());
        if (counts) {
          ++counts->body_centroid_fixed;
        }
      }
    }
  } else {
    for (const auto& scan : lidar) {
      const double t_bar = scan.t_sensor_;
      for (const auto& q_L : scan.points_L_) {
        owned->push_back(std::make_unique<FixedTrajSphereFactor>(
            fixed_traj, t_bar, q_L, levers.L_B_to_G, cfg.sphere_radius_m,
            inv_sigma_r));
        problem->AddResidualBlock(
            owned->back().get(), lidar_loss, &lidar_state->t_d,
            lidar_state->q.coeffs().data(), lidar_state->t.data());
        if (counts) {
          ++counts->sphere;
        }
      }
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
      if (counts) {
        ++counts->apriltag;
      }
    }
  }
}

ExtrinsicRefinerProblemAudit ExtrinsicRefiner::AuditProblem(
    const BodyTrajectory& fixed_traj,
    const std::vector<LiDARTargetObservation>& lidar,
    const std::vector<BodyClusterObservation>& body_cluster,
    const std::vector<AprilTagObservation>& tags,
    const LeverArmConfig& levers, const NoiseModel& noise,
    const CoarseExtrinsicInit& init, const ExtrinsicRefinerConfig& cfg) {
  ExtrinsicRefinerProblemAudit audit;
  audit.joint_path_active =
      cfg.lidar_target_mode == LidarTargetMode::kBodyCluster &&
      cfg.body_lever_arm_mode == BodyLeverArmMode::kJointOptimize;

  ExtrinsicOptimizeState lidar_state;
  ExtrinsicOptimizeState camera_state;
  lidar_state.SetFromInit(init, true);
  camera_state.SetFromInit(init, false);
  Eigen::Vector3d L_B_joint = levers.L_B_to_body_centroid;

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

  Eigen::Vector3d* L_B_ptr =
      audit.joint_path_active ? &L_B_joint : nullptr;
  Stage2FactorCounts counts;
  AddStage2FixedFactors(scope.problem.get(), &scope.owned_costs, fixed_traj,
                        lidar, body_cluster, tags, levers, noise, cfg,
                        &lidar_state, &camera_state, L_B_ptr, lidar_loss,
                        camera_loss, &counts);
  scope.SetLocalParamSO3(lidar_state.q.coeffs().data());
  scope.SetLocalParamSO3(camera_state.q.coeffs().data());

  audit.body_centroid_fixed_blocks = counts.body_centroid_fixed;
  audit.body_centroid_joint_blocks = counts.body_centroid_joint;
  audit.sphere_blocks = counts.sphere;
  audit.apriltag_blocks = counts.apriltag;
  audit.total_residual_blocks =
      counts.body_centroid_fixed + counts.body_centroid_joint + counts.sphere +
      counts.apriltag;

  if (audit.joint_path_active) {
    audit.L_B_block_present =
        scope.problem->HasParameterBlock(L_B_joint.data());
    if (audit.L_B_block_present) {
      audit.L_B_block_constant =
          scope.problem->IsParameterBlockConstant(L_B_joint.data());
    }
    ceres::Problem::EvaluateOptions eval_opts;
    eval_opts.apply_loss_function = true;
    eval_opts.parameter_blocks = {L_B_joint.data()};
    std::vector<double> grad_L_B(3, 0.0);
    scope.problem->Evaluate(eval_opts, &audit.cost_at_init, nullptr, &grad_L_B,
                            nullptr);
    audit.L_B_gradient_norm_at_init = Eigen::Map<const Eigen::Vector3d>(
                                          grad_L_B.data())
                                          .norm();

    const Eigen::Vector3d L_B_saved = L_B_joint;
    L_B_joint.x() += 1e-3;
    double cost_pert = 0.0;
    scope.problem->Evaluate(ceres::Problem::EvaluateOptions(), &cost_pert,
                            nullptr, nullptr, nullptr);
    audit.L_B_numeric_cost_drop_1mm_x = audit.cost_at_init - cost_pert;
    L_B_joint = L_B_saved;
  }

  return audit;
}

Eigen::Vector3d ExtrinsicRefiner::OptimizeJointBodyLeverAtFixedExtrinsic(
    const BodyTrajectory& fixed_traj,
    const std::vector<BodyClusterObservation>& body_cluster,
    const CoarseExtrinsicInit& init, const LeverArmConfig& levers,
    const NoiseModel& noise, const ExtrinsicRefinerConfig& cfg) {
  ExtrinsicOptimizeState lidar_state;
  lidar_state.SetFromInit(init, true);
  Eigen::Vector3d L_B_joint = levers.L_B_to_body_centroid;

  CeresSo3ProblemScope scope;
  // Linear in L_B; robust loss would bias p_B away from the observed-mean LS.
  ceres::LossFunction* lidar_loss = nullptr;

  ExtrinsicOptimizeState camera_dummy;
  camera_dummy.SetFromInit(init, false);
  ExtrinsicRefinerConfig joint_cfg = cfg;
  joint_cfg.body_lever_arm_mode = BodyLeverArmMode::kJointOptimize;
  joint_cfg.lidar_target_mode = LidarTargetMode::kBodyCluster;
  AddStage2FixedFactors(scope.problem.get(), &scope.owned_costs, fixed_traj, {},
                        body_cluster, {}, levers, noise, joint_cfg, &lidar_state,
                        &camera_dummy, &L_B_joint, lidar_loss, nullptr);
  scope.problem->SetParameterBlockConstant(&lidar_state.t_d);
  scope.problem->SetParameterBlockConstant(lidar_state.q.coeffs().data());
  scope.problem->SetParameterBlockConstant(lidar_state.t.data());

  ceres::Solver::Options opts;
  opts.max_num_iterations = cfg.max_iterations;
  opts.minimizer_progress_to_stdout = false;
  ceres::Solver::Summary summary;
  ceres::Solve(opts, scope.problem.get(), &summary);
  return L_B_joint;
}

ExtrinsicRefinerResult ExtrinsicRefiner::Refine(
    const BodyTrajectory& fixed_traj,
    const std::vector<LiDARTargetObservation>& lidar,
    const std::vector<BodyClusterObservation>& body_cluster,
    const std::vector<AprilTagObservation>& tags,
    const LeverArmConfig& levers, const NoiseModel& noise,
    const CoarseExtrinsicInit& init, const ExtrinsicRefinerConfig& cfg,
    const std::vector<Eigen::Vector3d>* L_B_per_obs) {
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

  if (cfg.body_lever_arm_mode == BodyLeverArmMode::kJointOptimize) {
    throw std::runtime_error(
        "ExtrinsicRefiner::Refine: use OptimizeJointBodyLeverAtFixedExtrinsic "
        "+ nominal re-init path (kJointOptimize is not a single-pass refine)");
  }

  AddStage2FixedFactors(scope.problem.get(), &scope.owned_costs, fixed_traj,
                        lidar, body_cluster, tags, levers, noise, cfg,
                        &out.lidar, &out.camera, nullptr, lidar_loss,
                        camera_loss, nullptr, L_B_per_obs);
  scope.SetLocalParamSO3(out.lidar.q.coeffs().data());
  if (scope.problem->HasParameterBlock(out.camera.q.coeffs().data())) {
    scope.SetLocalParamSO3(out.camera.q.coeffs().data());
  }

  scope.problem->SetParameterLowerBound(&out.lidar.t_d, 0, -cfg.t_d_max_abs_s);
  scope.problem->SetParameterUpperBound(&out.lidar.t_d, 0, cfg.t_d_max_abs_s);
  if (scope.problem->HasParameterBlock(&out.camera.t_d)) {
    scope.problem->SetParameterLowerBound(&out.camera.t_d, 0, -cfg.t_d_max_abs_s);
    scope.problem->SetParameterUpperBound(&out.camera.t_d, 0, cfg.t_d_max_abs_s);
  }

  ceres::Solver::Options opts;
  opts.max_num_iterations = cfg.max_iterations;
  opts.minimizer_progress_to_stdout = false;
  ceres::Solve(opts, scope.problem.get(), &out.summary);
  out.converged = out.summary.IsSolutionUsable();
  return out;
}

}  // namespace clic_calib
