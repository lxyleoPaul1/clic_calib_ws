#include <clic_calib/estimator/stage2_extrinsic_fim.h>

#include <clic_calib/estimator/ceres_so3_scope.h>
#include <clic_calib/factor/fixed_traj_apriltag_factor.h>
#include <clic_calib/factor/fixed_traj_sphere_factor.h>

#include <cmath>
#include <limits>
#include <stdexcept>

namespace clic_calib {
namespace {

void AddStage2FactorsForFim(
    ceres::Problem* problem,
    std::vector<std::unique_ptr<ceres::CostFunction>>* owned,
    const BodyTrajectory& fixed_traj,
    const std::vector<LiDARTargetObservation>& lidar,
    const std::vector<AprilTagObservation>& tags,
    const LeverArmConfig& levers, const NoiseModel& noise, double sphere_radius_m,
    const PinholeIntrinsics& K, const RadtanDistortion& dist, int marker_id,
    ExtrinsicOptimizeState* lidar_state, ExtrinsicOptimizeState* camera_state) {
  const double inv_sigma_r = 1.0 / noise.lidar_ranging_sigma_m;
  const double inv_sigma_pix = 1.0 / noise.camera_pixel_sigma;

  for (const auto& scan : lidar) {
    const double t_bar = scan.t_sensor_;
    for (const auto& q_L : scan.points_L_) {
      owned->push_back(std::make_unique<FixedTrajSphereFactor>(
          fixed_traj, t_bar, q_L, levers.L_B_to_G, sphere_radius_m,
          inv_sigma_r));
      problem->AddResidualBlock(owned->back().get(), nullptr, &lidar_state->t_d,
                                lidar_state->q.coeffs().data(),
                                lidar_state->t.data());
    }
  }

  const auto lg_it = levers.L_G_to_M.find(marker_id);
  if (lg_it == levers.L_G_to_M.end()) {
    throw std::runtime_error("BuildStage2ExtrinsicInformation: no L_G_to_M");
  }
  const Eigen::Vector3d L_G_to_M = lg_it->second;

  for (const auto& det : tags) {
    const double t_bar = det.t_sensor_;
    for (int c = 0; c < 4; ++c) {
      owned->push_back(std::make_unique<FixedTrajAprilTagFactor>(
          fixed_traj, t_bar, det.corners_pixel_[c], levers.L_B_to_G, L_G_to_M,
          K, dist, inv_sigma_pix));
      problem->AddResidualBlock(owned->back().get(), nullptr,
                                &camera_state->t_d,
                                camera_state->q.coeffs().data(),
                                camera_state->t.data());
    }
  }
}

}  // namespace

Eigen::MatrixXd InformationFromCeresJacobian(const ceres::CRSMatrix& crs) {
  const int n = std::max(0, crs.num_cols);
  Eigen::MatrixXd F = Eigen::MatrixXd::Zero(n, n);
  for (int row = 0; row < crs.num_rows; ++row) {
    const int begin = crs.rows[row];
    const int end = crs.rows[row + 1];
    for (int k = begin; k < end; ++k) {
      const int col_k = crs.cols[k];
      const double val_k = crs.values[k];
      for (int l = k; l < end; ++l) {
        const int col_l = crs.cols[l];
        const double val_l = crs.values[l];
        F(col_k, col_l) += val_k * val_l;
        if (col_k != col_l) {
          F(col_l, col_k) += val_k * val_l;
        }
      }
    }
  }
  return F;
}

SymmetricInformationReport AnalyzeInformationMatrix(const Eigen::MatrixXd& F) {
  SymmetricInformationReport r;
  r.dim = static_cast<int>(F.rows());
  if (F.rows() == 0) {
    return r;
  }
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(F);
  const Eigen::VectorXd evals = es.eigenvalues();
  r.lambda_min = evals.minCoeff();
  r.lambda_max = evals.maxCoeff();
  r.is_pd = r.lambda_min > 0.0;
  if (r.is_pd && r.lambda_min > 0.0) {
    r.cond = r.lambda_max / r.lambda_min;
  }
  const double tol =
      std::max(1e-12, 1e-10 * std::max(std::abs(r.lambda_max), 1.0));
  for (int i = 0; i < evals.size(); ++i) {
    if (evals(i) > tol) {
      ++r.rank;
    }
  }
  return r;
}

Eigen::MatrixXd BuildStage2ExtrinsicInformation(
    const BodyTrajectory& fixed_traj,
    const std::vector<LiDARTargetObservation>& lidar,
    const std::vector<AprilTagObservation>& tags,
    const ExtrinsicOptimizeState& lidar_state,
    const ExtrinsicOptimizeState& camera_state,
    const LeverArmConfig& levers, const NoiseModel& noise,
    double sphere_radius_m, const PinholeIntrinsics& K,
    const RadtanDistortion& dist, int marker_id) {
  ExtrinsicOptimizeState lw = lidar_state;
  ExtrinsicOptimizeState cw = camera_state;
  CeresSo3ProblemScope scope;
  AddStage2FactorsForFim(scope.problem.get(), &scope.owned_costs, fixed_traj,
                         lidar, tags, levers, noise, sphere_radius_m, K, dist,
                         marker_id, &lw, &cw);
  scope.SetLocalParamSO3(lw.q.coeffs().data());
  scope.SetLocalParamSO3(cw.q.coeffs().data());

  ceres::Problem::EvaluateOptions opts;
  opts.apply_loss_function = false;
  opts.num_threads = 1;
  ceres::CRSMatrix J;
  scope.problem->Evaluate(opts, nullptr, nullptr, nullptr, &J);
  return InformationFromCeresJacobian(J);
}

}  // namespace clic_calib
