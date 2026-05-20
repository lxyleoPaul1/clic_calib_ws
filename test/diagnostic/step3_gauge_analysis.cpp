/*
 * STEP 3 diagnostic — gauge / observability on Python E2E synthetic data.
 *
 * Usage:
 *   step3_gauge_analysis <config_dir> <obs.clicob> <rtk.csv> [--gt-prior]
 */

#include <clic_calib/estimator/calibration_estimator.h>
#include <clic_calib/io/observation_archive.h>
#include <clic_calib/io/rtk_reader.h>
#include <clic_calib/utils/sophus_utils.hpp>

#include <ceres/crs_matrix.h>

#include <cmath>
#include <iostream>
#include <map>
#include <string>
#include <vector>

namespace {

Eigen::MatrixXd InformationFromCRSJacobian(const ceres::CRSMatrix& crs) {
  const int n = std::max(0, crs.num_cols);
  Eigen::MatrixXd F = Eigen::MatrixXd::Zero(n, n);
  for (int r = 0; r < crs.num_rows; ++r) {
    const int begin = crs.rows[r];
    const int end = crs.rows[r + 1];
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

struct ParameterMap {
  int td_lidar = -1;
  int td_camera = -1;
  std::vector<int> lidar_trans;
  std::vector<int> lidar_rot;
  std::vector<int> camera_trans;
  std::vector<int> traj_pos;
  std::vector<int> traj_rot;
};

ParameterMap BuildParameterMap(const clic_calib::CalibrationEstimator& estimator) {
  const ceres::Problem& prob = estimator.problem();
  const clic_calib::AnalysisParameterLayout layout =
      estimator.analysis_parameter_layout();

  std::vector<double*> blocks;
  prob.GetParameterBlocks(&blocks);

  ParameterMap map;
  int cursor = 0;
  std::vector<int> one_dof_blocks;

  for (double* ptr : blocks) {
    const int local = prob.ParameterBlockLocalSize(ptr);
    const int global = prob.ParameterBlockSize(ptr);
    const bool extrinsic =
        std::find(layout.extrinsic_local_indices.begin(),
                  layout.extrinsic_local_indices.end(),
                  cursor) != layout.extrinsic_local_indices.end() ||
        (local == 3 && global == 4 &&
         std::find(layout.extrinsic_local_indices.begin(),
                   layout.extrinsic_local_indices.end(), cursor) !=
             layout.extrinsic_local_indices.end());

    bool is_extr = false;
    for (int k = 0; k < local; ++k) {
      if (std::find(layout.extrinsic_local_indices.begin(),
                    layout.extrinsic_local_indices.end(),
                    cursor + k) != layout.extrinsic_local_indices.end()) {
        is_extr = true;
        break;
      }
    }

    if (local == 1) {
      one_dof_blocks.push_back(cursor);
    } else if (is_extr && local == 3 && global == 4) {
      for (int k = 0; k < 3; ++k) {
        if (static_cast<int>(map.lidar_rot.size()) < 3 &&
            static_cast<int>(layout.lidar_rot_local_indices.size()) == 3 &&
            cursor + k == layout.lidar_rot_local_indices[k]) {
          map.lidar_rot.push_back(cursor + k);
        } else if (static_cast<int>(map.lidar_rot.size()) == 3 &&
                   static_cast<int>(map.lidar_trans.size()) < 3 &&
                   static_cast<int>(layout.lidar_trans_local_indices.size()) == 3 &&
                   cursor + k == layout.lidar_trans_local_indices[k]) {
          map.lidar_trans.push_back(cursor + k);
        } else if (map.lidar_rot.size() == 3 && map.lidar_trans.size() == 3 &&
                   static_cast<int>(map.camera_trans.size()) < 3) {
          map.camera_trans.push_back(cursor + k);
        }
      }
    } else if (!is_extr && local == 3 && global == 3) {
      for (int k = 0; k < 3; ++k) {
        map.traj_pos.push_back(cursor + k);
      }
    } else if (!is_extr && local == 3 && global == 4) {
      for (int k = 0; k < 3; ++k) {
        map.traj_rot.push_back(cursor + k);
      }
    }

    cursor += local;
  }

  if (one_dof_blocks.size() >= 1) {
    map.td_lidar = one_dof_blocks[0];
  }
  if (one_dof_blocks.size() >= 2) {
    map.td_camera = one_dof_blocks[1];
  }
  return map;
}

double ComponentEnergy(const Eigen::VectorXd& v, const std::vector<int>& idx) {
  double s = 0.0;
  for (int i : idx) {
    if (i >= 0 && i < v.size()) {
      s += v(i) * v(i);
    }
  }
  return std::sqrt(s);
}

struct RunResult {
  double t_d_lidar = 0.0;
  double t_d_camera = 0.0;
  clic_calib::SE3d T_LW;
  double final_cost = 0.0;
  double lambda_min_full = 0.0;
  double lambda_min_fext = 0.0;
  Eigen::VectorXd worst_full;
  Eigen::VectorXd worst_fext;
  ParameterMap pmap;
};

RunResult AnalyzeRun(clic_calib::CalibrationEstimator* estimator, int max_iters) {
  const ceres::Solver::Summary summary = estimator->solve(max_iters);
  RunResult out;
  out.final_cost = summary.final_cost;
  out.t_d_lidar = estimator->get_t_d_lidar(0);
  out.t_d_camera = estimator->get_t_d_camera(0);
  out.T_LW = estimator->get_T_LW(0);
  out.pmap = BuildParameterMap(*estimator);

  ceres::Problem::EvaluateOptions eval_opts;
  eval_opts.apply_loss_function = false;
  eval_opts.num_threads = 1;

  ceres::CRSMatrix crs;
  std::vector<double> residuals;
  double cost = 0.0;
  estimator->problem().Evaluate(eval_opts, &cost, &residuals, nullptr, &crs);

  const Eigen::MatrixXd F_full = InformationFromCRSJacobian(crs);
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es_full(F_full);
  out.lambda_min_full = es_full.eigenvalues().minCoeff();
  out.worst_full = es_full.eigenvectors().col(0);

  clic_calib::ObservabilityAnalyzer analyzer;
  const clic_calib::ObservabilityReport fext = analyzer.analyze(*estimator);
  out.lambda_min_fext = fext.lambda_min;
  out.worst_fext = fext.worst_eigenvector;

  return out;
}

void PrintRun(const char* label, const RunResult& r) {
  const Eigen::VectorXd& v = r.worst_full;
  const double norm_v = v.norm() > 0 ? v.norm() : 1.0;
  const Eigen::VectorXd vn = v / norm_v;

  std::cout << "=== " << label << " ===\n"
            << "  final_cost:    " << r.final_cost << "\n"
            << "  T_LW.t:        " << r.T_LW.translation().transpose() << "\n"
            << "  t_d_lidar:     " << r.t_d_lidar << " s\n"
            << "  t_d_camera:    " << r.t_d_camera << " s\n"
            << "  lambda_min(F_full): " << r.lambda_min_full << "\n"
            << "  lambda_min(F_ext):  " << r.lambda_min_fext << "\n"
            << "  |v_min| components (normalized):\n"
            << "    t_d_lidar:   " << ComponentEnergy(vn, {r.pmap.td_lidar})
            << "\n"
            << "    t_d_camera:  " << ComponentEnergy(vn, {r.pmap.td_camera})
            << "\n"
            << "    T_LW trans:  "
            << ComponentEnergy(vn, r.pmap.lidar_trans) << "\n"
            << "    T_LW rot:    " << ComponentEnergy(vn, r.pmap.lidar_rot)
            << "\n"
            << "    T_CW trans:  "
            << ComponentEnergy(vn, r.pmap.camera_trans) << "\n"
            << "    traj pos:    " << ComponentEnergy(vn, r.pmap.traj_pos)
            << "\n"
            << "    traj rot:    " << ComponentEnergy(vn, r.pmap.traj_rot)
            << "\n";
}

bool LoadData(const std::string& obs_path, const std::string& rtk_path,
              clic_calib::ObservationArchive::LidarBySensor* lidar,
              clic_calib::ObservationArchive::AprilTagBySensor* apriltag,
              std::vector<clic_calib::RTKMeasurement>* rtk) {
  if (!clic_calib::ObservationArchive::Read(obs_path, lidar, apriltag)) {
    return false;
  }
  clic_calib::CSVReader reader;
  *rtk = reader.read(rtk_path);
  return !rtk->empty();
}

void PopulateEstimator(clic_calib::CalibrationEstimator* estimator,
                       const clic_calib::ObservationArchive::LidarBySensor& lidar,
                       const clic_calib::ObservationArchive::AprilTagBySensor& tag,
                       const std::vector<clic_calib::RTKMeasurement>& rtk) {
  estimator->add_rtk_measurements(rtk);
  for (const auto& kv : lidar) {
    estimator->add_lidar_target_observations(kv.first, kv.second);
  }
  for (const auto& kv : tag) {
    estimator->add_apriltag_observations(kv.first, kv.second);
  }
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 4) {
    std::cerr << "Usage: " << argv[0]
              << " <config_dir> <obs.clicob> <rtk.csv> [--gt-prior]\n";
    return 1;
  }

  const std::string config_dir = argv[1];
  const std::string obs_path = argv[2];
  const std::string rtk_path = argv[3];
  bool gt_prior = false;
  for (int i = 4; i < argc; ++i) {
    if (std::string(argv[i]) == "--gt-prior") {
      gt_prior = true;
    }
  }

  clic_calib::ObservationArchive::LidarBySensor lidar;
  clic_calib::ObservationArchive::AprilTagBySensor apriltag;
  std::vector<clic_calib::RTKMeasurement> rtk;
  if (!LoadData(obs_path, rtk_path, &lidar, &apriltag, &rtk)) {
    std::cerr << "Failed to load observations/RTK\n";
    return 1;
  }

  // Python simulate_uav_trajectory.py ground truth extrinsics.
  const clic_calib::SE3d T_LW_gt(clic_calib::SO3d::rotY(-0.15),
                                 Eigen::Vector3d(0.0, 0.0, 0.5));
  const clic_calib::SE3d T_CW_gt(clic_calib::SO3d::rotX(0.1),
                                 Eigen::Vector3d(2.0, 1.5, 0.2));

  clic_calib::CalibrationEstimator baseline(config_dir);
  PopulateEstimator(&baseline, lidar, apriltag, rtk);
  const RunResult baseline_result = AnalyzeRun(&baseline, 150);
  PrintRun("Baseline (yaml prior mean = initial_T_LW)", baseline_result);

  clic_calib::CalibrationEstimator gt_prior_est(config_dir);
  PopulateEstimator(&gt_prior_est, lidar, apriltag, rtk);
  gt_prior_est.set_extrinsic_prior_T_LW(0, T_LW_gt);
  gt_prior_est.set_extrinsic_prior_T_CW(0, T_CW_gt);
  const RunResult gt_result = AnalyzeRun(&gt_prior_est, 150);
  PrintRun("GT extrinsic prior (simulation truth mean)", gt_result);

  std::cout << "\nGT reference: T_LW.t = " << T_LW_gt.translation().transpose()
            << ", t_d_lidar = 0.030 s, t_d_camera = -0.015 s\n";

  if (gt_prior) {
    std::cout << "(--gt-prior flag noted; GT prior run always executed)\n";
  }
  return 0;
}
