/*
 * clic_calib — offline batch calibration.
 *
 * Usage:
 *   calibrate_offline <config_dir> <obs.clicob> <rtk.csv>
 *                     [--attitude attitude.csv] [-o calibration.json] [--iters N]
 *
 * Real-data streams (interfaces wired; no rosbag parsing in this binary):
 *   RTK CSV          → Stage-1 position
 *   attitude CSV     → Stage-1 PSDK fused attitude (required for two-stage)
 *   observations     → preprocess_rosbag output (sphere centers + AprilTag)
 */

#include <clic_calib/estimator/calibration_estimator.h>
#include <clic_calib/estimator/real_data_session.h>
#include <clic_calib/io/calibration_result.h>

#include <iostream>
#include <string>

namespace {

void PrintUsage(const char* prog) {
  std::cerr << "Usage: " << prog
            << " <config_dir> <observations.clicob> <rtk.csv>"
            << " [--attitude attitude.csv] [-o calibration.json] [--iters N]\n";
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 4) {
    PrintUsage(argv[0]);
    return 1;
  }

  const std::string config_dir = argv[1];
  const std::string obs_path = argv[2];
  const std::string rtk_path = argv[3];
  std::string attitude_path;
  std::string output_path = "calibration.json";
  int max_iters = 150;

  for (int i = 4; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--attitude" && i + 1 < argc) {
      attitude_path = argv[++i];
    } else if (arg == "-o" && i + 1 < argc) {
      output_path = argv[++i];
    } else if (arg == "--iters" && i + 1 < argc) {
      max_iters = std::stoi(argv[++i]);
    }
  }

  const clic_calib::RealDataReadinessReport readiness =
      clic_calib::RealDataSession::CheckReadiness(config_dir);
  clic_calib::RealDataSession::PrintReadinessReport(std::cout, readiness);
  if (!readiness.config_dir_ok) {
    std::cerr << "Config directory incomplete: " << config_dir << "\n";
    return 1;
  }

  clic_calib::CalibrationEstimator estimator(config_dir);

  clic_calib::RealDataPaths paths;
  paths.config_dir = config_dir;
  paths.rtk_csv = rtk_path;
  paths.attitude_csv = attitude_path;
  paths.observations_clicob = obs_path;

  try {
    clic_calib::RealDataSession::WireInto(&estimator, paths);
  } catch (const std::exception& e) {
    std::cerr << "Failed to wire real-data streams: " << e.what() << "\n";
    return 1;
  }

  if (attitude_path.empty()) {
    std::cerr << "Warning: no --attitude CSV; falling back to RTK-only joint "
                 "solve (not valid for field calibration).\n";
  }

  const ceres::Solver::Summary summary = estimator.solve(max_iters);
  if (!summary.IsSolutionUsable()) {
    std::cerr << "Calibration solve did not converge:\n" << summary.FullReport()
              << "\n";
    return 1;
  }

  clic_calib::CalibrationResult result;
  result.solver_summary = summary;
  result.T_LW[0] = estimator.get_T_LW(0);
  result.T_CW[0] = estimator.get_T_CW(0);
  result.t_d_lidar[0] = estimator.get_t_d_lidar(0);
  result.t_d_camera[0] = estimator.get_t_d_camera(0);
  clic_calib::CollectProblemResiduals(estimator.problem(), &result);

  std::cout << "Calibration complete\n"
            << "  final_cost: " << result.final_cost << "\n"
            << "  T_LW.t:     " << result.T_LW[0].translation().transpose()
            << "\n"
            << "  t_d_lidar:  " << result.t_d_lidar[0] << " s\n"
            << "  t_d_camera: " << result.t_d_camera[0] << " s\n";

  if (!clic_calib::WriteCalibrationJson(result, output_path)) {
    std::cerr << "Failed to write JSON: " << output_path << "\n";
    return 1;
  }
  std::cout << "Wrote " << output_path << "\n";
  return 0;
}
