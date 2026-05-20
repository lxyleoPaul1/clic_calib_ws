/*
 * clic_calib — offline batch calibration.
 *
 * Usage:
 *   calibrate_offline <config_dir> <obs.clicob> <rtk.csv> [-o calibration.json]
 *                     [--iters N]
 */

#include <clic_calib/estimator/calibration_estimator.h>
#include <clic_calib/io/calibration_result.h>
#include <clic_calib/io/observation_archive.h>
#include <clic_calib/io/rtk_reader.h>

#include <iostream>
#include <string>

namespace {

void PrintUsage(const char* prog) {
  std::cerr << "Usage: " << prog
            << " <config_dir> <observations.clicob> <rtk.csv>"
            << " [-o calibration.json] [--iters N]\n";
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
  std::string output_path = "calibration.json";
  int max_iters = 150;

  for (int i = 4; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "-o" && i + 1 < argc) {
      output_path = argv[++i];
    } else if (arg == "--iters" && i + 1 < argc) {
      max_iters = std::stoi(argv[++i]);
    }
  }

  clic_calib::ObservationArchive::LidarBySensor lidar;
  clic_calib::ObservationArchive::AprilTagBySensor apriltag;
  if (!clic_calib::ObservationArchive::Read(obs_path, &lidar, &apriltag)) {
    std::cerr << "Failed to read observation archive: " << obs_path << "\n";
    return 1;
  }

  clic_calib::CSVReader rtk_reader;
  const std::vector<clic_calib::RTKMeasurement> rtk = rtk_reader.read(rtk_path);
  if (rtk.empty()) {
    std::cerr << "No RTK measurements in: " << rtk_path << "\n";
    return 1;
  }

  clic_calib::CalibrationEstimator estimator(config_dir);
  estimator.add_rtk_measurements(rtk);
  for (const auto& kv : lidar) {
    estimator.add_lidar_target_observations(kv.first, kv.second);
  }
  for (const auto& kv : apriltag) {
    estimator.add_apriltag_observations(kv.first, kv.second);
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
