/*
 * clic_calib — offline observability analysis (§4.8).
 *
 * Usage:
 *   analyze_observability <config_dir> <obs.clicob> <rtk.csv> [-o report.json]
 */

#include <clic_calib/estimator/calibration_estimator.h>
#include <clic_calib/estimator/observability_analyzer.h>
#include <clic_calib/io/observation_archive.h>
#include <clic_calib/io/rtk_reader.h>

#include <iostream>
#include <string>

namespace {

void PrintUsage(const char* prog) {
  std::cerr << "Usage: " << prog
            << " <config_dir> <observations.clicob> <rtk.csv> [-o report.json]\n";
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
  std::string output_path = "observability_report.json";
  for (int i = 4; i + 1 < argc; ++i) {
    if (std::string(argv[i]) == "-o") {
      output_path = argv[i + 1];
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

  const ceres::Solver::Summary summary = estimator.solve(100);
  if (!summary.IsSolutionUsable()) {
    std::cerr << "Calibration solve did not converge:\n" << summary.FullReport()
              << "\n";
    return 1;
  }

  clic_calib::ObservabilityAnalyzer analyzer;
  const clic_calib::ObservabilityReport report = analyzer.analyze(estimator);

  std::cout << "Observability report\n"
            << "  lambda_min(F_ext): " << report.lambda_min << "\n"
            << "  lambda_max(F_ext): " << report.lambda_max << "\n"
            << "  condition_number:  " << report.condition_number << "\n"
            << "  PDOP_ext:          " << report.pdop_ext << "\n"
            << "  worst_direction:   " << report.worst_direction.transpose()
            << "\n";

  if (!clic_calib::ObservabilityAnalyzer::WriteReportJson(report, output_path)) {
    std::cerr << "Failed to write JSON: " << output_path << "\n";
    return 1;
  }
  std::cout << "Wrote " << output_path << "\n";
  return 0;
}
