#include <clic_calib/io/calibration_result.h>

#include <cmath>
#include <fstream>
#include <iomanip>

namespace clic_calib {
namespace {

void WriteJsonArray(std::ostream& os, const std::vector<double>& v) {
  os << "[";
  for (size_t i = 0; i < v.size(); ++i) {
    if (i > 0) {
      os << ", ";
    }
    os << v[i];
  }
  os << "]";
}

void WriteSE3Json(std::ostream& os, const SE3d& T) {
  const Eigen::Quaterniond q = T.unit_quaternion();
  const Eigen::Vector3d t = T.translation();
  os << "{"
     << "\"qw\": " << q.w() << ", \"qx\": " << q.x() << ", \"qy\": " << q.y()
     << ", \"qz\": " << q.z() << ", \"tx\": " << t.x() << ", \"ty\": " << t.y()
     << ", \"tz\": " << t.z() << "}";
}

}  // namespace

void CollectProblemResiduals(ceres::Problem& problem,
                           CalibrationResult* result) {
  if (!result) {
    return;
  }
  ceres::Problem::EvaluateOptions opts;
  std::vector<double> residuals;
  problem.Evaluate(opts, &result->final_cost, &residuals, nullptr, nullptr);
  result->residual_values = std::move(residuals);

  double sq = 0.0;
  result->residual_max_abs = 0.0;
  for (double r : result->residual_values) {
    sq += r * r;
    result->residual_max_abs =
        std::max(result->residual_max_abs, std::abs(r));
  }
  if (!result->residual_values.empty()) {
    result->residual_rms =
        std::sqrt(sq / static_cast<double>(result->residual_values.size()));
  }
}

bool WriteCalibrationJson(const CalibrationResult& result,
                          const std::string& path) {
  std::ofstream os(path);
  if (!os) {
    return false;
  }
  os << std::setprecision(16);
  os << "{\n";
  os << "  \"final_cost\": " << result.final_cost << ",\n";
  os << "  \"solver\": {\n";
  os << "    \"initial_cost\": " << result.solver_summary.initial_cost
     << ",\n";
  os << "    \"final_cost\": " << result.solver_summary.final_cost << ",\n";
  os << "    \"iterations\": "
     << result.solver_summary.num_successful_steps << ",\n";
  os << "    \"termination\": \""
     << ceres::TerminationTypeToString(result.solver_summary.termination_type)
     << "\"\n";
  os << "  },\n";

  os << "  \"extrinsics\": {\n";
  os << "    \"lidar\": {\n";
  bool first = true;
  for (const auto& kv : result.T_LW) {
    if (!first) {
      os << ",\n";
    }
    first = false;
    os << "      \"" << kv.first << "\": { \"T_LW\": ";
    WriteSE3Json(os, kv.second);
    const auto td = result.t_d_lidar.find(kv.first);
    os << ", \"t_d_lidar_s\": "
       << (td != result.t_d_lidar.end() ? td->second : 0.0) << " }";
  }
  os << "\n    },\n    \"camera\": {\n";
  first = true;
  for (const auto& kv : result.T_CW) {
    if (!first) {
      os << ",\n";
    }
    first = false;
    os << "      \"" << kv.first << "\": { \"T_CW\": ";
    WriteSE3Json(os, kv.second);
    const auto td = result.t_d_camera.find(kv.first);
    os << ", \"t_d_camera_s\": "
       << (td != result.t_d_camera.end() ? td->second : 0.0) << " }";
  }
  os << "\n    }\n  },\n";

  os << "  \"residuals\": {\n";
  os << "    \"rms\": " << result.residual_rms << ",\n";
  os << "    \"max_abs\": " << result.residual_max_abs << ",\n";
  os << "    \"values\": ";
  WriteJsonArray(os, result.residual_values);
  os << "\n  }\n";
  os << "}\n";
  return static_cast<bool>(os);
}

}  // namespace clic_calib
