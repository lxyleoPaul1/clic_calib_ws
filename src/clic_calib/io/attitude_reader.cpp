#include <clic_calib/io/attitude_reader.h>

#include <clic_calib/utils/sophus_utils.hpp>

#include <fstream>
#include <algorithm>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace clic_calib {
namespace {

bool IsHeaderLine(const std::string& line) {
  return line.find("t_world") != std::string::npos ||
         line.find("timestamp") != std::string::npos ||
         line.find("qw") != std::string::npos ||
         line.find("roll") != std::string::npos;
}

AttitudeObservation ParseAttitudeRow(const std::string& line,
                                     const Eigen::Matrix3d& cov,
                                     double transport_delay_s) {
  std::istringstream iss(line);
  std::string cell;
  std::vector<double> vals;
  while (std::getline(iss, cell, ',')) {
    if (!cell.empty()) {
      vals.push_back(std::stod(cell));
    }
  }
  if (vals.size() < 4) {
    throw std::runtime_error("AttitudeReader: expected ≥4 columns");
  }

  AttitudeObservation obs;
  obs.t_world_ = vals[0] - transport_delay_s;
  obs.covariance_ = cov;

  if (vals.size() >= 5) {
    const Eigen::Quaterniond q(vals[1], vals[2], vals[3], vals[4]);
    obs.R_WB_observed_ = SO3d(q.normalized());
  } else if (vals.size() == 4) {
    constexpr double kDegToRad = M_PI / 180.0;
    const SO3d R =
        SO3d::rotZ(vals[3] * kDegToRad) * SO3d::rotY(vals[2] * kDegToRad) *
        SO3d::rotX(vals[1] * kDegToRad);
    obs.R_WB_observed_ = R;
  } else {
    throw std::runtime_error(
        "AttitudeReader: expected quaternion (t,qw,qx,qy,qz) or euler "
        "(t,roll,pitch,yaw) columns");
  }
  return obs;
}

}  // namespace

AttitudeReader::AttitudeReader() = default;

AttitudeReader::AttitudeReader(const Options& options) : options_(options) {}

std::vector<AttitudeObservation> AttitudeReader::read(
    const std::string& path) const {
  std::ifstream in(path);
  if (!in) {
    throw std::runtime_error("AttitudeReader: cannot open " + path);
  }

  const Eigen::Matrix3d cov = options_.noise_model.AttitudeTangentCovarianceRad2();
  std::vector<AttitudeObservation> out;
  std::string line;
  bool first = true;
  while (std::getline(in, line)) {
    if (line.empty()) {
      continue;
    }
    if (first && IsHeaderLine(line)) {
      first = false;
      continue;
    }
    first = false;
    out.push_back(
        ParseAttitudeRow(line, cov, options_.transport_delay_s));
  }
  SortAttitudeObservationsByTime(&out);
  return out;
}

void SortAttitudeObservationsByTime(std::vector<AttitudeObservation>* obs) {
  if (!obs) {
    return;
  }
  std::sort(obs->begin(), obs->end(),
            [](const AttitudeObservation& a, const AttitudeObservation& b) {
              return a.t_world_ < b.t_world_;
            });
}

}  // namespace clic_calib
