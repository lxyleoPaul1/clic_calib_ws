#include <clic_calib/io/observation_archive.h>

#include <fstream>
#include <cstring>
#include <stdexcept>

namespace clic_calib {
namespace {

constexpr char kMagic[8] = {'C', 'L', 'I', 'C', 'O', 'B', '0', '1'};
constexpr uint32_t kVersion = 1;

template <typename T>
void WritePod(std::ostream& os, const T& v) {
  os.write(reinterpret_cast<const char*>(&v), sizeof(T));
}

template <typename T>
bool ReadPod(std::istream& is, T* v) {
  is.read(reinterpret_cast<char*>(v), sizeof(T));
  return static_cast<bool>(is);
}

void WriteVector3(std::ostream& os, const Eigen::Vector3d& v) {
  WritePod(os, v.x());
  WritePod(os, v.y());
  WritePod(os, v.z());
}

bool ReadVector3(std::istream& is, Eigen::Vector3d* v) {
  double x, y, z;
  if (!ReadPod(is, &x) || !ReadPod(is, &y) || !ReadPod(is, &z)) {
    return false;
  }
  *v = Eigen::Vector3d(x, y, z);
  return true;
}

void WriteLidarObs(std::ostream& os, const LiDARTargetObservation& obs) {
  WritePod(os, obs.t_sensor_);
  WritePod(os, obs.sensor_id_);
  const uint32_t n = static_cast<uint32_t>(obs.points_L_.size());
  WritePod(os, n);
  for (const auto& p : obs.points_L_) {
    WriteVector3(os, p);
  }
  const uint32_t m = static_cast<uint32_t>(obs.per_point_dt_.size());
  WritePod(os, m);
  for (double dt : obs.per_point_dt_) {
    WritePod(os, dt);
  }
}

bool ReadLidarObs(std::istream& is, LiDARTargetObservation* obs) {
  uint32_t n = 0, m = 0;
  if (!ReadPod(is, &obs->t_sensor_) || !ReadPod(is, &obs->sensor_id_) ||
      !ReadPod(is, &n)) {
    return false;
  }
  obs->points_L_.resize(n);
  for (uint32_t i = 0; i < n; ++i) {
    if (!ReadVector3(is, &obs->points_L_[i])) {
      return false;
    }
  }
  if (!ReadPod(is, &m)) {
    return false;
  }
  obs->per_point_dt_.resize(m);
  for (uint32_t i = 0; i < m; ++i) {
    if (!ReadPod(is, &obs->per_point_dt_[i])) {
      return false;
    }
  }
  return true;
}

void WriteAprilTagObs(std::ostream& os, const AprilTagObservation& obs) {
  WritePod(os, obs.t_sensor_);
  WritePod(os, obs.sensor_id_);
  WritePod(os, obs.tag_id_);
  WritePod(os, obs.detection_confidence_);
  for (const auto& c : obs.corners_pixel_) {
    WritePod(os, c.x());
    WritePod(os, c.y());
  }
}

bool ReadAprilTagObs(std::istream& is, AprilTagObservation* obs) {
  if (!ReadPod(is, &obs->t_sensor_) || !ReadPod(is, &obs->sensor_id_) ||
      !ReadPod(is, &obs->tag_id_) || !ReadPod(is, &obs->detection_confidence_)) {
    return false;
  }
  for (auto& c : obs->corners_pixel_) {
    double x, y;
    if (!ReadPod(is, &x) || !ReadPod(is, &y)) {
      return false;
    }
    c = Eigen::Vector2d(x, y);
  }
  return true;
}

uint32_t CountObservations(const ObservationArchive::LidarBySensor& m) {
  uint32_t c = 0;
  for (const auto& kv : m) {
    c += static_cast<uint32_t>(kv.second.size());
  }
  return c;
}

uint32_t CountObservations(const ObservationArchive::AprilTagBySensor& m) {
  uint32_t c = 0;
  for (const auto& kv : m) {
    c += static_cast<uint32_t>(kv.second.size());
  }
  return c;
}

}  // namespace

bool ObservationArchive::Write(const std::string& path,
                               const LidarBySensor& lidar,
                               const AprilTagBySensor& apriltag) {
  std::ofstream os(path, std::ios::binary);
  if (!os) {
    return false;
  }
  os.write(kMagic, sizeof(kMagic));
  WritePod(os, kVersion);
  WritePod(os, CountObservations(lidar));
  WritePod(os, CountObservations(apriltag));

  for (const auto& kv : lidar) {
    for (const auto& obs : kv.second) {
      WriteLidarObs(os, obs);
    }
  }
  for (const auto& kv : apriltag) {
    for (const auto& obs : kv.second) {
      WriteAprilTagObs(os, obs);
    }
  }
  return static_cast<bool>(os);
}

bool ObservationArchive::Read(const std::string& path, LidarBySensor* lidar,
                              AprilTagBySensor* apriltag) {
  if (!lidar || !apriltag) {
    return false;
  }
  lidar->clear();
  apriltag->clear();

  std::ifstream is(path, std::ios::binary);
  if (!is) {
    return false;
  }

  char magic[8];
  is.read(magic, sizeof(magic));
  if (std::memcmp(magic, kMagic, sizeof(kMagic)) != 0) {
    return false;
  }

  uint32_t version = 0;
  uint32_t num_lidar = 0;
  uint32_t num_tags = 0;
  if (!ReadPod(is, &version) || version != kVersion || !ReadPod(is, &num_lidar) ||
      !ReadPod(is, &num_tags)) {
    return false;
  }

  for (uint32_t i = 0; i < num_lidar; ++i) {
    LiDARTargetObservation obs;
    if (!ReadLidarObs(is, &obs)) {
      return false;
    }
    (*lidar)[obs.sensor_id_].push_back(std::move(obs));
  }
  for (uint32_t i = 0; i < num_tags; ++i) {
    AprilTagObservation obs;
    if (!ReadAprilTagObs(is, &obs)) {
      return false;
    }
    (*apriltag)[obs.sensor_id_].push_back(std::move(obs));
  }
  return static_cast<bool>(is);
}

}  // namespace clic_calib
