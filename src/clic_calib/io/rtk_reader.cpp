#include <clic_calib/io/rtk_reader.h>

#include <algorithm>
#include <cmath>
#include <cctype>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace clic_calib {
namespace {

constexpr double kWgs84A = 6378137.0;

struct GeoReference {
  double lat0_deg = 0.0;
  double lon0_deg = 0.0;
  double alt0_m = 0.0;
  bool initialized = false;
};

double DegreesToRadians(double deg) { return deg * M_PI / 180.0; }

Eigen::Vector3d LlaDegToEnu(const GeoReference& ref, double lat_deg, double lon_deg,
                            double alt_m) {
  const double lat0 = DegreesToRadians(ref.lat0_deg);
  const double d_lat = DegreesToRadians(lat_deg - ref.lat0_deg);
  const double d_lon = DegreesToRadians(lon_deg - ref.lon0_deg);
  const double cos_lat0 = std::cos(lat0);
  const double east = d_lon * cos_lat0 * kWgs84A;
  const double north = d_lat * kWgs84A;
  const double up = alt_m - ref.alt0_m;
  return Eigen::Vector3d(east, north, up);
}

void MaybeSetReference(GeoReference* ref, double lat_deg, double lon_deg, double alt_m) {
  if (!ref->initialized) {
    ref->lat0_deg = lat_deg;
    ref->lon0_deg = lon_deg;
    ref->alt0_m = alt_m;
    ref->initialized = true;
  }
}

std::string Trim(const std::string& s) {
  size_t b = 0;
  while (b < s.size() && std::isspace(static_cast<unsigned char>(s[b]))) {
    ++b;
  }
  size_t e = s.size();
  while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) {
    --e;
  }
  return s.substr(b, e - b);
}

std::string ToUpper(std::string s) {
  for (char& c : s) {
    c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
  }
  return s;
}

bool VerifyNmeaChecksum(const std::string& line) {
  const auto star = line.find('*');
  if (star == std::string::npos || star + 3 > line.size()) {
    return true;  // allow lines without checksum in tests
  }
  unsigned char cs = 0;
  for (size_t i = 1; i < star; ++i) {
    cs ^= static_cast<unsigned char>(line[i]);
  }
  const int expected = std::stoi(line.substr(star + 1, 2), nullptr, 16);
  return static_cast<int>(cs) == expected;
}

double ParseNmeaLatitude(const std::string& field, const std::string& hemisphere) {
  if (field.empty()) {
    return 0.0;
  }
  const double raw = std::stod(field);
  const int degrees = static_cast<int>(raw / 100.0);
  const double minutes = raw - degrees * 100.0;
  double lat = degrees + minutes / 60.0;
  if (hemisphere == "S" || hemisphere == "s") {
    lat = -lat;
  }
  return lat;
}

double ParseNmeaLongitude(const std::string& field, const std::string& hemisphere) {
  if (field.empty()) {
    return 0.0;
  }
  const double raw = std::stod(field);
  const int degrees = static_cast<int>(raw / 100.0);
  const double minutes = raw - degrees * 100.0;
  double lon = degrees + minutes / 60.0;
  if (hemisphere == "W" || hemisphere == "w") {
    lon = -lon;
  }
  return lon;
}

RTKMeasurement::FixStatus GgaQualityToFixStatus(int quality) {
  switch (quality) {
    case 4:
    case 7:
      return RTKMeasurement::FixStatus::FIXED;
    case 5:
    case 6:
      return RTKMeasurement::FixStatus::FLOAT;
    case 1:
    case 2:
      return RTKMeasurement::FixStatus::SINGLE;
    default:
      return RTKMeasurement::FixStatus::INVALID;
  }
}

RTKMeasurement::FixStatus ParseFixStatusToken(const std::string& token) {
  const std::string t = ToUpper(Trim(token));
  if (t == "FIXED" || t == "4" || t == "RTK_FIXED") {
    return RTKMeasurement::FixStatus::FIXED;
  }
  if (t == "FLOAT" || t == "5" || t == "RTK_FLOAT") {
    return RTKMeasurement::FixStatus::FLOAT;
  }
  if (t == "SINGLE" || t == "1" || t == "GPS") {
    return RTKMeasurement::FixStatus::SINGLE;
  }
  return RTKMeasurement::FixStatus::INVALID;
}

std::vector<std::string> SplitCsv(const std::string& line) {
  std::vector<std::string> fields;
  std::stringstream ss(line);
  std::string cell;
  while (std::getline(ss, cell, ',')) {
    fields.push_back(Trim(cell));
  }
  return fields;
}

bool ParseGgaLine(const std::string& line, GeoReference* ref, bool use_origin,
                  RTKMeasurement* out) {
  if (line.size() < 6 || line[0] != '$') {
    return false;
  }
  const std::string talker = line.substr(1, 5);
  if (talker != "GNGGA" && talker != "GPGGA" && talker != "GLGGA") {
    return false;
  }
  if (!VerifyNmeaChecksum(line)) {
    return false;
  }

  std::vector<std::string> f;
  {
    std::stringstream ss(line.substr(0, line.find('*')));
    std::string cell;
    while (std::getline(ss, cell, ',')) {
      f.push_back(cell);
    }
  }
  if (f.size() < 10) {
    return false;
  }

  const int quality = f[6].empty() ? 0 : std::stoi(f[6]);
  if (quality == 0) {
    return false;
  }

  const double lat = ParseNmeaLatitude(f[2], f[3]);
  const double lon = ParseNmeaLongitude(f[4], f[5]);
  const double alt = f[9].empty() ? 0.0 : std::stod(f[9]);
  const double hdop = (f.size() > 8 && !f[8].empty()) ? std::stod(f[8]) : 1.0;

  if (use_origin) {
    MaybeSetReference(ref, lat, lon, alt);
  } else if (!ref->initialized) {
    ref->lat0_deg = 0.0;
    ref->lon0_deg = 0.0;
    ref->alt0_m = 0.0;
    ref->initialized = true;
  }

  double t_s = 0.0;
  if (!f[1].empty()) {
    const double hhmmss = std::stod(f[1]);
    const int hh = static_cast<int>(hhmmss / 10000.0);
    const int mm = static_cast<int>((hhmmss - hh * 10000) / 100.0);
    const double ss = hhmmss - hh * 10000 - mm * 100;
    t_s = hh * 3600.0 + mm * 60.0 + ss;
  }

  out->t_world_ = t_s;
  out->p_A_W_observed_ = LlaDegToEnu(*ref, lat, lon, alt);
  const double sigma_h = std::max(0.01, hdop * 0.05);
  const double sigma_v = std::max(0.02, hdop * 0.10);
  out->covariance_.setZero();
  out->covariance_(0, 0) = sigma_h * sigma_h;
  out->covariance_(1, 1) = sigma_h * sigma_h;
  out->covariance_(2, 2) = sigma_v * sigma_v;
  out->fix_status_ = GgaQualityToFixStatus(quality);
  out->cn0_dbHz_ = 0.0;
  out->multipath_metric_ = hdop;
  return true;
}

bool ParseDjiCsvRow(const std::vector<std::string>& f, GeoReference* ref,
                    bool use_origin, RTKMeasurement* out) {
  if (f.size() < 5) {
    return false;
  }
  const double t_s = std::stod(f[0]);
  const double lat = std::stod(f[1]);
  const double lon = std::stod(f[2]);
  const double alt = std::stod(f[3]);
  const int fix_type = std::stoi(f[4]);
  const double cn0 = (f.size() > 5 && !f[5].empty()) ? std::stod(f[5]) : 0.0;
  const double mp = (f.size() > 6 && !f[6].empty()) ? std::stod(f[6]) : 0.0;

  if (use_origin) {
    MaybeSetReference(ref, lat, lon, alt);
  } else if (!ref->initialized) {
    ref->initialized = true;
  }

  out->t_world_ = t_s;
  out->p_A_W_observed_ = LlaDegToEnu(*ref, lat, lon, alt);
  out->covariance_ = Eigen::Matrix3d::Identity() * 0.01;
  out->fix_status_ = GgaQualityToFixStatus(fix_type);
  out->cn0_dbHz_ = cn0;
  out->multipath_metric_ = mp;
  return true;
}

}  // namespace

void SortRtkMeasurementsByTime(std::vector<RTKMeasurement>* measurements) {
  std::sort(measurements->begin(), measurements->end(),
            [](const RTKMeasurement& a, const RTKMeasurement& b) {
              return a.t_world_ < b.t_world_;
            });
}

NMEAReader::NMEAReader(bool use_first_fix_as_origin)
    : use_first_fix_as_origin_(use_first_fix_as_origin) {}

std::vector<RTKMeasurement> NMEAReader::read(const std::string& path) const {
  std::ifstream in(path);
  if (!in) {
    throw std::runtime_error("NMEAReader: cannot open " + path);
  }

  std::vector<RTKMeasurement> out;
  GeoReference ref;
  std::string line;
  while (std::getline(in, line)) {
    line = Trim(line);
    if (line.empty()) {
      continue;
    }
    RTKMeasurement m;
    if (ParseGgaLine(line, &ref, use_first_fix_as_origin_, &m)) {
      out.push_back(m);
    }
  }
  SortRtkMeasurementsByTime(&out);
  return out;
}

DJIDatLogReader::DJIDatLogReader(bool use_first_fix_as_origin)
    : use_first_fix_as_origin_(use_first_fix_as_origin) {}

std::vector<RTKMeasurement> DJIDatLogReader::read(const std::string& path) const {
  std::ifstream in(path);
  if (!in) {
    throw std::runtime_error("DJIDatLogReader: cannot open " + path);
  }

  std::vector<RTKMeasurement> out;
  GeoReference ref;
  std::string line;
  bool header_skipped = false;
  while (std::getline(in, line)) {
    line = Trim(line);
    if (line.empty() || line[0] == '#') {
      continue;
    }
    if (!header_skipped &&
        (line.find("timestamp") != std::string::npos ||
         line.find("latitude") != std::string::npos)) {
      header_skipped = true;
      continue;
    }
    const auto fields = SplitCsv(line);
    RTKMeasurement m;
    if (ParseDjiCsvRow(fields, &ref, use_first_fix_as_origin_, &m)) {
      out.push_back(m);
    }
  }
  SortRtkMeasurementsByTime(&out);
  return out;
}

CSVReader::CSVReader(bool use_first_row_as_origin)
    : use_first_row_as_origin_(use_first_row_as_origin) {}

std::vector<RTKMeasurement> CSVReader::read(const std::string& path) const {
  std::ifstream in(path);
  if (!in) {
    throw std::runtime_error("CSVReader: cannot open " + path);
  }

  std::vector<RTKMeasurement> out;
  GeoReference ref;
  std::string line;
  bool first_line = true;
  while (std::getline(in, line)) {
    line = Trim(line);
    if (line.empty() || line[0] == '#') {
      continue;
    }
    if (first_line && line.find("timestamp") != std::string::npos) {
      first_line = false;
      continue;
    }
    first_line = false;

    const auto f = SplitCsv(line);
    if (f.size() < 8) {
      continue;
    }

    const double t_s = std::stod(f[0]);
    const double lat = std::stod(f[1]);
    const double lon = std::stod(f[2]);
    const double alt = std::stod(f[3]);
    const double sigma_n = std::stod(f[4]);
    const double sigma_e = std::stod(f[5]);
    const double sigma_u = std::stod(f[6]);

    if (use_first_row_as_origin_) {
      MaybeSetReference(&ref, lat, lon, alt);
    } else if (!ref.initialized) {
      ref.initialized = true;
    }

    RTKMeasurement m;
    m.t_world_ = t_s;
    m.p_A_W_observed_ = LlaDegToEnu(ref, lat, lon, alt);
    m.covariance_.setZero();
    m.covariance_(0, 0) = sigma_n * sigma_n;
    m.covariance_(1, 1) = sigma_e * sigma_e;
    m.covariance_(2, 2) = sigma_u * sigma_u;
    m.fix_status_ = ParseFixStatusToken(f[7]);
    m.cn0_dbHz_ = 0.0;
    m.multipath_metric_ = 0.0;
    out.push_back(m);
  }
  SortRtkMeasurementsByTime(&out);
  return out;
}

std::vector<RTKMeasurement> RtkReader::LoadDat(const std::string& path) {
  return DJIDatLogReader().read(path);
}

}  // namespace clic_calib
