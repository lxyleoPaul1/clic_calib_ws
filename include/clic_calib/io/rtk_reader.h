#pragma once

#include <clic_calib/sensor_data/rtk_measurement.h>

#include <memory>
#include <string>
#include <vector>

namespace clic_calib {

/** @brief Reads RTK/GNSS logs into time-sorted RTKMeasurement vectors. */
class RTKReader {
 public:
  virtual ~RTKReader() = default;

  /** @brief Parse @p path and return measurements sorted by t_world_. */
  virtual std::vector<RTKMeasurement> read(const std::string& path) const = 0;
};

class NMEAReader : public RTKReader {
 public:
  /** @brief If true, use first valid fix as ENU origin (default true). */
  explicit NMEAReader(bool use_first_fix_as_origin = true);

  std::vector<RTKMeasurement> read(const std::string& path) const override;

 private:
  bool use_first_fix_as_origin_;
};

/**
 * @brief DJI M3E text flight-log export (comma-separated).
 *
 * Expected columns (header row optional):
 *   timestamp_utc, latitude_deg, longitude_deg, altitude_m,
 *   fix_type, cn0_dbHz, multipath
 * fix_type: 4=RTK fixed, 5=RTK float, 1=single, 0=invalid.
 */
class DJIDatLogReader : public RTKReader {
 public:
  explicit DJIDatLogReader(bool use_first_fix_as_origin = true);

  std::vector<RTKMeasurement> read(const std::string& path) const override;

 private:
  bool use_first_fix_as_origin_;
};

/**
 * @brief General CSV: timestamp, lat, lon, alt, sigma_n, sigma_e, sigma_u, fix_status.
 *
 * fix_status: FIXED | FLOAT | SINGLE | INVALID (case-insensitive) or 4/5/1/0.
 * Positions converted to local ENU [m] relative to the first row.
 */
class CSVReader : public RTKReader {
 public:
  explicit CSVReader(bool use_first_row_as_origin = true);

  std::vector<RTKMeasurement> read(const std::string& path) const override;

 private:
  bool use_first_row_as_origin_;
};

/** @brief Sort measurements by t_world_ ascending. */
void SortRtkMeasurementsByTime(std::vector<RTKMeasurement>* measurements);

/** @brief Legacy static entry (delegates to DJIDatLogReader). */
class RtkReader {
 public:
  static std::vector<RTKMeasurement> LoadDat(const std::string& path);
};

}  // namespace clic_calib
