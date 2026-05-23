#pragma once

#include <clic_calib/sensor_data/attitude_observation.h>
#include <clic_calib/utils/noise_model.h>

#include <string>
#include <vector>

namespace clic_calib {

/**
 * Reads PSDK fused-attitude logs into @ref AttitudeObservation samples.
 *
 * Expected CSV (header optional):
 *   t_world_s, qw, qx, qy, qz
 * or
 *   t_world_s, roll_deg, pitch_deg, yaw_deg
 *
 * Timestamps must share the RTK/world clock. @ref AttitudeReader applies
 * AttitudeStreamConfig::transport_delay_s (t ← t − delay) before return.
 * Per-sample covariance is filled from @ref NoiseModel::AttitudeTangentCovarianceRad2
 * (Σ_att diagonal from config/noise_model.yaml — replace with hover-measured STD).
 */
class AttitudeReader {
 public:
  struct Options {
    double transport_delay_s = 0.0;
    NoiseModel noise_model;
  };

  explicit AttitudeReader(const Options& options);
  AttitudeReader();

  std::vector<AttitudeObservation> read(const std::string& path) const;

 private:
  Options options_;
};

void SortAttitudeObservationsByTime(std::vector<AttitudeObservation>* obs);

}  // namespace clic_calib
