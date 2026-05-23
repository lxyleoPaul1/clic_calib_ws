#pragma once

#include <clic_calib/sensor_data/attitude_observation.h>
#include <clic_calib/sensor_data/rtk_measurement.h>
#include <clic_calib/spline/trajectory.h>
#include <clic_calib/utils/lever_arm.h>

#include <ceres/ceres.h>

#include <memory>
#include <vector>

namespace clic_calib {

struct Stage1TrajectoryConfig {
  double knot_interval_s = 0.05;
  double alpha_p = 0.01;
  double alpha_R = 0.01;
  /** Subsample 50 Hz PSDK stream → ~2 Hz factors (probe stride = 25). */
  int attitude_stride = 25;
  /** H3: drop knots outside observation support after solve. */
  bool trim_to_observation_support = true;
};

struct Stage1TrajectoryResult {
  std::shared_ptr<BodyTrajectory> trajectory;
  ceres::Solver::Summary summary;
};

/**
 * Real-data / synthetic Stage-1 inputs (same schema as probe scenario bundles).
 * Time window defines knot init span and post-solve trim support.
 */
struct Stage1TrajectoryInput {
  std::vector<RTKMeasurement> rtk;
  std::vector<AttitudeObservation> attitude;
  double t_obs_lo = 0.0;
  double t_obs_hi = 0.0;

  /** Probe convention: RTK time span for Stage-1 init (attitude usually inside). */
  static Stage1TrajectoryInput FromRtkAttitudeStreams(
      std::vector<RTKMeasurement> rtk,
      std::vector<AttitudeObservation> attitude);

  /** Union of RTK + attitude + optional extra sensor bar times. */
  static Stage1TrajectoryInput FromObservationStreams(
      const std::vector<RTKMeasurement>& rtk,
      const std::vector<AttitudeObservation>& attitude,
      const std::vector<double>& extra_bar_times_s = {});
};

/**
 * Stage-1 trajectory fit: anisotropic PSDK attitude + RTK (three-pass decoupled)
 * + light smoothness. Port of probe FitStage1WithAttitude().
 */
class Stage1TrajectoryFitter {
 public:
  static Stage1TrajectoryResult Fit(const Stage1TrajectoryInput& input,
                                    const LeverArmConfig& levers,
                                    const Stage1TrajectoryConfig& cfg);
};

}  // namespace clic_calib
