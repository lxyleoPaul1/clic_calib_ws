#include <clic_calib/estimator/trajectory_stage.h>

namespace clic_calib {

TrajectoryStageResult TrajectoryStage::Estimate(
    const std::vector<RTKMeasurement>& rtk,
    const std::vector<AttitudeObservation>& attitude,
    const LeverArmConfig& levers, const TrajectoryStageConfig& cfg,
    double t_obs_lo, double t_obs_hi) {
  Stage1TrajectoryInput input;
  input.rtk = rtk;
  input.attitude = attitude;
  input.t_obs_lo = t_obs_lo;
  input.t_obs_hi = t_obs_hi;

  Stage1TrajectoryConfig s1_cfg;
  s1_cfg.knot_interval_s = cfg.knot_interval_s;
  s1_cfg.alpha_p = cfg.alpha_p;
  s1_cfg.alpha_R = cfg.alpha_R;
  s1_cfg.attitude_stride = cfg.attitude_stride;

  const Stage1TrajectoryResult r =
      Stage1TrajectoryFitter::Fit(input, levers, s1_cfg);
  TrajectoryStageResult out;
  out.trajectory = r.trajectory;
  out.summary = r.summary;
  return out;
}

}  // namespace clic_calib
