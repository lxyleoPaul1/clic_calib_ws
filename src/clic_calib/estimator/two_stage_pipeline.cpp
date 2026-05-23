#include <clic_calib/estimator/two_stage_pipeline.h>

namespace clic_calib {

TwoStagePipelineResult TwoStagePipeline::Run(
    const std::vector<RTKMeasurement>& rtk,
    const std::vector<AttitudeObservation>& attitude,
    const std::vector<LiDARTargetObservation>& lidar,
    const std::vector<AprilTagObservation>& tags,
    const LeverArmConfig& levers, const NoiseModel& noise,
    const TwoStagePipelineConfig& cfg) {
  TwoStagePipelineResult out;

  std::vector<double> bar_times;
  bar_times.reserve(lidar.size() + tags.size());
  for (const auto& scan : lidar) {
    bar_times.push_back(scan.t_sensor_);
  }
  for (const auto& det : tags) {
    bar_times.push_back(det.t_sensor_);
  }

  const Stage1TrajectoryInput s1_input =
      Stage1TrajectoryInput::FromObservationStreams(rtk, attitude, bar_times);
  const Stage1TrajectoryResult s1 =
      Stage1TrajectoryFitter::Fit(s1_input, levers, cfg.stage1);
  out.stage1_summary = s1.summary;
  out.stage1_ok = s1.summary.IsSolutionUsable();
  out.trajectory = s1.trajectory;
  if (!out.stage1_ok || !out.trajectory) {
    return out;
  }

  try {
    out.geometric_init = ExtrinsicInitializer::FromGeometric(
        *out.trajectory, lidar, tags, levers, cfg.init);
  } catch (const std::exception&) {
    return out;
  }

  out.extrinsics = ExtrinsicRefiner::Refine(*out.trajectory, lidar, tags,
                                            levers, noise, out.geometric_init.init,
                                            cfg.refine);
  out.stage2_ok = out.extrinsics.converged;
  return out;
}

}  // namespace clic_calib
