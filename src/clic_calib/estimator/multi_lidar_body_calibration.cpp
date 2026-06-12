#include <clic_calib/estimator/multi_lidar_body_calibration.h>

#include <clic_calib/estimator/relative_extrinsic.h>
#include <clic_calib/target/body_centroid_analysis.h>

namespace clic_calib {

MultiLidarBodyCalibrationResult CalibrateMultiLidarBodyGated(
    const BodyGatedCalibrationInput& shared_mission,
    const std::vector<LidarBodyCalibrationSpec>& sensors,
    const LeverArmConfig& levers, const NoiseModel& noise,
    const TwoStagePipelineConfig& base_cfg, int observed_mean_iters,
    const ObservedMeanAspectGate& gate,
    const std::vector<double>* temporal_sqrt_info_scales) {
  MultiLidarBodyCalibrationResult out;
  if (sensors.size() < 2) {
    return out;
  }

  std::vector<int> ids;
  ids.reserve(sensors.size());
  for (const auto& spec : sensors) {
    BodyGatedCalibrationInput per_in = shared_mission;
    per_in.body_obs = spec.body_obs;
    per_in.gt_T_LW = spec.gt_T_LW;

    const std::vector<double>* scales = spec.temporal_sqrt_info_scales;
    if (scales == nullptr) {
      scales = temporal_sqrt_info_scales;
    }

    double u_B_azimuth_std_deg = -1.0;
    if (spec.aspect_trajectory != nullptr) {
      u_B_azimuth_std_deg = ComputeUBAzimuthStdDeg(
          *spec.aspect_trajectory, per_in.nominal_t_d_L_s, spec.lidar_post_W,
          spec.body_obs);
    }

    PerSensorBodyCalibResult psr;
    psr.u_B_azimuth_std_deg = u_B_azimuth_std_deg;
    psr.calib = CalibrateBodyGatedObservedMean(
        per_in, levers, noise, base_cfg, observed_mean_iters, gate,
        u_B_azimuth_std_deg, scales);
    out.per_sensor[spec.sensor_id] = psr;
    ids.push_back(spec.sensor_id);
  }

  if (ids.size() < 2) {
    return out;
  }
  const auto& a = out.per_sensor.at(ids[0]).calib;
  const auto& b = out.per_sensor.at(ids[1]).calib;
  const SE3d T_a =
      a.observed_mean_applied ? a.T_LW_observed_mean : a.T_LW_centroid;
  const SE3d T_b =
      b.observed_mean_applied ? b.T_LW_observed_mean : b.T_LW_centroid;
  out.T_rel_observed_mean = ComposeRelativeExtrinsic(T_a, T_b);
  out.T_rel_centroid_only =
      ComposeRelativeExtrinsic(a.T_LW_centroid, b.T_LW_centroid);

  if (out.gt_T_rel) {
    const auto e =
        RelativeExtrinsicErrorVsGt(out.T_rel_observed_mean, *out.gt_T_rel);
    out.rel_rot_deg = e.rot_deg;
    out.rel_trans_mm = e.trans_mm;
  }
  return out;
}

}  // namespace clic_calib
