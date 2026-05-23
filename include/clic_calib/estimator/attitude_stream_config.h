#pragma once

#include <iosfwd>
#include <string>

namespace clic_calib {

/**
 * Stage-1 PSDK attitude stream settings (loaded from config/spline.yaml).
 *
 * Timestamps share the RTK/world clock. @p transport_delay_s is a constant
 * FC transport delay absorbed jointly with sensor t_d in Stage 2 (default 0).
 */
struct AttitudeStreamConfig {
  /** Subsample 50 Hz PSDK → ~2 Hz attitude factors (synthetic/probe: 25). */
  int stride = 25;
  /**
   * Constant attitude transport delay [s], subtracted from sample timestamps
   * at ingest. Estimate from hover cross-correlation in field; 0 for synthetic.
   */
  double transport_delay_s = 0.0;

  static AttitudeStreamConfig FromSplineYaml(const std::string& path);
  static AttitudeStreamConfig FromConfigDir(const std::string& config_dir);

  void Log(std::ostream& os) const;
};

}  // namespace clic_calib
