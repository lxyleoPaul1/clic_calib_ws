#pragma once

#include <clic_calib/target/body_centroid_analysis.h>

#include <iomanip>
#include <iostream>

namespace clic_calib {
namespace experiments {

inline void PrintAspectBiasScatterReport(const char* sensor_tag,
                                         const AspectBiasScatterReport& r) {
  std::cout << std::fixed << std::setprecision(3);
  std::cout << "  [" << sensor_tag << "] u_B az std=" << r.u_B_azimuth_std_deg
            << " deg  varying_RMS=" << r.bias_varying_rms_mm << " mm\n";
  std::cout << "    r(u_B_az,bias): x=" << r.corr_u_az_bias_x
            << " y=" << r.corr_u_az_bias_y << " z=" << r.corr_u_az_bias_z
            << "\n";
}

inline void PrintTidalLockAudit(const char* label,
                                const TidalLockAudit& a) {
  std::cout << std::fixed << std::setprecision(3);
  std::cout << "  [" << label << "] yaw-orbit std="
            << a.yaw_minus_orbit_std_deg << " deg  orbit_az↔yaw r="
            << a.corr_orbit_azimuth_body_yaw << "  locked="
            << (a.tidal_locked ? "YES" : "NO") << "\n";
}

}  // namespace experiments
}  // namespace clic_calib
