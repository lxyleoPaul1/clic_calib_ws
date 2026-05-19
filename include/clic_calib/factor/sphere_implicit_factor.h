/*
 * clic_calib — §4.3 LiDAR point-to-sphere-surface residual (canonical).
 *
 * t_k^L = bar_t_k^L - t_d^L
 * p_G^L = R_LW * p_G^W(t_k^L) + t_LW
 * r_{k,l}^L = || q_{k,l}^L - p_G^L || - R_ball
 *
 * Whitening: / sigma_r. Robust: Cauchy(1.0).
 * Parameter blocks: T_LW (6), t_d^L (1), 4+4 spline knots.
 * ∂r/∂t_d^L uses analytic dot_p_WB, omega_WB (NOT finite differences).
 *
 * See doc/DERIVATIONS.md §4.3.
 */

#pragma once

namespace clic_calib {
namespace analytic_derivative {

class SphereImplicitFactor {};

}  // namespace analytic_derivative
}  // namespace clic_calib
