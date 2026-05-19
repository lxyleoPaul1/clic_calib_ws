/*
 * clic_calib — §4.5 Trajectory smoothness regularizer (canonical).
 *
 * R_smooth = alpha_p * sum_k ||ddot_p_WB(t_k_sample)||^2 * dt
 *          + alpha_R * sum_k ||omega_WB(t_k_sample)||^2 * dt
 * Sample at knot midpoints. Defaults alpha_p = alpha_R = 0.01.
 *
 * See doc/DERIVATIONS.md §4.5.
 */

#pragma once

namespace clic_calib {
namespace analytic_derivative {

class TrajectorySmoothnessFactor {};

}  // namespace analytic_derivative
}  // namespace clic_calib
