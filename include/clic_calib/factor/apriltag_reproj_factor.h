/*
 * clic_calib — §4.4 AprilTag reprojection residual (canonical).
 *
 * t_k^C = bar_t_k^C - t_d^C
 * p_{M_j}^W = p_WB(t_k^C) + R_WB(t_k^C) * (L_{B→G} + L_{G→M_j})
 * p_{M_j}^C = R_CW * p_{M_j}^W + t_CW
 * u_pred = pi(K * p_{M_j}^C);  r_{k,j}^C = tilde_u_{k,j} - u_pred
 *
 * K fixed. Distortion: radtan or equidistant (OpenCV). Per-marker residuals only.
 * Whitening: / sigma_pix. Robust: Huber(2.0 px).
 *
 * See doc/DERIVATIONS.md §4.4.
 */

#pragma once

namespace clic_calib {
namespace analytic_derivative {

class AprilTagReprojFactor {};

}  // namespace analytic_derivative
}  // namespace clic_calib
