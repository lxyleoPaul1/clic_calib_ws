/*
 * clic_calib — §4.2 RTK position residual (canonical).
 *
 * r_k^R = tilde_p_A^k - [ p_WB(t_k^R) + R_WB(t_k^R) * L_{B→A} ]
 * e_k^R = chol(Sigma_k^R)^{-1} * r_k^R
 * NO robust kernel.
 *
 * Parameter blocks: 4x R^3 knots + 4x SO(3) knots adjacent to t_k^R.
 * Jacobians: ∂r/∂p_k = -B_j(u)*I_3; ∂r/∂ξ_k per Sommer CVPR 2020 (right-trivialized).
 *
 * See doc/DERIVATIONS.md §4.2.
 */

#pragma once

#include <clic_calib/spline/spline_segment.h>

#include <ceres/ceres.h>
#include <Eigen/Core>

namespace clic_calib {
namespace analytic_derivative {

class RtkPositionFactor : public ceres::CostFunction {
 public:
  // Phase 1: implement Evaluate + analytic Jacobians exactly per DERIVATIONS.md §4.2.
};

}  // namespace analytic_derivative
}  // namespace clic_calib
