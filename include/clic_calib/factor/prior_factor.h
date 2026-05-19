/*
 * clic_calib — §4.6 Extrinsic prior factor (canonical).
 *
 * r^prior_X = Log( T_XW^{-1} * T_XW^prior ),  X in {C, L}
 * Diagonal whitening: ~5 deg rot, ~0.5 m trans (configurable).
 *
 * See doc/DERIVATIONS.md §4.6.
 */

#pragma once

namespace clic_calib {
namespace analytic_derivative {

class ExtrinsicPriorFactor {};

}  // namespace analytic_derivative
}  // namespace clic_calib
