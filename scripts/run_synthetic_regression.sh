#!/usr/bin/env bash
# Synthetic phase regression — GREEN GATE set only.
# FIM↔MC tests are diagnostic / future-work (excluded from pass/fail).
#
# Usage:
#   cd src/clic_calib
#   ./scripts/run_synthetic_regression.sh
#
# Requires experiment configs (spline.yaml, noise_model.yaml, lever_arms.yaml).
# Set CLIC_EXPERIMENTS_DIR if not at test/experiments/config_c_nearfield.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIN="${ROOT}/build/local_tests"
PASS=0
FAIL=0
SKIP=0

if [[ ! -d "${BIN}" ]]; then
  echo "ERROR: ${BIN} not found. Build probe tests first."
  exit 1
fi

run_test() {
  local label="$1"
  local bin="$2"
  shift 2
  if [[ ! -x "${BIN}/${bin}" ]]; then
    echo "[SKIP] ${label} — binary ${bin} missing"
    SKIP=$((SKIP + 1))
    return 0
  fi
  echo "---- ${label} ----"
  if "${BIN}/${bin}" "$@" 2>&1 | tail -8; then
    echo "[PASS] ${label}"
    PASS=$((PASS + 1))
  else
    echo "[FAIL] ${label}"
    FAIL=$((FAIL + 1))
  fi
}

echo "==== Synthetic regression GREEN GATE $(date -Iseconds) ===="
echo "Root: ${ROOT}"
echo ""

# --- 1. Factor Jacobians (always required) ---
for t in test_rtk_factor_jacobian \
         test_sphere_factor_jacobian \
         test_apriltag_factor_jacobian \
         test_prior_smoothness_jacobian; do
  run_test "${t}" "${t}"
done

# --- 2. Identifiability λ_min response (fast, needs config) ---
run_test "identifiability_near_vs_200m" test_fim_assembly_audit \
  --gtest_filter='FimAssemblyFix.Gate3FinalRoutingNearFieldVs200m'

# --- 3. Degeneracy coplanar vs multi-layer ---
run_test "degeneracy_reprojection" test_step5_degeneracy_reprojection \
  --gtest_filter='Step5Degeneracy.DistantTargetReprojectionErrorVsRange'

# --- 4. Two-stage validated gates (need full experiment config) ---
run_test "abc_ladder_config_c" test_two_stage_probe \
  --gtest_filter='TwoStageClosedFormInit.Gate3BasinDiscriminationLadder'

run_test "config_c_mc_accuracy" test_two_stage_probe \
  --gtest_filter='TwoStageClosedFormInit.Gate5FimMcClosedFormStage2'

run_test "sigma_yaw_flatness" test_two_stage_probe \
  --gtest_filter='TwoStageClosedFormInit.Gate4YawSensitivitySweep'

# --- 5. §2 UQ validation via noise-source decomposition (N=100) ---
if [[ -x "${BIN}/test_two_stage_probe" ]]; then
  echo "---- uq_noise_decomposition [N=100] ----"
  export CLIC_UQ_N="${CLIC_UQ_N:-100}"
  if "${BIN}/test_two_stage_probe" \
      --gtest_filter='TwoStageClosedFormInit.GateUqNoiseDecompositionMc' 2>&1 | tail -25; then
    echo "[PASS] uq_noise_decomposition"
    PASS=$((PASS + 1))
  else
    echo "[FAIL] uq_noise_decomposition"
    FAIL=$((FAIL + 1))
  fi
fi

# --- DIAGNOSTIC ONLY (run but do not fail suite) ---
echo ""
echo "==== DIAGNOSTIC (known limitation — not green-gate) ===="
for filt in 'TwoStageClosedFormInit.Gate3FimMcSideBySideComparison' \
            'TwoStageClosedFormInit.Gate5MarginalFimMcComparison'; do
  if [[ -x "${BIN}/test_two_stage_probe" ]]; then
    echo "---- ${filt} [informational] ----"
    "${BIN}/test_two_stage_probe" --gtest_filter="${filt}" 2>&1 | tail -15 || true
  fi
done

echo ""
echo "==== Summary: passed=${PASS} failed=${FAIL} skipped=${SKIP} ===="
if [[ ${FAIL} -gt 0 ]]; then
  echo "GREEN GATE: FAIL"
  exit 1
fi
echo "GREEN GATE: PASS (marginal FIM↔MC excluded; UQ via decomposition)"
exit 0
