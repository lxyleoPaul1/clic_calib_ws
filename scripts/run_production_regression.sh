#!/usr/bin/env bash
# Production two-stage estimator regression — locks probe-parity numbers.
#
# Usage:
#   cd src/clic_calib
#   ./scripts/run_production_regression.sh          # build (if needed) + run
#   ./scripts/run_production_regression.sh --no-build
#
# Environment:
#   CLIC_UQ_N=100   MC samples for UQ gate (default 100; ~60s)
#
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIN="${ROOT}/build/local_tests"
NO_BUILD=0
if [[ "${1:-}" == "--no-build" ]]; then
  NO_BUILD=1
fi

PASS=0
FAIL=0

run_test() {
  local label="$1"
  local bin="$2"
  shift 2
  if [[ ! -x "${BIN}/${bin}" ]]; then
    echo "[FAIL] ${label} — missing ${BIN}/${bin} (run compile_local_tests.sh)"
    FAIL=$((FAIL + 1))
    return 1
  fi
  echo ""
  echo "==== ${label} ===="
  if "${BIN}/${bin}" "$@" ; then
    echo "[PASS] ${label}"
    PASS=$((PASS + 1))
  else
    echo "[FAIL] ${label}"
    FAIL=$((FAIL + 1))
  fi
}

if [[ "${NO_BUILD}" -eq 0 ]]; then
  echo "==== Building production regression binaries ===="
  "${ROOT}/scripts/compile_local_tests.sh"
fi

export CLIC_UQ_N="${CLIC_UQ_N:-100}"

echo ""
echo "==== Production regression $(date -Iseconds) ===="
echo "Root: ${ROOT}"
echo "CLIC_UQ_N=${CLIC_UQ_N}"

# --- Jacobian / H2 chain ---
run_test "attitude_factor_jacobian (H2 chain)" \
  test_attitude_factor_jacobian

# --- Stage 1 ---
run_test "stage1_trajectory_fitter (probe parity)" \
  test_stage1_trajectory_fitter

# --- Closed-form init ---
run_test "extrinsic_initializer (Umeyama + IPPE)" \
  test_extrinsic_initializer

# --- Config C + FIM assembly (H2 columns) ---
run_test "two_stage_pipeline (Config C + FIM assembly)" \
  test_two_stage_pipeline

# --- UQ decomposition (long) ---
run_test "uq_decomposition (Gate3 + rep FIM)" \
  test_uq_decomposition \
  --gtest_filter='UqDecomposition.Gate3ValidatedDecomposition:UqDecomposition.RepSeedFixedTrajFimDiagnostic:UqDecomposition.FixedTrajMcVsFimAtRep'

# --- Real-data interface contract ---
run_test "real_data_interface (GATE 5)" \
  test_real_data_interface

echo ""
echo "==== Summary: passed=${PASS} failed=${FAIL} ===="
if [[ ${FAIL} -gt 0 ]]; then
  echo "PRODUCTION REGRESSION: FAIL"
  exit 1
fi
echo "PRODUCTION REGRESSION: PASS"
exit 0
