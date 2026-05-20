#!/usr/bin/env bash
# Run phase regression: factor Jacobians, smoke (noise-free), noise-regime sweeps.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="$(cd "${ROOT}/../.." && pwd)"
LOCAL_BIN="${ROOT}/build/local_tests"

pass=0
fail=0

run_script() {
  local script="$1"
  echo "---- ${script} ----"
  if bash "${ROOT}/scripts/${script}"; then
    pass=$((pass + 1))
  else
    fail=$((fail + 1))
  fi
}

run_bin() {
  local bin="$1"
  if [[ -x "${bin}" ]] && "${bin}"; then
    echo "[PASS] $(basename "${bin}")"
    pass=$((pass + 1))
  else
    echo "[FAIL] $(basename "${bin}")"
    fail=$((fail + 1))
  fi
}

USE_LOCAL=0
if [[ ! -f "${WS}/devel/lib/clic_calib/test_rtk_factor_jacobian" ]] \
   && ! command -v rosrun >/dev/null 2>&1; then
  echo "---- compile_local_tests.sh (no catkin devel) ----"
  bash "${ROOT}/scripts/compile_local_tests.sh"
  USE_LOCAL=1
fi

run_one() {
  local t="$1"
  if [[ "${USE_LOCAL}" -eq 1 ]]; then
    run_bin "${LOCAL_BIN}/${t}"
  elif [[ -f "${WS}/devel/lib/clic_calib/${t}" ]]; then
    run_bin "${WS}/devel/lib/clic_calib/${t}"
  elif [[ -x "${LOCAL_BIN}/${t}" ]]; then
    run_bin "${LOCAL_BIN}/${t}"
  else
    echo "[FAIL] ${t} (not built)"
    fail=$((fail + 1))
  fi
}

echo "==== Factor Jacobian tests ===="
run_script run_factor_tests.sh

echo "==== Smoke (noise-free, strict — fast wiring check) ===="
run_one test_full_pipeline_synthetic

echo "==== Noise-regime characterization (N=20 seed sweeps — paper tables) ===="
for t in test_pipeline_noise_sweep test_patent_z_accuracy test_prior_ablation; do
  run_one "${t}"
done

echo "==== Observability structure (noise-free FIM ablation) ===="
run_one test_observability_synthetic

echo "==== Diagnostics ===="
if [[ "${USE_LOCAL}" -eq 1 ]]; then
  run_one test_td_single_variable
else
  if [[ -x "${LOCAL_BIN}/test_td_single_variable" ]]; then
    run_one test_td_single_variable
  elif [[ -f "${WS}/devel/lib/clic_calib/test_td_single_variable" ]]; then
    run_one test_td_single_variable
  else
    bash "${ROOT}/scripts/compile_local_tests.sh"
    run_one test_td_single_variable
  fi
fi

# Phase 4 detection (optional if built)
if [[ -f "${WS}/devel/lib/clic_calib/test_sphere_extractor" ]]; then
  run_script run_detection_tests.sh
else
  echo "---- run_detection_tests.sh (skipped, not built) ----"
fi

echo "==== Regression summary: passed=${pass} failed=${fail} ===="
echo "Paper numbers: doc/results/synthetic_evaluation.md"
exit "${fail}"
