#!/usr/bin/env bash
# Run all phase regression tests (factor, pipeline, observability, patent, detection).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="$(cd "${ROOT}/../.." && pwd)"

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
  if [[ -x "${bin}" ]]; then
    if "${bin}"; then
      echo "[PASS] $(basename "${bin}")"
      pass=$((pass + 1))
    else
      echo "[FAIL] $(basename "${bin}")"
      fail=$((fail + 1))
    fi
  elif command -v rosrun >/dev/null 2>&1; then
    local name
    name="$(basename "${bin}")"
    if rosrun clic_calib "${name}"; then
      echo "[PASS] ${name}"
      pass=$((pass + 1))
    else
      echo "[FAIL] ${name}"
      fail=$((fail + 1))
    fi
  else
    echo "[SKIP] ${bin} (not built)"
    fail=$((fail + 1))
  fi
}

# Phase 2 factor Jacobian tests
run_script run_factor_tests.sh

# Phase 3/5 pipeline + observability + patent
for t in test_full_pipeline_synthetic test_observability_synthetic test_patent_z_accuracy; do
  run_bin "${WS}/devel/lib/clic_calib/${t}"
done

# Phase 4 detection (optional if built)
if [[ -f "${WS}/devel/lib/clic_calib/test_sphere_extractor" ]]; then
  run_script run_detection_tests.sh
else
  echo "---- run_detection_tests.sh (skipped, not built) ----"
fi

echo "==== Regression summary: passed=${pass} failed=${fail} ===="
exit "${fail}"
