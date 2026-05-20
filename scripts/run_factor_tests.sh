#!/usr/bin/env bash
# Run Phase-2 factor Jacobian unit tests.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="$(cd "${ROOT}/../.." && pwd)"

TESTS=(
  test_rtk_factor_jacobian
  test_sphere_factor_jacobian
  test_apriltag_factor_jacobian
  test_prior_smoothness_jacobian
)

pass=0
fail=0

run_one() {
  local bin="$1"
  if "${bin}"; then
    echo "[PASS] $(basename "${bin}")"
    pass=$((pass + 1))
  else
    echo "[FAIL] $(basename "${bin}")"
    fail=$((fail + 1))
  fi
}

if [[ -f "${WS}/devel/lib/clic_calib/test_rtk_factor_jacobian" ]]; then
  for t in "${TESTS[@]}"; do
    run_one "${WS}/devel/lib/clic_calib/${t}"
  done
elif [[ -x "${ROOT}/build/local_tests/test_rtk_factor_jacobian" ]]; then
  for t in "${TESTS[@]}"; do
    run_one "${ROOT}/build/local_tests/${t}"
  done
elif command -v rosrun >/dev/null 2>&1; then
  for t in "${TESTS[@]}"; do
    if rosrun clic_calib "${t}"; then
      echo "[PASS] ${t}"
      pass=$((pass + 1))
    else
      echo "[FAIL] ${t}"
      fail=$((fail + 1))
    fi
  done
else
  echo "Build tests first: scripts/compile_local_tests.sh"
  exit 1
fi

echo "----"
echo "Passed: ${pass}  Failed: ${fail}"
exit "${fail}"
