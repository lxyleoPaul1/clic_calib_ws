#!/usr/bin/env bash
# Run Phase-4 target detection unit tests.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="$(cd "${ROOT}/../.." && pwd)"

TESTS=(
  test_sphere_extractor
  test_apriltag_wrapper
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

if [[ -f "${WS}/devel/lib/clic_calib/test_sphere_extractor" ]]; then
  for t in "${TESTS[@]}"; do
    run_one "${WS}/devel/lib/clic_calib/${t}"
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
  echo "Build tests first: catkin_make -DCATKIN_ENABLE_TESTING=ON"
  exit 1
fi

echo "----"
echo "Passed: ${pass}  Failed: ${fail}"
exit "${fail}"
