#!/usr/bin/env bash
# Run all Phase-2 factor Jacobian unit tests (requires sourced ROS + catkin workspace).
set -euo pipefail

TESTS=(
  test_rtk_factor_jacobian
  test_sphere_factor_jacobian
  test_apriltag_factor_jacobian
  test_prior_smoothness_jacobian
)

pass=0
fail=0

for t in "${TESTS[@]}"; do
  if rosrun clic_calib "${t}"; then
    echo "[PASS] ${t}"
    pass=$((pass + 1))
  else
    echo "[FAIL] ${t}"
    fail=$((fail + 1))
  fi
done

echo "----"
echo "Passed: ${pass}  Failed: ${fail}"
exit "${fail}"
