#!/usr/bin/env bash
# Build clic_calib GTests with g++ when catkin/devel is unavailable (WSL / no ROS).
#
# Usage:
#   scripts/compile_local_tests.sh          # build all
#   scripts/compile_local_tests.sh --run    # build then execute
#
# Requires: g++, pkg-config (eigen3), libceres, libgtest, libglog, libgflags,
#           libyaml-cpp, libpthread.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${ROOT}/build/local_tests"
RUN_AFTER=0
if [[ "${1:-}" == "--run" ]]; then
  RUN_AFTER=1
fi

mkdir -p "${BUILD_DIR}"

EIGEN_INC="$(pkg-config --cflags eigen3 2>/dev/null || true)"
COMMON=(
  -std=c++17 -O2
  "-I${ROOT}/include"
  "-I${ROOT}/src"
  "-I${ROOT}/test"
  "-include" "${ROOT}/include/clic_calib/utils/eigen_utils.hpp"
  ${EIGEN_INC}
)

LIBS=(-lgtest -lceres -lglog -lgflags -lyaml-cpp -lpthread)

TRAJ_OBJ="${BUILD_DIR}/trajectory.o"
if [[ ! -f "${TRAJ_OBJ}" ]] || [[ "${ROOT}/src/clic_calib/spline/trajectory.cpp" -nt "${TRAJ_OBJ}" ]]; then
  echo "[compile] trajectory.o"
  g++ "${COMMON[@]}" -c "${ROOT}/src/clic_calib/spline/trajectory.cpp" -o "${TRAJ_OBJ}"
fi

EST_SRCS=(
  "${ROOT}/src/clic_calib/estimator/calibration_estimator.cpp"
  "${ROOT}/src/clic_calib/estimator/observability_analyzer.cpp"
  "${ROOT}/src/clic_calib/spline/trajectory.cpp"
  "${ROOT}/src/clic_calib/utils/lever_arm.cpp"
  "${ROOT}/src/clic_calib/io/rtk_reader.cpp"
  "${ROOT}/src/clic_calib/io/observation_archive.cpp"
  "${ROOT}/src/clic_calib/io/calibration_result.cpp"
)

build_one() {
  local name="$1"
  local src="$2"
  shift 2
  local extra=("$@")
  echo "[compile] ${name}"
  g++ "${COMMON[@]}" "${src}" "${extra[@]}" "${LIBS[@]}" -o "${BUILD_DIR}/${name}"
}

FACTOR_TESTS=(
  test_rtk_factor_jacobian
  test_sphere_factor_jacobian
  test_apriltag_factor_jacobian
  test_prior_smoothness_jacobian
)

for t in "${FACTOR_TESTS[@]}"; do
  build_one "${t}" "${ROOT}/test/${t}.cpp" "${TRAJ_OBJ}"
done

EST_TESTS=(
  test_full_pipeline_synthetic
  test_observability_synthetic
  test_patent_z_accuracy
)

for t in "${EST_TESTS[@]}"; do
  build_one "${t}" "${ROOT}/test/${t}.cpp" "${EST_SRCS[@]}"
done

build_one test_td_single_variable \
  "${ROOT}/test/diagnostic/test_td_single_variable.cpp" \
  "${TRAJ_OBJ}" \
  "${ROOT}/src/clic_calib/utils/lever_arm.cpp"

echo "[compile] done → ${BUILD_DIR}/"

if [[ "${RUN_AFTER}" -eq 1 ]]; then
  pass=0
  fail=0
  for t in "${FACTOR_TESTS[@]}" "${EST_TESTS[@]}" test_td_single_variable; do
    if "${BUILD_DIR}/${t}"; then
      echo "[PASS] ${t}"
      pass=$((pass + 1))
    else
      echo "[FAIL] ${t}"
      fail=$((fail + 1))
    fi
  done
  echo "---- Passed: ${pass}  Failed: ${fail} ----"
  exit "${fail}"
fi
