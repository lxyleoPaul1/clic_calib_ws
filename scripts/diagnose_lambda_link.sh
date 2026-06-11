#!/usr/bin/env bash
# Reproduce λ_min under (a) minimal vs (b) full EST_SRCS link sets — same inputs.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD="${ROOT}/build/lambda_link_audit"
mkdir -p "${BUILD}"

read -r -a EIGEN_INC_ <<< "$(pkg-config --cflags eigen3 2>/dev/null || true)"
read -r -a OPENCV_CFLAGS_ <<< "$(pkg-config --cflags opencv4 2>/dev/null || true)"
read -r -a OPENCV_LIBS_ <<< "$(pkg-config --libs opencv4 2>/dev/null || true)"

COMMON=(
  -std=c++17 -O2
  "-I${ROOT}/include"
  "-I${ROOT}/src"
  "-I${ROOT}/test"
  "-include" "${ROOT}/include/clic_calib/utils/eigen_utils.hpp"
)
if ((${#EIGEN_INC_[@]})); then COMMON+=("${EIGEN_INC_[@]}"); fi
if ((${#OPENCV_CFLAGS_[@]})); then COMMON+=("${OPENCV_CFLAGS_[@]}"); fi

LIBS=(-lgtest -lceres -lglog -lgflags -lyaml-cpp -lpthread)
if ((${#OPENCV_LIBS_[@]})); then LIBS+=("${OPENCV_LIBS_[@]}"); fi

TRAJ_OBJ="${BUILD}/trajectory.o"
TC_OBJ="${BUILD}/temporal_correlation.o"
if [[ ! -f "${TC_OBJ}" ]]; then
  g++ "${COMMON[@]}" -c "${ROOT}/src/clic_calib/utils/temporal_correlation.cpp" -o "${TC_OBJ}"
fi

CORE=(
  "${ROOT}/src/clic_calib/estimator/calibration_estimator.cpp"
  "${ROOT}/src/clic_calib/estimator/observability_analyzer.cpp"
  "${ROOT}/src/clic_calib/estimator/stage1_trajectory_fitter.cpp"
  "${ROOT}/src/clic_calib/estimator/trajectory_stage.cpp"
  "${ROOT}/src/clic_calib/estimator/extrinsic_initializer.cpp"
  "${ROOT}/src/clic_calib/estimator/extrinsic_refiner.cpp"
  "${ROOT}/src/clic_calib/estimator/attitude_stream_config.cpp"
  "${ROOT}/src/clic_calib/spline/trajectory.cpp"
  "${ROOT}/src/clic_calib/utils/lever_arm.cpp"
  "${ROOT}/src/clic_calib/utils/noise_model.cpp"
  "${TC_OBJ}"
)

EXTRA=(
  "${ROOT}/src/clic_calib/estimator/stage2_extrinsic_fim.cpp"
  "${ROOT}/src/clic_calib/estimator/two_stage_pipeline.cpp"
  "${ROOT}/src/clic_calib/estimator/uq_decomposition.cpp"
  "${ROOT}/src/clic_calib/estimator/real_data_session.cpp"
  "${ROOT}/src/clic_calib/io/rtk_reader.cpp"
  "${ROOT}/src/clic_calib/io/attitude_reader.cpp"
  "${ROOT}/src/clic_calib/io/observation_archive.cpp"
  "${ROOT}/src/clic_calib/io/calibration_result.cpp"
)

build_and_run() {
  local tag="$1"
  shift
  local -a extra_srcs=("$@")
  local out="${BUILD}/obs_${tag}"
  echo "[diagnose] link=${tag} n_src=$(( ${#CORE[@]} + ${#extra_srcs[@]} ))"
  g++ "${COMMON[@]}" "${ROOT}/test/test_observability_synthetic.cpp" \
    "${CORE[@]}" "${extra_srcs[@]}" "${LIBS[@]}" -o "${out}"
  CLIC_FIM_LINK_TAG="${tag}" "${out}" \
    --gtest_filter=ObservabilitySynthetic.FimLinkAuditMulti 2>&1 \
    | grep '\[fim_audit\]'
}

echo "=== git HEAD: $(git -C "${ROOT}" rev-parse --short HEAD) ==="
if ! git -C "${ROOT}" diff --quiet HEAD -- src/clic_calib/io/observation_archive.cpp; then
  echo "[warn] observation_archive.cpp differs from HEAD"
fi

build_and_run "minimal"
build_and_run "full_est_srcs" "${EXTRA[@]}"

for f in "${EXTRA[@]}"; do
  base="$(basename "${f}")"
  extra_one=("${f}")
  case "${base}" in
    uq_decomposition.cpp)
      # Needs stage2_extrinsic_fim symbols at link time.
      extra_one=(
        "${ROOT}/src/clic_calib/estimator/stage2_extrinsic_fim.cpp"
        "${f}"
      )
      ;;
    real_data_session.cpp)
      # Pulls in CSV/attitude/archive readers.
      extra_one=(
        "${ROOT}/src/clic_calib/io/rtk_reader.cpp"
        "${ROOT}/src/clic_calib/io/attitude_reader.cpp"
        "${ROOT}/src/clic_calib/io/observation_archive.cpp"
        "${f}"
      )
      ;;
  esac
  build_and_run "minimal+${base}" "${extra_one[@]}"
done
