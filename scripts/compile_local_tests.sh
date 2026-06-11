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

OPENCV_LIBS="$(pkg-config --libs opencv4 2>/dev/null || true)"
OPENCV_CFLAGS="$(pkg-config --cflags opencv4 2>/dev/null || true)"
COMMON+=(${OPENCV_CFLAGS})

TRAJ_OBJ="${BUILD_DIR}/trajectory.o"
if [[ ! -f "${TRAJ_OBJ}" ]] || [[ "${ROOT}/src/clic_calib/spline/trajectory.cpp" -nt "${TRAJ_OBJ}" ]]; then
  echo "[compile] trajectory.o"
  g++ "${COMMON[@]}" -c "${ROOT}/src/clic_calib/spline/trajectory.cpp" -o "${TRAJ_OBJ}"
fi

TEMPORAL_CORR_OBJ="${BUILD_DIR}/temporal_correlation.o"
if [[ ! -f "${TEMPORAL_CORR_OBJ}" ]] ||
   [[ "${ROOT}/src/clic_calib/utils/temporal_correlation.cpp" -nt "${TEMPORAL_CORR_OBJ}" ]]; then
  echo "[compile] temporal_correlation.o"
  g++ "${COMMON[@]}" -c "${ROOT}/src/clic_calib/utils/temporal_correlation.cpp" \
    -o "${TEMPORAL_CORR_OBJ}"
fi

EST_SRCS=(
  "${ROOT}/src/clic_calib/estimator/calibration_estimator.cpp"
  "${ROOT}/src/clic_calib/estimator/observability_analyzer.cpp"
  "${ROOT}/src/clic_calib/estimator/stage1_trajectory_fitter.cpp"
  "${ROOT}/src/clic_calib/estimator/trajectory_stage.cpp"
  "${ROOT}/src/clic_calib/estimator/extrinsic_initializer.cpp"
  "${ROOT}/src/clic_calib/estimator/extrinsic_refiner.cpp"
  "${ROOT}/src/clic_calib/estimator/stage2_extrinsic_fim.cpp"
  "${ROOT}/src/clic_calib/estimator/two_stage_pipeline.cpp"
  "${ROOT}/src/clic_calib/estimator/uq_decomposition.cpp"
  "${ROOT}/src/clic_calib/estimator/attitude_stream_config.cpp"
  "${ROOT}/src/clic_calib/estimator/real_data_session.cpp"
  "${ROOT}/src/clic_calib/spline/trajectory.cpp"
  "${ROOT}/src/clic_calib/utils/lever_arm.cpp"
  "${ROOT}/src/clic_calib/utils/noise_model.cpp"
  "${TEMPORAL_CORR_OBJ}"
  "${ROOT}/src/clic_calib/io/rtk_reader.cpp"
  "${ROOT}/src/clic_calib/io/attitude_reader.cpp"
  "${ROOT}/src/clic_calib/io/observation_archive.cpp"
  "${ROOT}/src/clic_calib/io/calibration_result.cpp"
  "${ROOT}/src/clic_calib/io/sensor_rig_config.cpp"
  "${ROOT}/src/clic_calib/config/body_model_config.cpp"
  "${ROOT}/src/clic_calib/config/body_cluster_detection_config.cpp"
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
  test_attitude_factor_jacobian
  test_body_centroid_factor_jacobian
  test_fixed_traj_body_centroid_factor_jacobian
  test_fixed_traj_body_centroid_joint_lever_factor_jacobian
)

for t in "${FACTOR_TESTS[@]}"; do
  extra=("${TRAJ_OBJ}")
  if [[ "${t}" == test_fixed_traj_body_centroid_factor_jacobian ]] ||
     [[ "${t}" == test_fixed_traj_body_centroid_joint_lever_factor_jacobian ]]; then
    extra+=("${ROOT}/src/clic_calib/utils/noise_model.cpp")
  fi
  build_one "${t}" "${ROOT}/test/${t}.cpp" "${extra[@]}"
done

EST_TESTS=(
  test_full_pipeline_synthetic
  test_pipeline_noise_sweep
  test_observability_synthetic
  test_patent_z_accuracy
  test_prior_ablation
)

for t in "${EST_TESTS[@]}"; do
  build_one "${t}" "${ROOT}/test/${t}.cpp" "${EST_SRCS[@]}" ${OPENCV_LIBS}
done

build_one test_td_single_variable \
  "${ROOT}/test/diagnostic/test_td_single_variable.cpp" \
  "${TRAJ_OBJ}" \
  "${ROOT}/src/clic_calib/utils/lever_arm.cpp"

build_one test_noise_regime_local_sweep \
  "${ROOT}/test/experiments/test_noise_regime_local_sweep.cpp" \
  "${EST_SRCS[@]}" ${OPENCV_LIBS}

build_one test_stage1_trajectory_fitter \
  "${ROOT}/test/test_stage1_trajectory_fitter.cpp" \
  "${TRAJ_OBJ}" \
  "${ROOT}/src/clic_calib/estimator/stage1_trajectory_fitter.cpp" \
  "${ROOT}/src/clic_calib/utils/lever_arm.cpp" \
  "${ROOT}/src/clic_calib/utils/noise_model.cpp" \
  "${TEMPORAL_CORR_OBJ}"

build_one test_extrinsic_initializer \
  "${ROOT}/test/test_extrinsic_initializer.cpp" \
  "${TRAJ_OBJ}" \
  "${ROOT}/src/clic_calib/estimator/stage1_trajectory_fitter.cpp" \
  "${ROOT}/src/clic_calib/estimator/extrinsic_initializer.cpp" \
  "${ROOT}/src/clic_calib/utils/lever_arm.cpp" \
  "${ROOT}/src/clic_calib/utils/noise_model.cpp" \
  "${TEMPORAL_CORR_OBJ}" \
  ${OPENCV_LIBS}

build_one test_two_stage_pipeline \
  "${ROOT}/test/test_two_stage_pipeline.cpp" \
  "${TRAJ_OBJ}" \
  "${ROOT}/src/clic_calib/estimator/stage1_trajectory_fitter.cpp" \
  "${ROOT}/src/clic_calib/estimator/extrinsic_initializer.cpp" \
  "${ROOT}/src/clic_calib/estimator/extrinsic_refiner.cpp" \
  "${ROOT}/src/clic_calib/estimator/stage2_extrinsic_fim.cpp" \
  "${ROOT}/src/clic_calib/estimator/two_stage_pipeline.cpp" \
  "${ROOT}/src/clic_calib/utils/lever_arm.cpp" \
  "${ROOT}/src/clic_calib/utils/noise_model.cpp" \
  "${TEMPORAL_CORR_OBJ}" \
  ${OPENCV_LIBS}

PHASE15_SRCS=(
  "${ROOT}/src/clic_calib/target/body_centroid_analysis.cpp"
  "${ROOT}/src/clic_calib/target/drone_model_registration.cpp"
)

build_one test_board_free_e2e_comparison \
  "${ROOT}/test/test_board_free_e2e_comparison.cpp" \
  "${TRAJ_OBJ}" \
  "${ROOT}/src/clic_calib/estimator/stage1_trajectory_fitter.cpp" \
  "${ROOT}/src/clic_calib/estimator/extrinsic_initializer.cpp" \
  "${ROOT}/src/clic_calib/estimator/extrinsic_refiner.cpp" \
  "${ROOT}/src/clic_calib/estimator/two_stage_pipeline.cpp" \
  "${ROOT}/src/clic_calib/utils/lever_arm.cpp" \
  "${ROOT}/src/clic_calib/utils/noise_model.cpp" \
  "${TEMPORAL_CORR_OBJ}" \
  ${OPENCV_LIBS}

build_one test_phase15_main_table \
  "${ROOT}/test/test_phase15_main_table.cpp" \
  "${TRAJ_OBJ}" \
  "${ROOT}/src/clic_calib/estimator/stage1_trajectory_fitter.cpp" \
  "${ROOT}/src/clic_calib/estimator/extrinsic_initializer.cpp" \
  "${ROOT}/src/clic_calib/estimator/extrinsic_refiner.cpp" \
  "${ROOT}/src/clic_calib/estimator/two_stage_pipeline.cpp" \
  "${ROOT}/src/clic_calib/config/body_model_config.cpp" \
  "${PHASE15_SRCS[@]}" \
  "${ROOT}/src/clic_calib/utils/lever_arm.cpp" \
  "${ROOT}/src/clic_calib/utils/noise_model.cpp" \
  "${TEMPORAL_CORR_OBJ}" \
  ${OPENCV_LIBS}

build_one test_phase3_dual_lidar_experiment_a \
  "${ROOT}/test/test_phase3_dual_lidar_experiment_a.cpp" \
  "${TRAJ_OBJ}" \
  "${ROOT}/src/clic_calib/estimator/stage1_trajectory_fitter.cpp" \
  "${ROOT}/src/clic_calib/estimator/extrinsic_initializer.cpp" \
  "${ROOT}/src/clic_calib/estimator/extrinsic_refiner.cpp" \
  "${ROOT}/src/clic_calib/estimator/two_stage_pipeline.cpp" \
  "${ROOT}/src/clic_calib/target/body_centroid_analysis.cpp" \
  "${ROOT}/src/clic_calib/utils/lever_arm.cpp" \
  "${ROOT}/src/clic_calib/utils/noise_model.cpp" \
  "${TEMPORAL_CORR_OBJ}" \
  ${OPENCV_LIBS}

build_one test_phase15_joint_opt_audit \
  "${ROOT}/test/test_phase15_joint_opt_audit.cpp" \
  "${TRAJ_OBJ}" \
  "${ROOT}/src/clic_calib/estimator/stage1_trajectory_fitter.cpp" \
  "${ROOT}/src/clic_calib/estimator/extrinsic_initializer.cpp" \
  "${ROOT}/src/clic_calib/estimator/extrinsic_refiner.cpp" \
  "${ROOT}/src/clic_calib/utils/lever_arm.cpp" \
  "${ROOT}/src/clic_calib/utils/noise_model.cpp" \
  "${TEMPORAL_CORR_OBJ}" \
  ${OPENCV_LIBS}

build_one test_phase15_contradiction_audit \
  "${ROOT}/test/test_phase15_contradiction_audit.cpp" \
  "${TRAJ_OBJ}" \
  "${ROOT}/src/clic_calib/estimator/stage1_trajectory_fitter.cpp" \
  "${ROOT}/src/clic_calib/estimator/extrinsic_initializer.cpp" \
  "${ROOT}/src/clic_calib/estimator/extrinsic_refiner.cpp" \
  "${ROOT}/src/clic_calib/estimator/two_stage_pipeline.cpp" \
  "${ROOT}/src/clic_calib/config/body_model_config.cpp" \
  "${PHASE15_SRCS[@]}" \
  "${ROOT}/src/clic_calib/utils/lever_arm.cpp" \
  "${ROOT}/src/clic_calib/utils/noise_model.cpp" \
  "${TEMPORAL_CORR_OBJ}" \
  ${OPENCV_LIBS}

build_one test_phase15_ablation \
  "${ROOT}/test/test_phase15_ablation.cpp" \
  "${TRAJ_OBJ}" \
  "${ROOT}/src/clic_calib/estimator/stage1_trajectory_fitter.cpp" \
  "${ROOT}/src/clic_calib/estimator/extrinsic_initializer.cpp" \
  "${ROOT}/src/clic_calib/estimator/extrinsic_refiner.cpp" \
  "${ROOT}/src/clic_calib/estimator/two_stage_pipeline.cpp" \
  "${ROOT}/src/clic_calib/config/body_model_config.cpp" \
  "${PHASE15_SRCS[@]}" \
  "${ROOT}/src/clic_calib/utils/lever_arm.cpp" \
  "${ROOT}/src/clic_calib/utils/noise_model.cpp" \
  "${TEMPORAL_CORR_OBJ}" \
  ${OPENCV_LIBS}

build_one test_uq_decomposition \
  "${ROOT}/test/test_uq_decomposition.cpp" \
  "${EST_SRCS[@]}" \
  ${OPENCV_LIBS}

build_one test_clicob_v2_roundtrip \
  "${ROOT}/test/test_clicob_v2_roundtrip.cpp" \
  "${ROOT}/src/clic_calib/io/observation_archive.cpp" \
  "${ROOT}/src/clic_calib/io/sensor_rig_config.cpp" \
  "${ROOT}/src/clic_calib/config/body_model_config.cpp" \
  "${ROOT}/src/clic_calib/config/body_cluster_detection_config.cpp" \
  "${ROOT}/src/clic_calib/utils/lever_arm.cpp" \
  "${ROOT}/src/clic_calib/utils/noise_model.cpp"

build_one test_clicob_v1_backward_compat \
  "${ROOT}/test/test_clicob_v1_backward_compat.cpp" \
  "${ROOT}/src/clic_calib/io/observation_archive.cpp"

build_one test_body_cluster_extractor \
  "${ROOT}/test/test_body_cluster_extractor.cpp" \
  "${TRAJ_OBJ}" \
  "${ROOT}/src/clic_calib/target/body_cluster_extractor.cpp" \
  "${ROOT}/src/clic_calib/target/drone_model_registration.cpp" \
  "${ROOT}/src/clic_calib/config/body_model_config.cpp" \
  "${ROOT}/src/clic_calib/config/body_cluster_detection_config.cpp" \
  "${ROOT}/src/clic_calib/utils/lever_arm.cpp" \
  "${ROOT}/src/clic_calib/utils/noise_model.cpp"

build_one test_real_data_interface \
  "${ROOT}/test/test_real_data_interface.cpp" \
  "${EST_SRCS[@]}" \
  ${OPENCV_LIBS}

echo "[compile] test_two_stage_probe (probe-only, needs OpenCV)"
g++ "${COMMON[@]}" \
  "${ROOT}/test/diagnostic/test_two_stage_probe.cpp" \
  "${ROOT}/src/clic_calib/spline/trajectory.cpp" \
  "${ROOT}/src/clic_calib/utils/lever_arm.cpp" \
  "${ROOT}/src/clic_calib/utils/noise_model.cpp" \
  "${LIBS[@]}" ${OPENCV_LIBS} \
  -o "${BUILD_DIR}/test_two_stage_probe"

echo "[compile] done → ${BUILD_DIR}/"
echo "[hint] Production parity gate: ./scripts/run_production_regression.sh"

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
