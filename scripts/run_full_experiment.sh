#!/usr/bin/env bash
# End-to-end experiment: synthetic data (default) or rosbag preprocess → calibrate → analyze → PDF.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="$(cd "${ROOT}/../.." && pwd)"
CONFIG="${ROOT}/config"

usage() {
  cat <<EOF
Usage: $(basename "$0") [OPTIONS] [OUTPUT_DIR]

Run full clic_calib offline pipeline and write experiment_report.pdf.

Options:
  --synthetic          Generate synthetic obs.clicob + rtk.csv (default)
  --coplanar         Coplanar synthetic flight (ablation)
  --range-m M        Standoff distance in metres (default 200)
  --rosbag PATH      Preprocess rosbag instead of synthetic data
  --config DIR       Config directory (default: package config/)
  -h, --help         Show this help

OUTPUT_DIR defaults to /tmp/clic_experiment
EOF
}

OUT_DIR="/tmp/clic_experiment"
MODE="synthetic"
RANGE_M="200"
BAG_PATH=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --synthetic) MODE="synthetic"; shift ;;
    --coplanar) MODE="coplanar"; shift ;;
    --range-m) RANGE_M="$2"; shift 2 ;;
    --rosbag) MODE="rosbag"; BAG_PATH="$2"; shift 2 ;;
    --config) CONFIG="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    -*) echo "Unknown option: $1" >&2; usage; exit 1 ;;
    *) OUT_DIR="$1"; shift ;;
  esac
done

mkdir -p "${OUT_DIR}"

find_bin() {
  local name="$1"
  if [[ -n "${CLIC_CALIB_BIN_DIR:-}" && -x "${CLIC_CALIB_BIN_DIR}/${name}" ]]; then
    echo "${CLIC_CALIB_BIN_DIR}/${name}"
  elif [[ -x "${WS}/devel/lib/clic_calib/${name}" ]]; then
    echo "${WS}/devel/lib/clic_calib/${name}"
  elif [[ -x "/tmp/${name}" ]]; then
    echo "/tmp/${name}"
  elif command -v rosrun >/dev/null 2>&1; then
    echo "rosrun clic_calib ${name}"
  else
    echo ""
  fi
}

CALIBRATE="$(find_bin calibrate_offline)"
ANALYZE="$(find_bin analyze_observability)"
PREPROCESS="$(find_bin preprocess_rosbag)"

if [[ -z "${CALIBRATE}" || -z "${ANALYZE}" ]]; then
  echo "Build clic_calib first: cd ${WS} && catkin_make --pkg clic_calib" >&2
  exit 1
fi

echo "==> Output directory: ${OUT_DIR}"
echo "==> Config: ${CONFIG}"

case "${MODE}" in
  synthetic)
    echo "==> Generating synthetic multi-layer data (range=${RANGE_M} m)"
    python3 "${ROOT}/scripts/simulate_uav_trajectory.py" \
      --output-dir "${OUT_DIR}" --range-m "${RANGE_M}" --multilayer
    ;;
  coplanar)
    echo "==> Generating synthetic coplanar ablation (range=${RANGE_M} m)"
    python3 "${ROOT}/scripts/simulate_uav_trajectory.py" \
      --output-dir "${OUT_DIR}" --range-m "${RANGE_M}" --coplanar --no-camera
    ;;
  rosbag)
    if [[ -z "${PREPROCESS}" ]]; then
      echo "preprocess_rosbag not found; build with rosbag support" >&2
      exit 1
    fi
    if [[ ! -f "${BAG_PATH}" ]]; then
      echo "Rosbag not found: ${BAG_PATH}" >&2
      exit 1
    fi
    echo "==> Preprocessing rosbag: ${BAG_PATH}"
    ${PREPROCESS} "${CONFIG}" "${BAG_PATH}" \
      -o "${OUT_DIR}/obs.clicob" --rtk-csv "${OUT_DIR}/rtk.csv"
    ;;
esac

OBS="${OUT_DIR}/obs.clicob"
RTK="${OUT_DIR}/rtk.csv"
CAL_JSON="${OUT_DIR}/calibration.json"
OBS_JSON="${OUT_DIR}/observability.json"

if [[ ! -f "${OBS}" || ! -f "${RTK}" ]]; then
  echo "Missing observations or RTK in ${OUT_DIR}" >&2
  exit 1
fi

echo "==> Running calibrate_offline"
${CALIBRATE} "${CONFIG}" "${OBS}" "${RTK}" -o "${CAL_JSON}"

echo "==> Running analyze_observability"
${ANALYZE} "${CONFIG}" "${OBS}" "${RTK}" -o "${OBS_JSON}"

echo "==> Plotting residuals"
python3 "${ROOT}/scripts/plot_residuals.py" "${CAL_JSON}" \
  -o "${OUT_DIR}/residuals.png"

echo "==> Visualizing FIM"
python3 "${ROOT}/scripts/fim_visualizer.py" "${OBS_JSON}" \
  -o "${OUT_DIR}/fim.png"

echo "==> Generating experiment report PDF"
python3 "${ROOT}/scripts/generate_experiment_report.py" "${OUT_DIR}" \
  -o "${OUT_DIR}/experiment_report.pdf"

echo "Done. Report: ${OUT_DIR}/experiment_report.pdf"
