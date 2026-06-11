#!/usr/bin/env bash
# Frozen-baseline isolation audit: each key metric must match between
# full-binary run (suite) and single-test --gtest_filter (isolated).
# Missing binaries, subprocess failure, or unparseable metrics => exit 1.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BD="${ROOT}/build/local_tests"

REQUIRED_BINS=(
  test_observability_synthetic
  test_two_stage_pipeline
  test_phase3_dual_lidar_experiment_a
  test_phase3_dual_lidar_phase_b
  test_phase15_main_table
)

for b in "${REQUIRED_BINS[@]}"; do
  if [[ ! -x "${BD}/${b}" ]]; then
    echo "[isolation] FAIL: required binary missing or not executable: ${BD}/${b}"
    echo "[isolation] Run: bash scripts/compile_local_tests.sh"
    exit 1
  fi
done

TMP="$(mktemp -d)"
trap 'rm -rf "${TMP}"' EXIT

python3 - "${TMP}" "${BD}" <<'PY'
import re, sys, subprocess
from pathlib import Path

tmp = Path(sys.argv[1])
bd = Path(sys.argv[2])

def run_suite(bin_name: str) -> str:
    p = tmp / f"{bin_name}_suite.log"
    subprocess.run([str(bd / bin_name)], check=True, stdout=p.open("w"), stderr=subprocess.STDOUT)
    return p.read_text()

def run_iso(bin_name: str, filt: str) -> str:
    p = tmp / f"{bin_name}_iso.log"
    subprocess.run([str(bd / bin_name), f"--gtest_filter={filt}"],
                   check=True, stdout=p.open("w"), stderr=subprocess.STDOUT)
    return p.read_text()

def near(a: float, b: float, tol: float, label: str) -> None:
    if abs(a - b) > tol:
        raise SystemExit(f"FAIL {label}: suite={a} isolated={b} tol={tol}")
    print(f"  OK {label}: {a} ~ {b}")

def f1(text: str, pat: str) -> float:
    m = re.search(pat, text, re.M)
    if not m:
        raise SystemExit(f"FAIL pattern not found: {pat!r}")
    return float(m.group(1))

results = []
fail = 0

def block(name, fn):
    global fail
    print(f"\n=== {name} ===")
    try:
        fn()
        results.append((name, "PASS", ""))
    except SystemExit as e:
        msg = str(e)
        print(msg)
        results.append((name, "FAIL", msg))
        fail = 1
    except subprocess.CalledProcessError as e:
        msg = f"subprocess exit {e.returncode}"
        print(f"FAIL {name}: {msg}")
        results.append((name, "FAIL", msg))
        fail = 1
    except Exception as e:
        print(f"FAIL {name}: {e}")
        results.append((name, "FAIL", str(e)))
        fail = 1

def check_obs():
    s = run_suite("test_observability_synthetic")
    i = run_iso("test_observability_synthetic",
                "ObservabilitySynthetic.CoplanarAblationIsDegenerate")
    lam_s = f1(s, r"multi_lambda_min=([0-9.eE+-]+)")
    lam_i = f1(i, r"multi_lambda_min=([0-9.eE+-]+)")
    near(lam_s, lam_i, 0.01, "multi_lambda_min")
    near(lam_s, 1.074, 0.01, "λ_min vs frozen 1.074")

def check_two_stage():
    s = run_suite("test_two_stage_pipeline")
    i = run_iso("test_two_stage_pipeline",
                "TwoStagePipeline.ReproducesConfigCFromClosedFormInit")
    m_s = re.search(r"cm-level=(\d+)/(\d+)", s)
    m_i = re.search(r"cm-level=(\d+)/(\d+)", i)
    if not m_s or not m_i:
        raise SystemExit("FAIL cm-level line not found")
    if m_s.group(1) != m_i.group(1) or m_s.group(2) != m_i.group(2):
        raise SystemExit(f"FAIL cm-level mismatch: {m_s.group(0)} vs {m_i.group(0)}")
    print(f"  OK cm-level: {m_s.group(0)}")
    lw_s = f1(s, r"\|LW trans\| err bias=([0-9.]+)")
    lw_i = f1(i, r"\|LW trans\| err bias=([0-9.]+)")
    near(lw_s, lw_i, 0.5, "|LW trans| mean [mm]")

def check_p3a():
    s = run_suite("test_phase3_dual_lidar_experiment_a")
    i = run_iso("test_phase3_dual_lidar_experiment_a",
                "Phase3DualLidarExperimentA.AspectDiagnosticFlightDAndPOIFlightE")
    cent_s = f1(s, r"P1\.5 centroid=([0-9.]+)")
    cent_i = f1(i, r"P1\.5 centroid=([0-9.]+)")
    near(cent_s, cent_i, 0.5, "P1.5 centroid [mm]")
    obs_s = f1(s, r"iter×3 obs=([0-9.]+)")
    obs_i = f1(i, r"iter×3 obs=([0-9.]+)")
    near(obs_s, obs_i, 0.5, "P1.5 observed-mean [mm]")
    m_s = re.search(
        r"Simulation sign-off @ 戊 0\.5 Hz.*?max\(obs\)≤[0-9.]+mm: (?:YES|NO) \(NE=([0-9.]+) SW=([0-9.]+)",
        s, re.S)
    m_i = re.search(
        r"Simulation sign-off @ 戊 0\.5 Hz.*?max\(obs\)≤[0-9.]+mm: (?:YES|NO) \(NE=([0-9.]+) SW=([0-9.]+)",
        i, re.S)
    if not m_s or not m_i:
        raise SystemExit("FAIL 戊 sign-off block not found")
    near(float(m_s.group(1)), float(m_i.group(1)), 0.5, "戊 NE obs [mm]")
    near(float(m_s.group(2)), float(m_i.group(2)), 0.5, "戊 SW obs [mm]")
    def ding_obs(text, sensor):
        blk = re.search(rf"--- 丁 ---.*?{sensor}.*?obs=\s*([0-9.]+)", text, re.S)
        if not blk:
            raise SystemExit(f"FAIL 丁 {sensor} obs not found")
        return float(blk.group(1))
    for sensor in ("NE", "SW"):
        near(ding_obs(s, sensor), ding_obs(i, sensor), 1.0, f"丁 {sensor} obs [mm]")

def check_p3b():
    s = run_suite("test_phase3_dual_lidar_phase_b")
    i = run_iso("test_phase3_dual_lidar_phase_b",
                "Phase3DualLidarPhaseB.RelativeExtrinsicMcAndCenterRegPrimary")
    coh_s = f1(s, r"common-mode cancellation YES \(([0-9.]+)×\)")
    coh_i = f1(i, r"common-mode cancellation YES \(([0-9.]+)×\)")
    near(coh_s, coh_i, 0.02, "coherent rel/abs")
    white_s = f1(s, r"white noise:\s+rel/mean\(abs\) = ([0-9.]+)")
    white_i = f1(i, r"white noise:\s+rel/mean\(abs\) = ([0-9.]+)")
    near(white_s, white_i, 0.1, "white rel/mean(abs)")

def check_p15():
    s = run_suite("test_phase15_main_table")
    i = run_iso("test_phase15_main_table",
                "Phase15MainTable.DiverseAspectVariantsAndTwoByFiveTable")
    obs_s = f1(s, r"① observed-mean p_B:.*?trans\|=([0-9.]+)")
    obs_i = f1(i, r"① observed-mean p_B:.*?trans\|=([0-9.]+)")
    near(obs_s, obs_i, 0.5, "phase15 observed-mean |trans|")
    joint_s = f1(s, r"② joint-opt p_B:.*?trans\|=([0-9.]+)")
    joint_i = f1(i, r"② joint-opt p_B:.*?trans\|=([0-9.]+)")
    near(joint_s, joint_i, 0.5, "phase15 joint-opt |trans|")
    def cent_div(text):
        m = re.search(
            r"\| centroid-only\s+\|\s*[0-9.]+ / [0-9.]+\s+\|\s*[0-9.]+ / ([0-9.]+)",
            text)
        if not m:
            raise SystemExit("FAIL centroid-only diverse cell not found")
        return float(m.group(1))
    near(cent_div(s), cent_div(i), 1.0, "phase15 centroid-only diverse |trans|")

block("⑧ observability λ_min", check_obs)
block("⑤ Config C two-stage 50/50 + |LW trans|", check_two_stage)
block("①③④⑥ phase3 experiment_a", check_p3a)
block("§7.2 phase3 phase_b", check_p3b)
block("① phase15 main table", check_p15)

print("\n=== ISOLATION SUMMARY ===")
for name, status, msg in results:
    print(f"  {status}: {name}" + (f" — {msg}" if msg and status == "FAIL" else ""))

if fail:
    sys.exit(1)
print("\n[isolation] PASSED")
PY
