# Time-offset observability diagnosis (`debug/time-offset`, 2026-05-20)

Paper context: supports §3 observability of \(t_d^L, t_d^C\) under multi-layer UAV motion and
explains why coplanar flight leaves vertical extrinsics weakly observed at long range.

---

## 1. Symptom

Python end-to-end calibration at 200 m standoff recovered **`t_d_lidar ≈ -1.0 s`**
(hard bound), while simulation ground truth is **`t_d^L = +30 ms`**. LiDAR and camera
time offsets appeared coupled to a translation gauge between prior mean and simulation GT.

---

## 2. STEP 1 — Single-variable experiment (Jacobian sanity)

**Tool:** `test/diagnostic/test_td_single_variable.cpp`

Isolated Ceres problems with **only `t_d` free**; trajectory knots and `T_LW` fixed.

| Sub-test | Motion | GT \(t_d\) | Recovered | Result |
|----------|--------|------------|-----------|--------|
| LiDAR dynamic | multi-layer spline | +30 ms | +30 ms ± 0.1 ms | PASS |
| LiDAR dynamic | multi-layer spline | −15 ms | −15 ms ± 0.1 ms | PASS |
| LiDAR **static** | constant pose | +30 ms | stuck at init | **Non-observable** (expected) |
| Camera dynamic | multi-layer spline | −15 ms | −15 ms ± 0.1 ms | PASS |

**Conclusion:** \(\partial r / \partial t_d\) is **correct** for both LiDAR and camera factors.
Failure in the full pipeline is **not** a wrong time-derivative sign.

---

## 3. STEP 3 — Gauge analysis (full problem)

**Tool:** `test/diagnostic/step3_gauge_analysis.cpp`

Reproduced on Python synthetic data (`seed=123`, 200 m):

| Configuration | `t_d_lidar` | Root cause |
|---------------|-------------|------------|
| yaml prior mean `(3, -1, 0.5)` vs sim GT `(0, 0, 0.5)` | **−1.0 s** (bound) | Prior / init mismatch + wide `[-1, +1] s` clamp |
| Prior mean aligned to sim GT | ≈ +30 ms | Gauge anchored |
| GT prior alone insufficient at 200 m without multi-layer | large Z / t_d drift | Weak pitch observability at range |

**Fix (estimator + config, not factor headers):**

- Align simulation GT `T_LW` with `config/sensor_rig.yaml` `initial_T_LW`.
- Tighten `t_d_max_abs_s` to **0.1 s** (`config/spline.yaml`).
- Unified prior strategy: `ExtrinsicPriorFactor` mean = yaml `initial_T_*`.
- RTK warm-start inside `solve()` (`RunRtkWarmStart()`).

---

## 4. STEP 4 — Strict tolerance reinstatement (noise-free smoke only)

Unified paths (yaml init, prior, `t_d` bounds, RTK warm-start). The test
`SmokeTest.NoiseFreeFullPipelineRecoversGroundTruth` (`test_full_pipeline_synthetic.cpp`)
uses **noise-free RTK + exact LiDAR/camera** — it is a **fast wiring / identity check**,
**not** a methodological noise evaluation.

**Paper-grade numbers:** see [synthetic_evaluation.md](../results/synthetic_evaluation.md)
(STEP 2–4 seed sweeps under `config/noise_model.yaml`).

### Verified precision (noise-free smoke, single deterministic run)

**Local smoke regression** (`SmokeTest.NoiseFreeFullPipelineRecoversGroundTruth`):

| Quantity | Recovered | Tolerance | |
|----------|-----------|-----------|---|
| `T_LW` rotation | 0° | < 0.5° | PASS |
| `T_CW` rotation | 0° | < 0.3° | PASS |
| `T_LW` translation | (3, −1, 0.5) m | < 5 cm | PASS |
| `T_CW` translation | (2, 1.5, 0.2) m | < 3 cm | PASS |
| `t_d^L` | 0.030 s | ± 2 ms | PASS |
| `t_d^C` | −0.015 s | ± 2 ms | PASS |
| Trajectory RMS | **1.7 cm** | < 3 cm | PASS |

### Superseded 200 m single-seed report (do not cite)

Earlier `test_patent_z_accuracy` printed **Z-error = 0.0 m** for one seed — display
truncation / init=prior=GT pinning, **not** methodological accuracy. Replaced by
20-seed sweep in [synthetic_evaluation.md §3](../results/synthetic_evaluation.md):
multi-layer **z_err = 2.973 ± 5.277 mm** (max 21.156 mm).

**FIM / observability** (`test_observability_synthetic`, noise-free RTK at linearization):

- Multi-layer: `λ_min(F_ext) > 0`, condition number ≪ 10⁸.
- Coplanar ablation: `λ_min` ≪ multi-layer, PDOP_ext larger, pitch/Z dominate worst eigenvector.

Under **realistic noise**, FIM posterior σ **does not** match Monte Carlo spread — see
[synthetic_evaluation.md §3.4](../results/synthetic_evaluation.md).

---

## 5. RMS vs 200 m Z-error (reviewer FAQ)

These metrics are **not contradictory**, but **neither alone proves noise-regime accuracy**:

- **Trajectory RMS** — mean 3-D body position error over the observed window (all axes, local or at standoff). Under realistic noise: **91.7 ± 16.3 mm** locally ([synthetic_evaluation.md §2](../results/synthetic_evaluation.md)).
- **200 m Z-error** — `\|T_LW.t.z − gt.z\|` after calibration; under multi-layer noise: **2.973 ± 5.277 mm** mean ([synthetic_evaluation.md §3](../results/synthetic_evaluation.md)).

The old claim that cm-level RMS implies sub-decimetre Z at 200 m applied only to the **noise-free smoke** path and is **withdrawn** for the noisy regime.

---

## 6. Reproduction

```bash
# Factor + pipeline + diagnostics (no catkin)
scripts/compile_local_tests.sh --run

# Full regression (catkin devel if present, else local g++)
scripts/run_regression_tests.sh
```
