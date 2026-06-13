# Board-free mechanism — oral-depth argument chain

**Status:** Phase 1 production modules (`body_gated_calibration`,
`multi_lidar_body_calibration`, `observed_mean_gate`, `relative_extrinsic`).
**Frozen baseline:** `f03c399` (self-contained checkout) + Phase 1 multi-LiDAR
`3fb7ddf`. Numbers: `doc/board_free_results_frozen.md`.

**Punchline:** aspect-dependent body-centroid bias is a **vector field on u_B**.
Under concentrated aspect it **degenerates with t_LW** and silently pollutes
extrinsics; **active 3D attitude excitation (POI tidal lock + diverse pitch/yaw)**
breaks the degeneracy so the bias decorrelates and averages in world frame —
**active excitation is necessary**, not optional, for unbiased board-free calibration.

---

## 1. bias_B ↔ u_B vector field (not a scalar offset)

**Model:** At each body-cluster epoch `i`,

\[
\text{bias}_{B,i} = \hat{\mathbf{c}}_{B,i}^{\mathrm{emp}} - \mathbf{L}_{B\to\mathrm{body}}^{\mathrm{nom}}
\]

where \(\hat{\mathbf{c}}^{\mathrm{emp}}\) is the empirical centroid of **visible**
sampled points (not the box center + noise). The observed-mean lever
\(\mathbf{p}_B\) estimates the **mean of this field** over the flown aspect distribution.

**Flight 丁** (wide u_B sweep, decoupled orbit) @ seed 13025:

| Sensor | u_B az std [°] | varying RMS [mm] | r(u_B, bias_x) | r(u_B, bias_y) |
|--------|----------------|------------------|----------------|----------------|
| NE | 14.1 | **110** | +0.53 | **−0.68** |
| SW | 17.6 | **124** | −0.64 | **+0.69** |
| P1.5 ref | — | **22** | — | — |

Opposite signs on x vs y → **near-sinusoidal** coupling to u_B azimuth; range-only
models rejected (`doc/board_free_results_frozen.md` §③).

**Test:** `test_phase3_dual_lidar_experiment_a` (Flight 丁 block).

---

## 2. 2×2 degeneracy table (u_B × R_WB)

Mechanism closure for board-free `p_B` identifiability:

|  | **R_WB sweeps** (pitch/roll excitation) | **R_WB near-locked** (low pitch std) |
|--|----------------------------------------|--------------------------------------|
| **u_B sweeps** (flight **丁**) | NE obs **60.3** / SW **89.9** mm; rel obs **311** mm | — |
| **u_B POI-locked** (flight **戊**) | NE obs **12.1** / SW **42.0** mm†; rel rot **0.12°** | 甲 gate OFF: **31.5** mm (both sensors) |

† **[勘误 @ P1 multiseed]** 12/42 为 anchor seed 特例；multiseed 结论见 `board_free_results_frozen.md` §⑥ 勘误与 `paper_supplementary_experiments.md`。

### Concentrated-aspect silent pollution (Phase 1.5)

| Path | \|trans\| [mm] | Gate | Interpretation |
|------|---------------|------|----------------|
| Centroid-only, concentrated | **211.4** | — | bias ∥ t_LW degeneracy |
| Centroid-only, diverse | **63.5** | — | yaw diversity partial help |
| Observed-mean ×3, diverse | **18.5** | ON | `p_B` decorrelates bias |
| Joint-opt p_B, concentrated | **~211** (no-op) | — | joint cannot uncouple |
| Observed-mean, diverse | **20.4** | ON | production target |

**Joint p_B no-op:** On concentrated aspect, joint optimization **does not** remove
the 211 mm floor — bias and t_LW remain coupled; only aspect diversity + gate unlocks
observed-mean.

**Test:** `test_phase15_main_table` (`EXPECT_NEAR` 211.4 mm concentrated centroid).

### Flight 乙 guard (gate as mechanism)

| yaw_cv | pitch_std | Gate | obs \|trans\| |
|--------|-----------|------|-------------|
| **0.32** | low | **OFF** | **165.0** mm (= centroid rollback) |

Gate OFF → system **knows** excitation is insufficient and refuses a worse `p_B`.

---

## 3. Frame-wise quantitative core: why excitation is necessary

World-frame bias driver per frame:
\[
\mathbf{v}_i = R_{LW}\, R_{WB}(t_i)\, \mathbf{b}_{\mathrm{const}}
\]

Aggregated over flight (`body_centroid_analysis` / `test_phase15_main_table`):

| Regime | ‖mean(v)‖ horizontal [mm] | pitch_std [°] | yaw_cv |
|--------|---------------------------|---------------|--------|
| Concentrated aspect | **~220** | ~0 | ~0 |
| Diverse (P1.5) | **~51** | ~16 | **~0.78** |

- **Horizontal ~220 → ~51 mm:** yaw sweeps average out horizontal components of
  \(R_{WB}\mathbf{b}\) — diversity in **attitude**, not translation alone.
- **Vertical ~19.5 mm residual:** body-Z component of \(\mathbf{b}_{\mathrm{const}}\);
  yaw cannot average it; **observed-mean \(\mathbf{p}_B\)** removes it.

**Tidal lock audit:**

| Scenario | yaw−orbit std [°] | locked |
|----------|-------------------|--------|
| P1.5 NearFieldHighAspect | **0.0** | YES |
| 丁 sector-0 (decoupled att) | **110.4** | NO |

**Conclusion sentence:** *Active 3D attitude excitation (diverse pitch/yaw + POI
sector lock where needed) is a **necessary condition** for unbiased board-free
extrinsic calibration — not a refinement.*

---

## 4. Gate + guardrail as engineering embodiment

| Layer | Rule | Frozen evidence |
|-------|------|-----------------|
| **Aspect gate** | `yaw_cv ≥ 0.65` (+ POI path) → try observed-mean ×3 | P1.5 ON @ 0.78 |
| **Fallback** | gate OFF → centroid-only | 乙 **165 mm** (not 1243 mm blow-up) |
| **Guardrail** | apply obs only if cost ≤ centroid **and** \|trans\|_obs ≤ \|trans\|_cent | `phase15_ablation_common.hpp` |

Gate = **system self-awareness**: when aspect support is insufficient, do not
pretend `p_B` is identifiable.

**Production modules:**

| Module | Role |
|--------|------|
| `observed_mean_gate` | yaw_cv threshold + POI path |
| `body_gated_calibration` | gated observed-mean ×3 + cost/trans guard |
| `multi_lidar_body_calibration` | **per-sensor independent Stage-2**; compose `T_rel` |
| `relative_extrinsic` | §7.2 cov ratio, center-reg metric |
| `Stage1TrajectoryFitter` | Prais–Winsten RTK (`use_rtk_prais_winsten=true`) |

**Multi-LiDAR (Phase 1 @ `3fb7ddf`):** Each sensor calls
`CalibrateBodyGatedObservedMean` in a **fresh `ExtrinsicRefiner` Ceres scope** —
no shared `CalibrationEstimator`, no shared Stage-1 pre-fit that cross-pollutes.
\(T_{SW\leftarrow NE} = T_{SW,W}\,T_{NE,W}^{-1}\) composed post hoc.
Regression: `test_multi_lidar_body_calibration` — production NE/SW = solo in-process.

**§7.2 (sim, sealed):** coherent RTK bias **0.166×** rel/abs; white noise **3.101×**.
Scope: injected coherent systematic bias — **not a field guarantee**
(`board_free_results_frozen.md` §7.2 disclosure).

---

## 5. Field expectation: 戊 NE/SW asymmetry (honest)

> **[勘误 @ P1 multiseed]** 本节几何各向异性机理论述已撤回；原文保留作历史记录，现行结论见 `board_free_results_frozen.md` §⑥ / Known issues 勘误及 `paper_supplementary_experiments.md`。

| Sensor | obs \|trans\| @ 0.5 Hz | Production @ `3fb7ddf` |
|--------|------------------------|-------------------------|
| NE | **12.15** mm | **12.149** mm |
| SW | **42.04** mm | **42.041** mm |

**Not a bug.** Diagonal corner-post geometry + sector POI coupling → SW effective
lever projection of \(\mathbf{b}_{\mathrm{const}}\) larger than NE. Re-verified
under **leak-free** production path (independent ExtrinsicRefiner per sensor).

**Paper:** Present asymmetry with per-sector geometry figure; explain via
`ComputeAspectLeverTranslationProjection` (lever_h, projected jitter). **Do not**
claim symmetric dual-LiDAR accuracy without measuring both sectors.

---

## 6. Known limitations (deferred)

| Issue | Status |
|-------|--------|
| `CalibrationEstimator::ClearProblem()` `release()` leak | **Deferred** — only if same estimator reused for 2nd FIM (`λ≈0.02`). Phase 5 online FIM must fix or use fresh instances. |
| Ceres `num_threads=1` | Phase 0 guard; revisit for large field bags |
| PW Stage-1 10 Hz RMSE 27.3 mm vs 23.7 mm @ 0.5 Hz | accepted; ρ scalar limitation |

Details: `doc/board_free_results_frozen.md` § Known issues.

---

## 7. Gate references

| Gate | Test / script |
|------|----------------|
| Frozen baseline + isolation 5/5 | `scripts/verify_frozen_baseline_isolation.sh` @ `f03c399`+ |
| λ_min **1.07357** / threshold 0.003 | `test_observability_synthetic` (fork-isolated suite) |
| 戊 0.5 Hz NE/SW | `test_phase3_dual_lidar_experiment_a` |
| Multi-LiDAR no-leak | `test_multi_lidar_body_calibration` @ `3fb7ddf` |
| §7.2 0.166× / 3.101× | `test_phase3_dual_lidar_phase_b` |
| 2×2 + 211 mm / 20 mm | `test_phase15_main_table` |
