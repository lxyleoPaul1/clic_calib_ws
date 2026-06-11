# Board-free calibration — frozen baseline (Phase A → B handoff)

**Status:** FROZEN @ `72050cc` family (PW + NE-only POI roll). Do not mutate these
numbers when changing `CalibrationEstimator` in Phase B.

**Rep seed:** `13025` unless noted. **Noise:** `config/noise_model.yaml`.

Reproduce: `scripts/compile_local_tests.sh --run` (full gate).

---

## ① Single-LiDAR Phase 1.5 (NearFieldHighAspect)

| Metric | Value [mm] | Test | Commit |
|--------|------------|------|--------|
| Centroid-only \|trans\| | **63.5** | `test_phase3_dual_lidar_experiment_a` (alignment) | `75f1e17` |
| Observed-mean ×3 \|trans\| | **18.5** | same | `75f1e17` |
| Joint-opt pass-1 (reference) | **20.4** | `test_phase15_joint_opt_audit` | `75f1e17` |
| GT varying RMS (bias field) | **~22** | `test_phase15_main_table` | `75f1e17` |

**Claim:** diverse aspect (yaw_cv ≈ 0.78, pitch_std ≈ 16°) enables `p_B`; centroid
floor ~64 mm, observed-mean ~18–20 mm.

---

## ② bias_B ↔ u_B vector field (aspect diagnosis)

**Flight 丁 @ GT back-projection** (`test_phase3_dual_lidar_experiment_a`, Flight 丁 block):

| Sensor | u_B az std [°] | varying RMS [mm] | r(u_B_az, bias_x) | r(u_B_az, bias_y) |
|--------|----------------|------------------|-------------------|-------------------|
| NE | 14.1 | **110** | +0.53 | **−0.68** |
| SW | 17.6 | **124** | −0.64 | **+0.69** |
| P1.5 ref | — | **22** | — | — |

**Conclusion:** bias_B components correlate strongly with u_B azimuth (near-sinusoidal /
opposite signs on x vs y). Range-only models (a)(b)(c) excluded. **Commit:** `75f1e17`.
**Doc:** `doc/PHASE3_ASPECT_BIAS.md`.

**Tidal lock audit:**

| Scenario | yaw−orbit std [°] | locked |
|----------|-------------------|--------|
| P1.5 NearFieldHighAspect | **0.0** | YES |
| 丁 sector-0 (decoupled att) | **110.4** | NO |

---

## ③ Three failure dimensions (direction / amplitude / distance)

| Dimension | Diagnosis | Numbers | Resolution | Test / commit |
|-----------|-----------|---------|------------|---------------|
| **Direction** (u_B sweep) | `bias_B(u_B)` not constant; wide u_B on 丁 | varying RMS 110–124 mm vs P1.5 22 mm | Flight **戊 POI** locks u_B per sector | `test_phase3_dual_lidar_experiment_a` / `75f1e17` |
| **Amplitude** (R_WB / attitude) | pitch_std ≈ 0 → gate OFF; no `p_B` ID | 乙 centroid **165 mm** | yaw_cv ≥ 0.65 + pitch_std ≥ 14° gate; POI + roll tighten | `phase15_ablation_common.hpp` / `4b0c7eb` |
| **Distance** (range-only) | \|r(bias)\| weak (−0.53 NE / −0.16 SW); 5× range_coeff ineffective | no improvement | **Rejected** — `range_coeff×5` reverted | `75f1e17` |

---

## ④ 2×2 degeneracy table (u_B × R_WB)

Mechanism closure for board-free body-centroid `p_B` identifiability:

|  | **R_WB sweeps** (pitch/roll excitation) | **R_WB near-locked** (low pitch std) |
|--|----------------------------------------|--------------------------------------|
| **u_B sweeps** (flight **丁**, decoupled orbit) | NE obs **60.3** / SW **89.9** mm; rel obs **311** mm | — |
| **u_B POI-locked** (flight **戊**) | NE obs **12.2** mm; SW **42.0** mm†; rel rot **0.12°** | 甲 gate OFF: **31.5** mm (both sensors) |

† SW 42 mm = synthetic sector anisotropy (exempted at sim gate, not threshold raised).

**Flight 乙** (both u_B and yaw_cv low): gate OFF → obs = centroid **165.0** mm
(guard prevents 1243 mm blow-up). **Test:** `test_phase3_dual_lidar_experiment_a`.
**Commits:** `75f1e17`, `72050cc`.

---

## ⑤ Prais–Winsten RTK necessity (Stage-1)

| Stage-1 path | 10 Hz antenna RMSE [mm] | Notes | Commit |
|--------------|-------------------------|-------|--------|
| v2 uniform AR(1) scale only | **36.0** | does not decorrelate RTK drift | `76cf1d6` |
| **PW differenced whitening** | **27.3** | ρ from RTK residual lag-1 | `fbe701f` |
| Uniform Σ inflation (wrong) | **~125** | kills low-frequency; **do not reuse** | `76cf1d6` audit |
| 0.5 Hz (independent RTK) | **23.7** | PW auto-off (Δt ≥ 0.2 s) | `fbe701f` |

**Test:** Stage-1 audit block in `test_phase3_dual_lidar_experiment_a`.

---

## ⑥ Diagonal dual-LiDAR @ flight 戊 (seed 13025)

**Geometry:** NE/SW posts (±25, ±25) m, baseline **70.7 m**, POI serial lock
t∈[0,45)→NE, t∈[45,90)→SW. **NE-only** `poi_sector0_attitude_scale=0.35`.

| Tier | NE obs [mm] | SW obs [mm] | rel rot [°] | center-reg obs [mm] | rel-trans obs [mm] |
|------|-------------|-------------|-------------|---------------------|-------------------|
| **0.5 Hz** | **12.2** | **42.0**‡ | **0.12** | **41** | ~74 (0.06°×70.7 m) |
| 10 Hz (PW + NE roll) | 58.8 | 117.7† | 0.18 | 134 | — |

‡ **known synthetic sector artifact, exempted** @ sim gate 42 mm (not raised to 43).
† SW 10 Hz: observed-mean fallback (sector coupling via Stage-1).

**Sector anisotropy:** SW obs ~42 mm with gate ON, `p_B` applied; per-frame audit
u_B 3.58°→1.26° under both-flag does not shrink obs → synthesis limit.
**Commit:** `a78dce7` audit, `72050cc` production config.

**Test:** `test_phase3_dual_lidar_experiment_a`.

---

## ⑦ Observed-mean gate + guardrail (乙 165 mm rollback)

| Policy | Rule | 乙 @ 13025 | Test / commit |
|--------|------|------------|---------------|
| Aspect gate | yaw_cv ≥ **0.65** (+ POI tidal path) | yaw_cv ≈ **0.32** → gate **OFF** | `phase15_ablation_common.hpp` |
| Residual guard | monotonic Ceres cost; \|trans\|_obs ≤ centroid | obs = centroid **165.0** mm | `test_phase3_dual_lidar_experiment_a` |
| Anti-pattern | pitch-only gate (removed) | would open → **1243 mm** class failures | `4b0c7eb` |

**Claim:** production path never outputs observed-mean worse than centroid-only.

---

## Phase B scope (not frozen here)

Phase B changes **`CalibrationEstimator`** only. Outputs unique to multi-LiDAR:

1. **(a)** Relative extrinsic covariance + RTK common-mode cancellation
   (`Cov(T_{L1L2})` vs absolute) — `test_phase3_dual_lidar_phase_b`.
2. **(b)** Intersection **center registration error** as primary relative metric
   (replaces rel-trans @ 70.7 m baseline leverage).

### Phase B snapshot @ seed 13025 (`test_phase3_dual_lidar_phase_b`)

**(b) Primary relative metric (frozen):**

| Metric | @ rep |
|--------|-------|
| center-reg (obs) | **40.9 mm** |
| rel-trans (obs) | 49.0 mm |
| center / rel | **0.84×** |

**(a) RTK perturbation source (§7.2) — N=20, obs=@rep, serial 戊:**

| Injection | trace Cov(δt_rel) / mean Cov(δt_abs) | r(‖δp‖\_NE, ‖δp‖\_SW) |
|-----------|--------------------------------------|-------------------------|
| **White noise (per epoch)** | **3.10×** (stacks in T_rel) | **0.40** (segments independent) |
| **Coherent system bias** | **0.17×** (cancels in T_rel) | **1.00** |

**Precise claims:**

- Relative extrinsic is **robust to RTK systematic (coherent) bias** — common-mode
  cancellation in `T_rel` even with independent Stage-2.
- **White RTK noise composes** in the relative frame (~3× vs absolute) — not a
  joint-estimator issue; per-epoch noise + serial sectors do not share coherent
  perturbation across 0–45 s / 45–90 s.

**(3) Temporal overlap (10 s @ handoff):** coherent-bias rel/mean(abs) **0.15×**
(vs serial **0.17×**). Board-free does not require spatial FOV overlap; brief
temporal overlap can further tighten relative UQ.

See `doc/multi_lidar_board_free_blueprint.md`.

---

## Gate checklist (reproduce)

| Check | Command / test |
|-------|----------------|
| Full compile + run | `scripts/compile_local_tests.sh --run` |
| Config C 50/50 | `test_two_stage_pipeline` |
| P1.5 63.5 / 18.5 / 20.4 | `test_phase3_dual_lidar_experiment_a` alignment |
| Dual alignment parity | same test (`EXPECT_NEAR` dual vs single) |
| 乙 165 mm rollback | same test (`EXPECT_FALSE` gate, ~165 mm) |
| Sim sign-off 42 mm + SW exempt | same test sign-off block |
| Phase B (a)(b) | `test_phase3_dual_lidar_phase_b` |
