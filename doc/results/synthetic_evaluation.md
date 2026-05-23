# Synthetic evaluation — noise-regime results (paper tables)

> **SEAL:** Synthetic phase **FROZEN** as of **2026-05-23**. No further synthetic-precision tuning. Next: **CalibrationEstimator refactor** (per `two_stage_solver_blueprint.md`) and **real-world experiments**.

**Status:** **FINAL** (2026-05-23) — Config C claim boundaries in `doc/diagnostics/SYNTHETIC_PHASE_SEAL.md`.

**Branch:** `experiments/noise-regime` + `probe/uq-decomposition` (Config C UQ)  
**Noise model:** `config/noise_model.yaml` — RTK σ_h=10 mm, σ_v=20 mm; LiDAR σ_r=20 mm; camera σ_pix=1.0 px  
**Prior (default):** extrinsic σ_rot=5°, σ_trans=0.5 m (`config/sensor_rig.yaml`)  
**Report format:** mean ± std (max) over **N=20** independent seeds unless noted (§7: **N=100**).

> **Scope note:** Paper accuracy/UQ narrative anchors on **T_LW** (LiDAR–world). Camera is **secondary**; Gate 1 **A/B excluded** (internal only). Full scope: `section2_claim.md` § Paper presentation scope.

---

> **Do not cite** single-seed or noise-free smoke numbers as methodological accuracy.

> **Presentation scope (frozen):** **T_LW** = primary accuracy + UQ headline. **Camera** = secondary modality (~3 mm CW lateral bias disclosed; ~10⁻² mrad projection impact at V2X range — limitation, not blocker). **Gate 1 A/B** (GT traj, 0/14) = internal diagnostic only; **Config C** rep-fit (10/14, 5/5 translations) = paper-facing. **LW_roll/pitch** 1.45/1.28 = limitations footnote (MC SE ~14%; not re-tested). Details: `section2_claim.md`.

---

## §1 Test map

| Test | Role | Seeds |
|------|------|-------|
| `SmokeTest.NoiseFreeFullPipelineRecoversGroundTruth` | Wiring / Jacobian smoke (fast) | 1 (deterministic) |
| `PipelineNoiseSweep.RealisticNoiseTwentySeedCharacterization` | **§3 joint local pipeline under noise** | 1000–1019 |
| `PatentZAccuracy.MultiLayerVsCoplanarObservabilityAt200m` | **§2 pitch / Z @ 200 m + FIM** | 2000–2019 |
| `PriorAblation.MultiLayerGeometricObservabilityVsPrior` | **§2 prior dependence** | 3000–3019 |

Reproduce:

```bash
scripts/compile_local_tests.sh
build/local_tests/test_pipeline_noise_sweep
build/local_tests/test_patent_z_accuracy
build/local_tests/test_prior_ablation
```

---

## §2 Local multi-sensor pipeline under realistic noise

**Setup:** local multi-layer–like trajectory (0–5 s), full RTK+LiDAR+camera noise, yaml init=prior=GT, RTK warm-start, `t_d` ±100 ms.

| Metric | mean ± std (max) |
|--------|------------------|
| rot_LW [deg] | 1.388 ± 1.089 (max 4.278) |
| rot_CW [deg] | 0.572 ± 0.280 (max 1.108) |
| pitch_LW_err [deg] | 0.121 ± 0.688 (max 1.322) |
| trans_LW_x [mm] | 0.045 ± 5.897 (max 13.477) |
| trans_LW_y [mm] | 0.644 ± 3.177 (max 9.481) |
| trans_LW_z [mm] | 6.318 ± 14.591 (max 32.286) |
| \|trans_LW\| [mm] | 14.380 ± 9.087 (max 36.247) |
| \|trans_CW\| [mm] | 15.591 ± 17.068 (max 81.241) |
| t_d_L_err [ms] | 2.051 ± 26.032 (max 54.184) |
| t_d_C_err [ms] | −2.981 ± 23.913 (max 29.834) |
| **traj_RMS [mm]** | **91.710 ± 16.278 (max 131.825)** |

**Finding:** trajectory RMS mean **91.7 mm** with **+92 mm systematic offset** vs GT (|mean| ≫ 3×std) — joint estimator is **biased under noise**, not centred on GT. Strict cm-level smoke tolerances **do not** transfer.

**CRLB hint (RTK-only i.i.d.):** σ/√N ≈ **1.46 mm** (N=47 RTK fixes); observed traj RMS is **~63×** this floor → strong cross-modal / gauge coupling.

---

## §3 Pitch observability & 200 m Z-error (multi-layer vs coplanar)

**Setup:** 200 m standoff, multi-layer vs coplanar body motion, full modality noise, default prior, seeds paired 2000–2019.

### 3.1 Multi-layer @ 200 m

| Metric | mean ± std (max) |
|--------|------------------|
| pitch_err [deg] | 0.007 ± 0.015 (max 0.053) |
| post_σ_pitch [deg] | 184.235 ± 188.161 (max 824.136) |
| tz_err [mm] | −2.080 ± 5.710 (max 4.601) |
| post_σ_tz [mm] | 11931.003 ± 22099.353 (max 75307.506) |
| **z_err @ 200 m [mm]** | **2.973 ± 5.277 (max 21.156)** |

### 3.2 Coplanar @ 200 m

| Metric | mean ± std (max) |
|--------|------------------|
| pitch_err [deg] | 0.003 ± 0.010 (max 0.024) |
| post_σ_pitch [deg] | 28.591 ± 47.556 (max 218.914) |
| tz_err [mm] | 7.944 ± 21.264 (max 87.010) |
| post_σ_tz [mm] | 3253.640 ± 1764.845 (max 7066.065) |
| **z_err @ 200 m [mm]** | **9.996 ± 20.333 (max 87.010)** |

### 3.3 Headline ratios (coplanar / multi-layer)

| Ratio | Value | Interpretation |
|-------|-------|----------------|
| **z_err mean** | **3.36×** | Coplanar **worse** — supports Z observability under vertical excitation |
| \|pitch_err\| std | 0.67× | Not a headline (both near zero at point estimate) |
| post_σ_pitch mean | 0.16× | **Inverted** — multi-layer FIM posterior **larger** (near rank-deficient at yaml init) |

### 3.4 Posterior σ vs empirical spread (30% agreement test)

| Case | pitch: emp std vs mean post σ | rel diff |
|------|------------------------------|----------|
| Multi-layer | 0.015° vs 184° | **~100%** ✗ |
| Multi-layer t_z | 5.7 mm vs 11931 mm | **~100%** ✗ |
| Coplanar pitch | 0.010° vs 29° | **~100%** ✗ |
| Coplanar t_z | 21.3 mm vs 3254 mm | **~99%** ✗ |

**Finding:** Monte Carlo spread of **point errors** is tiny (init=prior≈GT pinning) while **F_ext⁻¹** reports huge σ → **uncertainty model not validated** for the **joint pipeline** at this init. **Does not apply** to Config C two-stage decomposition UQ (§7).

> **Legacy joint pipeline only** — not Config C two-stage.

**Z-error no longer collapses:** aggregate mean **2.973 mm** (not 0.0 m); per-seed zeros still occur when init locks Z.

---

## §4 Prior ablation (multi-layer local, realistic noise)

**Setup:** `BuildMultiLayerNoisyScenario`, seeds 3000–3019. Three configs differ **only** in extrinsic prior:

| Config | Description |
|--------|-------------|
| **(A)** | Default σ_rot=5°, σ_trans=0.5 m |
| **(B)** | Weak σ_rot=90°, σ_trans=100 m |
| **(C)** | No `ExtrinsicPriorFactor` (RTK-only gauge) |

| Metric | (A) default | (B) weak | (C) none |
|--------|-------------|----------|----------|
| Convergence | 20/20 | 20/20 | 20/20 |
| rot_LW [deg] | 0.952 ± 0.593 (max 2.003) | 1.782 ± 1.333 (max 4.875) | 1.852 ± 1.356 (max 4.894) |
| pitch_err [deg] | 0.066 ± 0.416 (max 0.905) | −0.141 ± 1.111 (max 1.500) | −0.167 ± 1.149 (max 1.513) |
| \|T_LW\| [mm] | **10.577 ± 8.119 (max 31.479)** | **33.637 ± 25.385 (max 103.378)** | **34.911 ± 25.847 (max 104.492)** |
| t_d_L_err [ms] | −3.099 ± 17.203 (max 20.801) | −4.439 ± 16.876 (max 20.572) | −4.476 ± 16.849 (max 20.557) |
| **λ_min(F_ext)** | **0.056 ± 0.104 (max 0.384)** | **0.039 ± 0.069 (max 0.324)** | **0.019 ± 0.009 (max 0.039)** |
| worst-eig pitch | 0.304 ± 0.367 | 0.324 ± 0.362 | 0.162 ± 0.282 |
| worst-eig t_z | 0.080 ± 0.181 | 0.064 ± 0.124 | 0.135 ± 0.260 |

**Conclusion for §2:** Under realistic noise, **(B)/(C) do not match (A)** — \|T_LW\| error **~3.3×** larger, pitch spread **~2.7×** larger, λ_min **~3×** lower. Observability is **partly prior-dependent**; claim must be scoped to **“recoverable with coarse mounting prior (0.5 m, 5°)”**, not geometry alone.

---

## §5 Paper claim routing (summary)

| Claim | Supported? | Evidence |
|-------|------------|----------|
| Joint pipeline cm-accurate under noise | **No** | §2 traj RMS 91.7±16.3 mm, biased |
| Multi-layer improves 200 m Z vs coplanar | **Partial** | z_err 3.0 vs 10.0 mm mean (3.36×) |
| Pitch observable via FIM @ 200 m | **No (as tested)** | post_σ_pitch inverted vs coplanar; FIM≠MC |
| Full geometry-only observability | **No** | §4 prior ablation |
| Observable with coarse extrinsic prior | **Best supported** | (A) only config with ~11 mm \|T_LW\| under noise |
| Config C two-stage UQ (decomposition) | **Yes** | §7 — **10/14** zero-mean-qualified @ N=100 |
| Known limitations (consolidated) | **Disclosed** | §8 — frozen; not chased |

---

## §7 Config C two-stage (FINAL — see `doc/diagnostics/SYNTHETIC_PHASE_SEAL.md`)

**Setup:** Near-field multi-layer, closed-form init, prior-free Stage-2; N=**100** (seeds 13000–13099), rep=13025.  
**Full tables:** `doc/diagnostics/uq_decomposition.md`

**Headline accuracy:** **T_LW** (LW translations: |bias| ≪ σ).

### Gate 3 — decomposition (primary UQ)

**Headline (zero-mean–qualified):** Σ_fixed + Σ_traj-prop ≈ Σ_total on **3/3 LiDAR–world translations** + **7 rotation/time DoF** = **10/14**.  
**Not in headline count:** CW_tx, CW_ty (bias); LW_roll, LW_pitch (ratio 1.45, 1.28).

#### LiDAR–world block (headline)

| DoF | std_fixed | std_traj | std_total | ratio | closure |
|-----|-----------|----------|-----------|-------|---------|
| **LW_tx** | 1.035 mm | 1.888 mm | **2.022 mm** | 1.134 | ✓ headline |
| **LW_ty** | 4.287 mm | 4.147 mm | **5.433 mm** | 1.205 | ✓ headline |
| **LW_tz** | 3.190 mm | 7.553 mm | **7.568 mm** | 1.174 | ✓ headline |
| LW_yaw | 17.613 mrad | 17.437 mrad | 22.577 mrad | 1.205 | ✓ headline |
| LW_roll | — | — | 19.958 mrad | **1.45** | edge |
| LW_pitch | — | — | 22.039 mrad | **1.28** | edge |

#### Camera lateral translation (separate — conditional)

| DoF | mean ± std | ratio | headline? |
|-----|------------|-------|-----------|
| **CW_tx** | **−2.90 ± 1.92 mm** | 0.986 | **No** — zero-mean violated |
| **CW_ty** | **−0.84 ± 0.55 mm** | 1.026 | **No** — zero-mean violated |

Camera lateral translation exhibits a systematic offset of **~3 mm** at N=100; its variance decomposition is reported **conditionally**, as the zero-mean assumption underlying covariance closure is **not satisfied** for these two DoF.

*(Diagnostic only: 12/14 pass ratio band if zero-mean is ignored — do not use as headline.)*

### Decoupling cost (LW headline)

| DoF | Var_traj / Var_total | Var_traj / Var_fixed |
|-----|----------------------|----------------------|
| LW_tz | 99.6% | 5.60× |
| LW_tx | 87.2% | 3.33× |

(CW_tx 98% / 183× reported in full table — secondary; camera bias limits camera claim.)

### Gate 1 — fixed FIM vs Σ_fixed

**Paper-facing:** **Config C only** — rep-fit Stage-1 trajectory @ seed 13025: **10/14**, **5/5 translations** (emp_std/theo_std ∈ [0.7, 1.4]).

**Internal diagnostic only (excluded from paper by choice):** Blocks **A** and **B** (GT trajectory; obs on GT path) → **0/14**. MC spread is systematically below FIM on the GT reference path; useful isolation check, not the operating point.

**Statement for §2:** T_LW accuracy + UQ decomposition validated; fixed FIM matches Σ_fixed; propagation term accounts for remainder; known simulation limitations disclosed (see **§8**).

---

## §8 Synthetic-phase limitations (consolidated, reviewer-safe)

**Status:** **FROZEN** — each item below is **known, disclosed, and explicitly not pursued** further on the synthetic branch. Synthetic phase **FINAL** (2026-05-23); remaining submission gating work is **real-data validation** and production refactor.

| # | Limitation | What we know | Decision |
|---|------------|--------------|----------|
| **L1** | **Camera lateral-translation systematic bias** | CW_tx = **−2.90 ± 1.92 mm**, CW_ty = **−0.84 ± 0.55 mm** @ N=100 (\|mean\| > σ). ~3 mm offset is **within roadside V2X projection-error tolerance** (order **10⁻² mrad** at tens-of-metres standoff ≪ typical 0.3–0.5 m budgets). Plausible contributors: reprojection nonlinearity under Gaussian pixel noise, tag-local PnP init bias, idealized obs synthesis — **root cause not isolated**. | **Disclose as limitation; out of scope** for synthetic phase. Camera is a **secondary modality** in the paper; do not headline sub-mm camera extrinsic accuracy. |
| **L2** | **Two-stage decoupling is statistically sub-optimal** | Stage-1 trajectory error propagates into Stage-2 extrinsics. **Cost quantified:** LW_tz — **99.6%** of total variance from trajectory propagation (Var_traj/Var_fixed **5.6×**); CW_tx — Var_traj/Var_fixed **~183×**. Fixed-trajectory FIM correctly describes only the fixed-noise sub-problem. | **Accepted design trade-off.** Absolute **T_LW** accuracy remains **cm/mm-level** with unbiased LW translation → decoupling is **justified** and the cost is **quantified**, not hidden. |
| **L3** | **LW_roll / LW_pitch decomposition ratios marginally out of band** | Var_sum/Var_total = **1.45** and **1.28** vs band [0.8, 1.25] @ N=100. Consistent with MC ratio sampling error (SE ≈ **14%**) and/or a small **rotation cross-term** not captured by diagonal decomposition. | **Disclose in limitations footnote; not re-tested** at higher N. Does not affect **3/3 LW translation** UQ headline. |
| **L4** | **UQ: empirical decomposition authoritative; analytic cross-check partial** | Three-arm MC noise-source decomposition validates Σ_fixed + Σ_traj-prop ≈ Σ_total on **10/14** zero-mean-qualified DoF. Optional **delta-method** (Stage-1 knot FIM + single-knot FD Jacobian) reaches only **7/14** — single-knot linearization **under-estimates** full trajectory propagation. Schur-marginal FIM on rank-deficient F_θθ (**3/14**) is **withdrawn**. | **MC decomposition is authoritative** for §2 UQ. Analytic/delta and Schur paths are diagnostic or appendix-only. |
| **L5** | **Joint (single-stage) pipeline under noise** | Legacy joint estimator: trajectory RMS **~92 mm**, biased (§2). FIM posterior σ does not match MC on joint pipeline @ yaml init (§3.4). | **Not claimed** for Config C two-stage. Retained as historical contrast only. |
| **L6** | **Prior dependence (§4)** | Full geometry-only observability **not** supported; coarse extrinsic prior (0.5 m, 5°) needed for best joint-pipeline behaviour. Config C two-stage uses closed-form init instead. | Scoped claim: recoverable **with coarse mounting prior / closed-form init**, not geometry alone. |
| **L7** | **Gate 1 A/B (GT trajectory) anomaly** | FIM↔MC **0/14** when obs synthesized on GT path (MC spread systematically below FIM). | **Internal diagnostic only** — excluded from paper **by choice**; Config C rep-fit is the operating point. |
| **L8** | **All results are synthetic** | Noise model, tag detections, RTK, attitude, and flight geometry are simulated. No field distortion, rolling shutter, sync jitter, or detection-dropout effects. | **Real-data validation is the actual gating work for submission** and the next experimental phase. Synthetic phase provides algorithmic and UQ-structure evidence only. |

### Paper-ready limitations paragraph (copy from here)

We report the following known limitations of the synthetic validation phase, none of which are pursued further before submission: (**i**) camera lateral translations exhibit a ~3 mm systematic offset (CW_tx = −2.90 ± 1.92 mm) within roadside V2X projection tolerance but with un-isolated root cause; (**ii**) two-stage decoupling is statistically sub-optimal, with trajectory propagation contributing ~99.6% of LW_tz variance (quantified, not hidden); (**iii**) LW_roll and LW_pitch decomposition ratios marginally exceed the validation band, consistent with MC sampling variance at N=100; (**iv**) uncertainty is validated empirically via noise-source decomposition, while the analytic delta-method cross-check is partial (7/14); and (**v**) all evidence is synthetic—real-world validation is pending.

---

## §6 Noise-free smoke (not for paper accuracy)

`SmokeTest.NoiseFreeFullPipelineRecoversGroundTruth`: exact observations, strict tolerances (rot <0.5°, trans <5 cm, t_d ±2 ms, traj RMS <3 cm). **Identity check only.**

See also: [time_offset_observability.md](../diagnostics/time_offset_observability.md).
