# Synthetic evaluation — noise-regime results (paper tables)

**Branch:** `experiments/noise-regime` (local, post STEP 0–4)  
**Noise model:** `config/noise_model.yaml` — RTK σ_h=10 mm, σ_v=20 mm; LiDAR σ_r=20 mm; camera σ_pix=1.0 px  
**Prior (default):** extrinsic σ_rot=5°, σ_trans=0.5 m (`config/sensor_rig.yaml`)  
**Report format:** mean ± std (max) over **N=20** independent seeds unless noted.

> **Do not cite** single-seed or noise-free smoke numbers as methodological accuracy.

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

**Finding:** Monte Carlo spread of **point errors** is tiny (init=prior≈GT pinning) while **F_ext⁻¹** reports huge σ → **uncertainty model not validated** at current init; do **not** claim FIM–empirical agreement in paper until init/prior ablation fixed.

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

---

## §6 Noise-free smoke (not for paper accuracy)

`SmokeTest.NoiseFreeFullPipelineRecoversGroundTruth`: exact observations, strict tolerances (rot <0.5°, trans <5 cm, t_d ±2 ms, traj RMS <3 cm). **Identity check only.**

See also: [time_offset_observability.md](../diagnostics/time_offset_observability.md).
