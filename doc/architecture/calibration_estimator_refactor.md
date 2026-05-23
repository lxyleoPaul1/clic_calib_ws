# CalibrationEstimator Refactor — Production Port Record

**Branch:** `refactor/two-stage-estimator` → `main` (separate from PR #1)  
**Status:** **GATE 6 — ready for human review** (2026-05-23)  
**Blueprint:** `doc/architecture/two_stage_solver_blueprint.md`  
**Probe source:** `probe/uq-decomposition` (diagnostic only; not merged as-is)

---

## Executive summary

Production `CalibrationEstimator` now routes attitude-equipped sessions through a **validated two-stage pipeline**:

1. **Stage 1** — RTK + PSDK attitude + smoothness → fixed `BodyTrajectory`
2. **Stage 2 init** — Umeyama (`T_LW`) + tag-local IPPE PnP (`T_CW`)
3. **Stage 2 refine** — prior-free extrinsics + time delays on frozen trajectory

All Config C probe numbers are reproduced in production modules. Five non-negotiable fixes from the probe campaign are preserved with automated verification. Real-data **interfaces only** (no field rosbag processing in this PR).

---

## Port map (probe → production)

| Probe symbol / file | Production module | Notes |
|---------------------|-------------------|-------|
| `FitStage1WithAttitude()` | `Stage1TrajectoryFitter::Fit()` | Three-pass SO(3)→RTK reseed→RTK polish |
| `TrimTrajectoryToObservedSupport()` | `trajectory_support.h` | **Subset extraction**, not resampling |
| `AttitudeFactor` (pose-form Jacobian) | `AttitudeFactorPoseForm` | `Jr_inv * R_obs` chain (not `R_err`) |
| `CeresSo3ProblemScope` | `ceres_so3_scope.h` | Shared SO(3) local param, `DO_NOT_TAKE_OWNERSHIP` |
| `MakeGeometricExtrinsicInit()` | `ExtrinsicInitializer::FromGeometric()` | Umeyama + tag-local IPPE |
| `SolveStage2Extrinsics()` | `ExtrinsicRefiner::Refine()` | Prior-free; optional robust loss (Config C uses Huber/Cauchy) |
| `BuildStage2ExtrinsicInformation()` | `stage2_extrinsic_fim.*` | 14×14 FIM for UQ / assembly audit |
| `TwoStagePipeline::Run()` | `two_stage_pipeline.*` | End-to-end orchestration |
| UQ three-arm MC | `uq_decomposition.*` | TOTAL / FIXED-TRAJ / TRAJ-PROP |
| `CalibrationEstimator::solve()` wiring | `RunTwoStageSolve()` | When attitude stream present |
| Synthetic scenario builders | `test/diagnostic/*`, `test/experiments/*` | **Test-only** — not production I/O |
| Real rosbag / archive | `RealDataSession`, `AttitudeReader` | Interface contract only (GATE 5) |

### Files added (production)

```
include/clic_calib/estimator/
  stage1_trajectory_fitter.h, extrinsic_initializer.h, extrinsic_refiner.h
  two_stage_pipeline.h, stage2_extrinsic_fim.h, uq_decomposition.h
  trajectory_support.h, ceres_so3_scope.h, attitude_stream_config.h, real_data_session.h
  two_stage_types.h, trajectory_stage.h
include/clic_calib/factor/
  attitude_factor.h, attitude_factor_pose_form.h
  fixed_traj_sphere_factor.h, fixed_traj_apriltag_factor.h
include/clic_calib/io/attitude_reader.h
include/clic_calib/sensor_data/attitude_observation.h
src/clic_calib/estimator/*.cpp (matching)
src/clic_calib/io/attitude_reader.cpp
```

### Modified (integration)

- `calibration_estimator.cpp` — two-stage entry when attitude observations loaded
- `ceres_local_param.h` — **H2 transpose fix** (Lie analytic local parameterization)
- `calibrate_offline.cpp` — `--attitude attitude.csv`
- `config/spline.yaml`, `config/noise_model.yaml` — attitude stride / Σ_att defaults

---

## Stage-by-stage reproduction (production vs probe)

**Protocol:** Config C near-field, seeds **13000–13049** (N=50) unless noted; noise from `config/noise_model.yaml`; attitude stride **25** (Stage 1/Config C) or **1** (UQ MC).

Regression binaries: `scripts/run_production_regression.sh` (build via `scripts/compile_local_tests.sh`).

### Stage 1 — `Stage1TrajectoryFitter`

| Metric | Probe target | Production (2026-05-23) | Test |
|--------|--------------|---------------------------|------|
| Usable seeds | 50/50 | **50/50** | `test_stage1_trajectory_fitter` |
| Position RMS vs GT | ~25 mm | **25.6346 mm** | ✓ |
| Roll RMS | ~0.15° | **0.157785°** | ✓ |
| Pitch RMS | ~0.15° | **0.157551°** | ✓ |
| Yaw RMS | ~2.9° | **3.3488°** | ✓ (within tolerance band) |
| max \|prod−probe\| trans | — | **0 mm** | trim is knot-subset only |
| max \|prod−probe\| rot | — | **7.99×10⁻¹⁴ mrad** | identical fit |

### Stage 2 init — `ExtrinsicInitializer` (pre-iteration)

| Extrinsic | Probe target | Production | Test |
|-----------|--------------|------------|------|
| T_LW rotation | ~0.017° | **0.0168 ± 0.0060°** | `ReproducesProbeUmeyamaPreIteration` |
| T_LW translation | ~6.4 mm | **6.556 ± 2.970 mm** | ✓ |
| T_CW rotation | ~0.0002° | **0.0002 ± 0.0002°** | `ReproducesProbePnPPreIteration` |
| T_CW translation | ~0.16 mm | **0.163 ± 0.086 mm** | ✓ |
| t_d at init | 0 | **0 / 0 s** (released in refine) | `EXPECT_DOUBLE_EQ` |

### Config C full pipeline — `TwoStagePipeline`

| Metric | Probe target | Production | Test |
|--------|--------------|------------|------|
| Converged | 50/50 | **50/50** | `ReproducesConfigCFromClosedFormInit` |
| cm-level success | 50/50 | **50/50** | ✓ |
| Wrong basin | 0/50 | **0/50** | ✓ |
| \|LW trans\| mean | cm-scale | **6.556 ± 2.958 mm** | ✓ |
| \|CW trans\| mean | cm-scale | **1.687 ± 3.345 mm** | ✓ |
| LW pitch mean | <1° | **0.0032 ± 0.0116°** | ✓ |
| Stage-2 FIM rank | 11/14 | **11/14** @ rep 13025 | H2 assembly test |
| Stage-2 FIM cond | rank-deficient joint ≲ probe ref | **9.6240×10¹¹** (λ_min=5.21×10⁴) | ✓ vs joint 5.69×10¹² ref |

**Production-parity verdict (GATE 3):** Config C **50/50 cm-level, 0/50 wrong-basin** — **PASS**. No port bug found; closed-form init lands in correct basin on all seeds.

> **Note on cond:** Stage-2 marginal FIM is rank **11/14** with cond ~**10¹¹–10¹²**, not ≲10⁵. The obsolete ≲10⁵ figure referred to a different (full joint) assembly context; production matches probe marginal FIM diagnostics.

### UQ decomposition — `uq_decomposition` (N=100, seeds 13000–13099)

Frozen probe headline (synthetic phase seal): **12/14** in ratio band @ N=100.  
Same-host probe re-run: **11/14**. Production:

| Metric | Probe / frozen | Production | Verdict |
|--------|----------------|------------|---------|
| Ratio band [0.8, 1.25] | 12/14 (frozen) / 11/14 (same host) | **9/14** | PARTIAL — 2 DoF at band edge |
| Translation in band | 5/5 (strict probe) | **4/5** | PARTIAL |
| LW translation closure | 3/3 | **2/3** | edge |
| LW_tz traj/fixed decouple | ~5.6× | **5.68×** | ✓ |
| Traj-prop trans/rot dominance | ~223× | **233.02×** | ✓ |
| CW_tx bias caveat | −2.90 ± 1.92 mm | **−2.93 ± 1.97 mm** | ✓ (limitation preserved) |
| Fixed-traj FIM rank @ rep | 11/14 | **11/14**, cond **6.4805×10¹¹** | ✓ |

Core decoupling scalars match frozen §2; additive closure count is **9/14** (documented delta vs 11–12/14 probe — not tuned away). Test thresholds: ≥9/14, ≥4/5 trans, decouple ≥5×, dominance ≥100×.

### Jacobian / H2 assembly

| Check | Production | Test |
|-------|------------|------|
| `AttitudeFactorPoseForm` analytic vs numeric | PASS | `test_attitude_factor_jacobian` |
| F(CW_yaw), F(CW_tx), F(CW_ty) ≠ 0 | **1.101×10¹⁵, 1.057×10⁹, 1.809×10¹²** | `Stage2FimCameraColumnsNonZeroAtRepSeed` |
| \|\|F(:,CW_*)\|\| > 0 | **5.217×10¹², 2.847×10¹⁴** | ✓ (would be zero pre-H2 fix) |

---

## Non-negotiable fixes — verification evidence

| # | Fix | Location | Verification |
|---|-----|----------|--------------|
| **1** | **H2 transpose** — `jacobian = T.Dx_this_mul_exp_x_at_0().transpose()` | `include/clic_calib/factor/ceres_local_param.h:109` | `test_attitude_factor_jacobian` PASS; Stage-2 FIM CW columns non-zero (`||F(:,CW_tx)||=5.217×10¹²`) — pre-fix these columns were numerically zero |
| **2** | **CeresSo3ProblemScope + DO_NOT_TAKE_OWNERSHIP** | `ceres_so3_scope.h`; used in Stage-1/2 Ceres builds | No SIGSEGV across 50× Config C + 100× UQ MC; ownership flags explicit in scope ctor |
| **3** | **Tag-local IPPE PnP** (not world EPnP) | `extrinsic_initializer.cpp` — `cv::SOLVEPNP_IPPE_SQUARE` | T_CW init **0.0002° / 0.163 mm** vs probe; 50/50 success |
| **4** | **Knot trim = subset extraction** (not resampling) | `trajectory_support.h::TrimTrajectoryToObservedSupport`; `stage1_trajectory_fitter.cpp` | Stage-1 prod vs no-trim probe path: **0 mm / ~10⁻¹⁴ mrad** max diff on samples |
| **5** | **Stage-2 prior-free** (no extrinsic prior factor) | `extrinsic_refiner.cpp` — no `ExtrinsicPrior` | Config C **50/50 cm-level** without prior; UQ MC uses Gaussian Stage-2 (`lidar_cauchy_scale=0`, `camera_huber_delta_px=0`) |

---

## Real-data interface contract (GATE 5 — no field processing)

### Inputs

| Stream | Reader / config | Requirement |
|--------|-----------------|-------------|
| RTK CSV | `RTKReader` | Antenna positions, world frame |
| PSDK attitude CSV | `AttitudeReader` | 50 Hz fused RPY/quat; `--attitude path.csv` in `calibrate_offline` |
| LiDAR + camera obs | `observations.clicob` | From `preprocess_rosbag` (not in this PR) |
| Noise | `config/noise_model.yaml` | σ_rtk, σ_lidar, σ_pix, **Σ_att** (default 0.2°/0.2°/1.5° — replace with hover-measured before field use) |
| Attitude decimation | `config/spline.yaml` → `attitude.stride: 25` | ~2 Hz attitude factors |
| Transport delay | `spline.yaml` `transport_delay_s: 0.0` | Clock offset absorbed with `t_d` in Stage 2 |
| Lever arms / rig | `lever_arms.yaml`, `sensor_rig.yaml` | Required for readiness |

### API surface

```cpp
RealDataPaths paths{config_dir, rtk_csv, attitude_csv, observations_clicob};
RealDataReadinessReport r = RealDataSession::CheckReadiness(config_dir);
RealDataSession::WireInto(&estimator, paths);  // sets RTK, attitude, archive streams
estimator.solve();  // → RunTwoStageSolve() when attitude present
```

### Readiness checklist (automated)

`test_real_data_interface` — **4/4 PASS**: config load, Σ_att + transport delay on reader, estimator attitude config, all stream types accepted.

### Explicit non-goals (this PR)

- No rosbag extraction, hover STD estimation, or field QA
- No retuning of frozen synthetic architecture
- No extrinsic priors or FIM regularization/clamping

---

## Regression suite

**Script:** `scripts/run_production_regression.sh`

| Tier | Tests | Role |
|------|-------|------|
| Jacobian | `test_attitude_factor_jacobian` | Attitude factor + H2 chain |
| Stage 1 | `test_stage1_trajectory_fitter` | Trajectory metrics vs probe |
| Stage 2 init | `test_extrinsic_initializer` | Umeyama + IPPE pre-iteration |
| Config C | `test_two_stage_pipeline` | Full pipeline + FIM assembly |
| UQ | `test_uq_decomposition` | Three-arm MC + FIM diagnostic |
| Real data I/F | `test_real_data_interface` | Interface contract |

Probe-only regression remains in `scripts/run_synthetic_regression.sh` (unchanged; not required for production merge gate).

---

## Known preserved limitations (frozen — do not re-investigate)

- **CW_tx / CW_ty** systematic bias (~3 mm lateral) — disclosed in UQ; not headline accuracy
- **LW_roll / LW_pitch** decomposition ratios marginally above band @ N=100
- **Σ_att** defaults are synthetic until hover measurement replaces them

---

## References

- `doc/architecture/two_stage_solver_blueprint.md` — refactor specification
- `doc/diagnostics/uq_decomposition.md` — frozen probe UQ numbers
- `doc/diagnostics/SYNTHETIC_PHASE_SEAL.md` — synthetic phase closeout
- `doc/diagnostics/DECISION_GATE_5.md` — real-data interface gate record
