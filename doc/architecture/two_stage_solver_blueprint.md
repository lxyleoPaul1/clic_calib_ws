# Two-Stage Solver Blueprint (Refactor Specification)

**Purpose:** Precise specification for refactoring `CalibrationEstimator` from validated probe code.  
**Source branch:** `probe/uq-decomposition` (probe-only; **do not** merge probe tests into production as-is).  
**Status:** **FINAL** (2026-05-23) — frozen blueprint; **next task** after synthetic phase seal (`doc/diagnostics/SYNTHETIC_PHASE_SEAL.md`).

---

## Architecture overview

```text
┌─────────────────────────────────────────────────────────────┐
│  STAGE 1 — Trajectory (continuous-time SE(3) B-spline)      │
│  Observations: RTK position, PSDK attitude, smoothness      │
│  Output: fixed trajectory θ̂ for Stage 2                    │
└──────────────────────────┬──────────────────────────────────┘
                           │ trajectory frozen
┌──────────────────────────▼──────────────────────────────────┐
│  STAGE 2 — Extrinsics + time delays                         │
│  Init: Umeyama (LiDAR) + tag-local IPPE PnP (camera)        │
│  Refine: {T_LW, T_CW, t_d^L, t_d^C} prior-free on fixed traj│
└─────────────────────────────────────────────────────────────┘
```

**Validated property:** cm-level extrinsic accuracy (MC).  
**UQ (validated):** Noise-source decomposition MC — Σ_total ≈ Σ_fixed + Σ_traj-prop (12/14 @ N=100). Fixed-trajectory FIM⁻¹ matches Σ_fixed on Config C. **Do not** use Schur-marginal FIM on rank-deficient joint F_θθ for UQ.

---

## Stage 1 — Trajectory estimation

### Inputs

| Stream | Specification |
|--------|---------------|
| RTK position | Antenna position in world frame; lever arm **L_{B→A}**; diagonal Σ_rtk from receiver (σ_h ~ 1 cm, σ_v ~ 2 cm synthetic) |
| PSDK attitude | Fused roll/pitch/yaw at **50 Hz** on **flight-controller clock** (same clock as RTK after sync) |
| Σ_att | Anisotropic diag(σ_roll², σ_pitch², σ_yaw²); synthetic σ_roll = σ_pitch = **0.2°**, σ_yaw = **1.5°** |
| Smoothness | α_p, α_R on spline acceleration (synthetic: **0.01**) |

**Critical:** RTK-only Stage-1 yields **~130°** orientation error; attitude stream is **mandatory**.

### Three-pass scheme (validated)

Probe: `FitStage1WithAttitude()` in `two_stage_probe_common.hpp`.

1. **Pass 1 — Attitude:** Fit SO(3) knots from PSDK attitude factors; identity or chord-length rotation init (not GT).
2. **Pass 2 — Reseed positions:** Initialize R³ knots from RTK antenna positions propagated through L_{B→A} and current rotation.
3. **Pass 3 — RTK with fixed rotation:** Add RTK position factors with rotation knots **fixed** (or strongly observed); optional Ceres polish.

### Spline configuration

| Parameter | Rule |
|-----------|------|
| `knot_dt` | `max(0.05 s, min_observation_interval)` |
| Support trim | Drop knots outside observation time support (H3 fix) |
| Order | Basalt-style SE(3) split SO(3)+R³, order 4 |

### Probe → production mapping

| Probe symbol | Target in `CalibrationEstimator` |
|--------------|----------------------------------|
| `FitStage1WithAttitude` | `TrajectoryStage::Estimate()` or equivalent |
| `BuildNearFieldFimNoisyScenarioWithAttitude` | **Test only** — replace with rosbag/archive reader |
| Attitude factor residual | `AttitudeFactor` in `factor/analytic_diff/` |
| RTK factor | existing RTK / position factor |
| Spline views | reuse `So3SplineView`, `RdSplineView` |

### Production changes required

- Wire PSDK attitude stream from preprocessed archive (not optional flag).
- Enforce **single clock** for RTK + attitude; estimate transport delay separately from t_d.
- Apply obs-support trimming before FIM or solve (prevents dangling knots → singular F_θθ).

---

## Stage 2 — Extrinsic calibration

### State vector (14 DoF at FIM level)

Order: `[t_d_L, LW_rot×3, LW_trans×3, t_d_C, CW_rot×3, CW_trans×3]`.

Lie groups: `T_LW`, `T_CW` via `LieLocalParameterization` with **DO_NOT_TAKE_OWNERSHIP** (SIGSEGV fix — parameter blocks owned by caller).

### Closed-form initialization (required)

Probe: `closed_form_init_common.hpp`.

| Sensor | Method | Probe function |
|--------|--------|----------------|
| LiDAR | **Umeyama 3D–3D** on `{p_G^L} ↔ {p_G^W}` from sphere centers | `SolveUmeyamaTLW()` / `MakeGeometricExtrinsicInit()` |
| Camera | **Tag-local IPPE PnP** on AprilTag corners | `SolveTagLocalPnPTCW()` |

**Do not use world-frame EPnP** for init — planar mirror ambiguity at near-field standoff.

Per-scan: sphere center in L → world via Stage-1 traj; tag corners → tag frame PnP.

### Refinement

Probe: `SolveStage2Extrinsics()` in `two_stage_probe_common.hpp`.

- Fix Stage-1 trajectory knots.
- Optimize `{T_LW, T_CW, t_d^L, t_d^C}` with sphere implicit + AprilTag reprojection factors.
- **Prior-free** on extrinsics (no extrinsic prior factor in validated Config C).
- `t_d` bounds: yaml nominal ± `t_d_max_abs_s`.

### Probe → production mapping

| Probe symbol | Target |
|--------------|--------|
| `MakeGeometricExtrinsicInit` | `ExtrinsicInitializer::FromGeometric()` |
| `SolveStage2Extrinsics` | `CalibrationEstimator::RefineExtrinsics()` |
| `Stage2ExtrinsicInformation` | `ObservabilityReport::FixedTrajectoryFim()` (diagnostic only) |
| `SphereImplicitFactor` | existing / ported analytic factor |
| `AprilTagReprojFactor` | existing / ported analytic factor |

### Must preserve from probe fixes

1. **H2 transpose** in `ceres_local_param.h` — `LieLocalParameterization` Jacobian layout.
2. **Shared ownership** pattern for Lie parameterization blocks.
3. Tag-local PnP init path (not world EPnP).

---

## Regression tests (must pass post-refactor)

Run via `scripts/run_synthetic_regression.sh`.

| Gate | Test filter | Criterion |
|------|-------------|-----------|
| Factor Jacobians | `test_*_factor_jacobian` | All analytic vs numeric pass |
| Config C cm-level | `TwoStageClosedFormInit.Gate3BasinDiscriminationLadder` | Config C ≪ A/B error |
| Config C MC | `TwoStageClosedFormInit.Gate5FimMcClosedFormStage2` | 50/50 converged; cm-level emp std |
| Identifiability | `FimAssemblyFix.Gate3FinalRoutingNearFieldVs200m` | Near-field λ_min > 0, rank 12/12 |
| σ_yaw flat | `TwoStageClosedFormInit.Gate4YawSensitivitySweep` | No σ_yaw bottleneck |
| Degeneracy | `Step5Degeneracy.DistantTargetReprojectionErrorVsRange` | Coplanar/multi ratio > 1 @ 200 m |

### Diagnostic only (not green-gate)

| Test | Label |
|------|-------|
| `Gate3FimMcSideBySideComparison` | FIM↔MC known limitation |
| `Gate5MarginalFimMcComparison` | Marginal FIM diagnostic |
| `FimMonteCarloConsistency.*` | Legacy joint FIM↔MC |

---

## Known carry-over limitations

| Issue | Impact | Mitigation |
|-------|--------|------------|
| F_θθ singular / indefinite at decoupled point | Schur marginal FIM invalid | Use MC noise-source decomposition (validated); fixed FIM for fixed sub-problem only |
| knot_dt vs obs gaps | Dangling knots → rank loss | `knot_dt = max(0.05, min_obs_dt)` + trim |
| Two-stage decoupling | Total MC > fixed FIM | Σ_traj-prop term (quantified per DoF in `uq_decomposition.md`) |
| Joint solve ill-conditioning | 14 m / 61 m failures | Stay two-stage for production |

---

## Suggested production module layout

```text
calibration/
  trajectory_stage.{h,cpp}      ← FitStage1WithAttitude
  extrinsic_initializer.{h,cpp} ← Umeyama + tag-PnP
  extrinsic_refiner.{h,cpp}     ← SolveStage2Extrinsics
  calibration_estimator.{h,cpp}  ← orchestration (replaces monolith)
  observability.{h,cpp}         ← FIM rank diagnostics (optional, not UQ)
```

**END OF BLUEPRINT.**
