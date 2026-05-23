# Synthetic Phase Summary — Debug Arc & Resolutions (FINAL)

**Status:** **FINAL** (2026-05-23) — synthetic phase frozen; see `doc/diagnostics/SYNTHETIC_PHASE_SEAL.md`.

One-page chronology for lab onboarding and paper appendix (“lessons learned”).

**Outcome:** Two-stage architecture **validated** for identifiability + **T_LW** cm-level accuracy. **§2 UQ validated** via noise-source decomposition (**12/14 DoF**, **5/5 translations** @ N=100). **Synthetic backend frozen** — no further tuning before ICRA.

---

## Timeline

| Step | Symptom | Root cause | Resolution |
|------|---------|------------|------------|
| **0 — Baseline** | Joint / early pipeline huge errors | Multiple compounding issues | Structured probe suite |
| **1 — Prior inconsistency** | Convention A vs B FIM disagree | Extrinsic prior vs pure obs | Convention B (prior-free obs) for §2 |
| **2 — H2 transpose bug** | F_ext rank 11/14 despite good factors | Jacobian transpose in assembly | Fix `ceres_local_param.h`; rank → 12/12 near-field |
| **3 — F_θθ dangling knots** | Trajectory FIM singular | knot_dt mismatch; unsupported knots | `knot_dt = max(0.05, min_obs_dt)` + obs-support trim |
| **4 — Missing attitude** | RTK-only traj yaw ~130° | Orientation unobservable without PSDK | Mandatory 50 Hz anisotropic attitude |
| **5 — Non-convex basin** | ±10° coarse init → meter errors | Stage-2 cost basin | Umeyama + tag-local PnP closed-form init |
| **6 — t_d boundary pin** | t_d stuck at ±1000 ms | Wrong init / gauge | Yaml nominal pairing + refine |
| **7 — FIM↔MC gap** | Fixed FIM 6/14 vs total MC | Two-stage: traj noise not in fixed FIM | **Explained** by Σ_traj-prop term |
| **8 — Marginal FIM** | Marginal 3/14 (worse than fixed 6/14) | Schur on **rank-deficient F_θθ** | **Withdrawn** — use MC decomposition |
| **9 — UQ decomposition** | Close total covariance | Three-arm MC @ N=100 | **CONFIRMED** 12/14; 5/5 translations |
| **10 — Seal** | Claim scope vs known bias | CW_tx/ty bias; LW_roll/pitch edge | **Disclosed**; synthetic **frozen** |

---

## What each gate proved

| Gate | Question | Answer | In paper? |
|------|----------|--------|-----------|
| Gate 1 Config C | Fixed FIM for sub-problem? | **10/14**, 5/5 trans | **Yes** |
| Gate 1 A/B | GT traj FIM↔MC? | 0/14 | **No — internal** |
| Gate 2 | Traj-prop translation-dominant? | 223× trans/rot | **Yes** |
| Gate 3 | Σ_total ≈ Σ_fixed + Σ_traj? | 12/14; 5/5 trans | **Yes** |
| Gate 5b | Schur marginal UQ? | 3/14 — invalid | Appendix note only |
| Gate 4 | Delta-method? | 7/14 partial | Optional appendix |

---

## Claim boundaries (FINAL)

| ✅ Headline (paper) | ⚠️ Disclosed limitation | ❌ Not claimed |
|--------------------|-------------------------|----------------|
| F_ext identifiability vs geometry | CW_tx **−2.9 mm**, CW_ty **−0.8 mm** bias | Sub-mm camera lateral accuracy |
| **T_LW** cm/mm MC accuracy | LW_roll/pitch ratios 1.45/1.28 | Schur-marginal FIM as UQ |
| Closed-form init necessity | CW decomposition rows (zero-mean caveat) | Joint MAP production |
| Noise-source UQ (5/5 trans) | | Gate 1 A/B GT traj |
| LW decoupling cost quantified | | Fixed FIM = total MC |
| σ_yaw insensitivity | | |
| Coplanar degeneracy @ 200 m | | |

---

## Artifacts

| Document | Path |
|----------|------|
| **Seal (binding)** | `doc/diagnostics/SYNTHETIC_PHASE_SEAL.md` |
| UQ decomposition | `doc/diagnostics/uq_decomposition.md` |
| Paper paragraph | `doc/results/section2_claim.md` |
| Refactor blueprint | `doc/architecture/two_stage_solver_blueprint.md` |

---

## Next task (post-seal only)

1. **Real-data campaign**
2. **Refactor `CalibrationEstimator`** — **do not** touch PR #1

**END OF PHASE SUMMARY.**
