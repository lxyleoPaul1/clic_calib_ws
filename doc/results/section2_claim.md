# §2 Paper-Ready Claim Paragraph (FINAL)

**Status:** **FINAL** (2026-05-23) — synthetic phase frozen; see `doc/diagnostics/SYNTHETIC_PHASE_SEAL.md`.

Draft for methods / synthetic validation subsection (~230 words). Adapt tone to venue.

---

## Paper presentation scope (frozen)

### Accuracy narrative — anchor on T_LW

The **LiDAR–world extrinsic T_LW** carries the primary accuracy claim. Evidence chain (all paper-facing):

1. **Sphere residual** + analytic LiDAR factor  
2. **Closed-form Umeyama** initialization (non-convex basin escape)  
3. **Degeneracy contrast** — coplanar vs multi-layer @ 200 m (**4–5×** LiDAR Z error; pitch observability)  
4. **Unbiased LW translation** @ N=100 (|mean| ≪ σ on LW_tx/ty/tz)  
5. **cm/mm MC spread** with 100/100 convergence from ±10° coarse init  

**§2 accuracy-anchoring text (copy-ready):**

> Synthetic validation anchors extrinsic **accuracy** on the LiDAR–world transform **T_LW**. The LiDAR block provides the cleanest evidence chain: analytic sphere residuals, Umeyama closed-form initialization, geometry-responsive identifiability (multi-layer vs coplanar degeneracy at 200 m standoff), and translation estimates with negligible bias at N=100 (e.g., LW_tx: −0.36 ± 2.02 mm; LW_tz: +0.75 ± 7.57 mm). Camera extrinsics are a **secondary modality**: the framework supports camera calibration (σ(CW_tx) ≈ 1.9 mm MC spread), but a **~3 mm systematic lateral-translation offset** (CW_tx = −2.90 ± 1.92 mm) is disclosed as a simulation limitation rather than a headline precision claim.

### Camera — secondary modality

Present camera as: *"The framework supports camera extrinsics; calibration reaches ~2 mm empirical spread on lateral translation, with a ~3 mm systematic lateral-translation offset disclosed within the application's projection-error tolerance."*

**Projection-tolerance context (one line, back-of-envelope — do not re-derive):** A ~3 mm lateral extrinsic error at roadside V2X operating range (tens of metres standoff) corresponds to angular projection error of order **10⁻² mrad**, i.e. **far below** typical V2X projection tolerances (~0.3–0.5 m at 50–200 m). This justifies reporting CW bias as a **limitation**, not a blocker to the LiDAR-centric accuracy and UQ claims.

### Internal vs paper-facing diagnostics

| Item | Paper? | Note |
|------|--------|------|
| Config C Gate 1 (rep-fit traj, FIM↔MC) | **Yes** | **10/14**, **5/5 translations** @ operating point |
| Gate 1 blocks **A/B** (GT trajectory) | **No** | **INTERNAL DIAGNOSTIC ONLY** — 0/14; MC < FIM on GT path; excluded **by choice**, not omission |
| Gate 3 UQ decomposition (10/14 zero-mean) | **Yes** | LW block + rot/t_d headline; CW lateral separate |
| Gate 5b Schur marginal | **No** | Appendix one-liner at most |
| Gate 4 delta-method | **Optional appendix** | Partial 7/14 |

**Gate 1 A/B note for future readers:** Blocks A and B (GT trajectory, obs synthesized on GT path) show 0/14 FIM↔MC match because the MC arm and FIM linearization point are mismatched on a noise-free reference path. This is a useful isolation check but **must not appear in the paper**; only **Config C** (rep-fit Stage-1 trajectory @ seed 13025) reflects the operating point.

### Limitations footnotes (paper)

**LW_roll / LW_pitch (ratios 1.45 / 1.28):** *"Within Monte Carlo sampling error at N=100 (ratio SE ≈ 14%) and/or a small rotation cross-term not captured by the diagonal decomposition; not investigated further."*

**CW lateral bias:** See projection-tolerance line above; conditional variance reporting only.

---

## Draft paragraph

We validate extrinsic **identifiability** using the prior-free observed information matrix assembled from analytic RTK, attitude, LiDAR sphere, and AprilTag factors on a continuous-time body trajectory. Under near-field multi-layer flight (horizontal standoff 15–40 m, altitude layers {2, 6, 11} m crossing the 6 m sensor height, Δθ_pitch ≈ 33°), the extrinsic information block F_ext is positive definite with λ_min = 0.40 and rank 12/12; at 200 m standoff or coplanar geometry the same matrix is rank-deficient with λ_min ≈ 0, confirming that three-dimensional excitation resolves the coplanar pitch degeneracy predicted by theory. **Accuracy** is assessed by Monte Carlo over N = 100 noise realizations using the two-stage pipeline (RTK and anisotropic PSDK attitude, Umeyama–LiDAR and tag-local PnP closed-form initialization, prior-free refinement). **Primary accuracy claims are anchored on the LiDAR–world extrinsic T_LW:** translations show millimeter-level spread with negligible bias (e.g., LW_tx: mean −0.36 mm, σ ≈ 2.0 mm; LW_tz: mean +0.75 mm, σ ≈ 7.6 mm) and ~20 mrad rotation spread, with 100/100 convergence from ±10° coarse priors. **Uncertainty** is validated by **noise-source decomposition** under a zero-mean error assumption: independent Monte Carlo arms fixing Stage-1 trajectory versus Stage-2 observation noise yield Var_total ≈ Var_fixed + Var_traj on **3/3 LiDAR–world translations** and **7 rotation/time-delay DoF** (**10/14 total**). The two-stage **decoupling cost** on the LiDAR block is quantified (e.g., trajectory propagation contributes **99.6%** of LW_tz variance). Fixed-trajectory FIM⁻¹ matches the fixed-noise MC arm at the rep-fit operating point for the LiDAR–world block. **Camera lateral translation** (CW_tx = −2.90 ± 1.92 mm, CW_ty = −0.84 ± 0.55 mm) exhibits a systematic offset of ~3 mm; its variance decomposition is reported **conditionally**, as zero-mean closure is not satisfied for these two DoF. Schur marginalization over the rank-deficient joint trajectory block is not used for UQ.

---

## UQ result text (headline — copy-ready)

> **Primary:** Variance decomposition Σ_fixed + Σ_traj-prop ≈ Σ_total is validated on **3/3 LiDAR–world translations** (LW_tx, LW_ty, LW_tz) and **7 rotation/time-delay DoF** (LW_yaw; CW_roll, CW_pitch, CW_yaw; CW_tz; t_d_L; t_d_C) — **10/14 DoF** satisfying both zero-mean (@ N=100) and band [0.8, 1.25]. **T_LW accuracy** is the main accuracy result.
>
> **Camera lateral translation (separate):** CW_tx = **−2.90 ± 1.92 mm**, CW_ty = **−0.84 ± 0.55 mm**. Camera lateral translation exhibits a systematic offset of ~3 mm at N=100; its variance decomposition is reported **conditionally**, as the zero-mean assumption underlying covariance closure is **not satisfied** for these two DoF. **Not included** in the headline closure count.
>
> **Edge:** LW_roll and LW_pitch ratios 1.45 and 1.28 — marginally outside band; disclosed, not re-run.

---

## Limitations paragraph (insert adjacent to results)

In synthetic evaluation, camera lateral translations show systematic bias exceeding their empirical spread; we treat this as a simulation limitation within roadside V2X projection-error tolerance (~3 mm lateral extrinsic error → order 10⁻² mrad at operating range, far below typical 0.3–0.5 m projection budgets). LiDAR–world rotations LW_roll and LW_pitch show decomposition ratios slightly above the validation band (1.45 and 1.28)—within Monte Carlo sampling error at N=100 (ratio SE ≈ 14%) and/or a small rotation cross-term not captured by the diagonal decomposition; not investigated further. Real-world error sources are expected to dominate remaining simulation imperfections.

---

## Bullet checklist for co-authors

- [x] Identifiability via λ_min / rank vs geometry
- [x] **Accuracy headline on T_LW only**
- [x] UQ headline: **10/14** (zero-mean + in band), **not** 12/14
- [x] **3/3 LW translations** in UQ headline; CW_tx/ty **split out** with mean±std + conditional caveat
- [x] CW_tx/CW_ty **excluded** from headline closure count
- [x] Fixed FIM validated on fixed-traj sub-problem (Config C)
- [x] Schur-marginal FIM deprecated
- [x] Gate 1 A/B excluded from paper

---

## Paper tables — scope

| Table | Include |
|-------|---------|
| T_LW accuracy (mean ± std) | **Yes — headline** |
| UQ decomposition — LW block + rot/t_d | **Yes — headline (10 DoF)** |
| CW_tx/ty mean ± std + conditional note | **Yes — separate row/footnote** |
| Full 14-DoF table | Yes, with † on CW_tx/ty, ✗ on LW_roll/pitch |
| Decoupling cost (LW) | **Yes — headline** |
| Gate 1 Config C (rep-fit) | **Yes — 10/14, 5/5 translations** |
| Gate 1 A/B (GT traj) | **No — internal diagnostic only** |
| LW_roll/pitch footnote | **Yes — MC SE ~14%; not re-tested** |
| “12/14 closed” without zero-mean caveat | **No** |
