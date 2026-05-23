# Synthetic Phase — FINAL / FROZEN (ICRA Submission Boundary)

**Branch:** `probe/uq-decomposition`  
**Date sealed:** **2026-05-23**  
**Status:** **FINAL / FROZEN** — no further synthetic experiments or tuning before submission.

---

## Handoff paragraph (STATUS summary)

The synthetic phase is **complete and frozen** (2026-05-23). **Validated:** near-field extrinsic identifiability (geometry-responsive rank/λ_min); Config C two-stage recovery of **T_LW** to cm/mm accuracy from coarse init (Umeyama + tag-local PnP, 100/100 @ N=100); σ_yaw insensitivity; and **§2 UQ** via three-arm noise-source decomposition — **3/3 LiDAR–world translations** plus **7 rotation/time DoF** (**10/14** zero-mean-qualified) with decoupling cost quantified (e.g., LW_tz: 99.6% variance from trajectory propagation). **Disclosed limitations (not chased):** camera lateral translation systematic bias (~3 mm; within V2X projection tolerance; root cause not isolated); LW_roll/pitch decomposition ratios marginally out of band; analytic delta-method partial (7/14); all results synthetic. **Next:** **`CalibrationEstimator` refactor** per `two_stage_solver_blueprint.md` (do not touch PR #1) and **real-world experiments** — the actual submission gating work.

---

## Strategic decision (binding)

The synthetic backend is **complete and frozen**. Remaining ICRA work is **real-world experiments** and **`CalibrationEstimator` refactor** — not synthetic-precision polishing.

Known imperfections are **honestly disclosed** in `synthetic_evaluation.md` §8; they are **not** investigated or fixed on this branch.

---

## What §2 may claim (headline)

| Claim | Scope | Evidence |
|-------|-------|----------|
| **Identifiability** | F_ext rank / λ_min vs geometry | Near-field 12/12; 200 m / coplanar degeneracy |
| **Accuracy (primary)** | **T_LW** | MC N=100; \|bias\| ≪ σ on LW translations |
| **Recovery pipeline** | Two-stage + closed-form init | Config C 100/100 |
| **§2 UQ** | Noise-source decomposition | **10/14** zero-mean; **3/3 LW trans**; CW lateral separate |
| **Fixed FIM sub-problem** | Σ_fixed @ rep-fit | Gate 1 Config C |

---

## Authoritative numbers (N=100, rep=13025)

**UQ headline:** 10/14 (zero-mean + in band): 3 LW trans + 7 rot/t_d.  
**Separate:** CW_tx −2.90 ± 1.92 mm, CW_ty −0.84 ± 0.55 mm.  
**Edge:** LW_roll 1.45, LW_pitch 1.28.

---

## Document index (all FINAL 2026-05-23)

| File | Status |
|------|--------|
| `doc/diagnostics/uq_decomposition.md` | FINAL |
| `doc/results/synthetic_evaluation.md` | FINAL |
| `doc/results/section2_claim.md` | FINAL |
| `doc/diagnostics/synthetic_phase_summary.md` | FINAL |
| `doc/architecture/two_stage_solver_blueprint.md` | FINAL |
| `doc/diagnostics/PROVISIONAL_CLOSEOUT.md` | FINAL (renamed scope) |

---

## Next work (post-seal only)

1. **Real-data campaign** — `doc/real_data/observation_model_spec.md`
2. **`CalibrationEstimator` refactor** — **do not** modify PR #1

**No further commits on this branch for synthetic tuning.**

---

**END SEAL.**
