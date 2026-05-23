# DECISION GATE — Synthetic Phase FINAL

**Branch:** `probe/uq-decomposition`  
**Date:** **FINAL** 2026-05-23  
**Status:** **FINAL / FROZEN** — synthetic phase sealed; handoff to refactor + real data

---

## All synthetic-phase docs — FINAL (2026-05-23)

| Document | Status |
|----------|--------|
| `doc/diagnostics/uq_decomposition.md` | **FINAL** |
| `doc/results/synthetic_evaluation.md` | **FINAL** |
| `doc/results/section2_claim.md` | **FINAL** |
| `doc/diagnostics/synthetic_phase_summary.md` | **FINAL** |
| `doc/architecture/two_stage_solver_blueprint.md` | **FINAL** (blueprint; next implementation task) |
| `doc/diagnostics/SYNTHETIC_PHASE_SEAL.md` | **FINAL** |
| `doc/diagnostics/PROVISIONAL_CLOSEOUT.md` | **FINAL** (legacy filename) |
| `doc/diagnostics/uq_noise_decomposition.md` | **FINAL** (pointer doc) |
| `doc/diagnostics/DECISION_GATE_0.md` | **FINAL** |

**No experiments run. No code changed** in this seal pass (documentation only).

---

## Final headline phrasing (literally true)

- **Accuracy:** **T_LW** (LiDAR–world) — cm/mm MC, negligible LW translation bias @ N=100.
- **UQ:** Σ_fixed + Σ_traj-prop ≈ Σ_total on **3/3 LiDAR–world translations** + **7 rotation/time DoF** = **10/14** (zero-mean-qualified @ N=100).
- **Camera lateral (separate, not in headline count):** CW_tx = **−2.90 ± 1.92 mm**, CW_ty = **−0.84 ± 0.55 mm** — conditional reporting; ~3 mm systematic offset disclosed as limitation.
- **Edge:** LW_roll/pitch ratios 1.45 / 1.28 — limitations footnote.
- **Diagnostic only (do not headline):** 12/14 ratio band if zero-mean ignored (includes biased CW_tx/ty).

---

## Handoff paragraph

The synthetic phase is **complete and frozen** (2026-05-23). **Validated:** near-field extrinsic identifiability; Config C two-stage **T_LW** recovery to cm/mm accuracy from coarse init; σ_yaw insensitivity; §2 UQ via noise-source decomposition (**10/14** zero-mean-qualified, **3/3 LW translations** + 7 rot/t_d) with decoupling cost quantified. **Disclosed limitations (not chased):** camera lateral ~3 mm bias (within V2X projection tolerance); LW_roll/pitch edge ratios; partial analytic cross-check (7/14); all-synthetic scope. **Next:** **`CalibrationEstimator` refactor** per blueprint (no PR #1 changes) and **real-world experiments**.

---

**END — SYNTHETIC PHASE FINAL. STOP.**
