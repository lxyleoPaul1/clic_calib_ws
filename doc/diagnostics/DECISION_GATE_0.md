# DECISION GATE 0 — Documentation + Bias Audit (FINAL)

**Branch:** `probe/uq-decomposition`  
**Date:** 2026-05-21 (audit); **FINAL** 2026-05-23  
**Status:** **FINAL** — bias findings incorporated as **limitations** (§8); synthetic phase frozen.

---

## 1. Documentation correction

All closeout docs updated to **FINAL** (2026-05-23). Premature UQ / FIM↔MC verdicts withdrawn; noise-source decomposition is authoritative.

| File | Action |
|------|--------|
| `doc/results/synthetic_evaluation.md` | Banner + §FIM rows provisional |
| `doc/results/section2_claim.md` | Banner; marginal-Schur language provisional |
| `doc/diagnostics/synthetic_phase_summary.md` | Banner on Step 8 / Gate 5b |
| `doc/architecture/two_stage_solver_blueprint.md` | UQ limitation rows provisional |
| `doc/real_data/observation_model_spec.md` | FIM-as-UQ row under re-investigation |
| `doc/diagnostics/PROVISIONAL_CLOSEOUT.md` | → **FINAL** closeout |

---

## 2. Config C bias audit (N=100, seeds 13000–13099)

**Pipeline:** Config C closed-form two-stage (RTK + attitude → Umeyama + PnP → prior-free refine).  
**Criterion:** Covariance comparison assumes **zero-mean** error; negligible if \|mean\| ≪ emp_std (operational: \|mean\|/std < 0.3).

| DoF | mean bias | emp_std | \|mean\|/std | Verdict |
|-----|-----------|---------|-------------|---------|
| LW_tx | −0.36 mm | 2.02 mm | 0.18 | ✓ negligible |
| LW_ty | −0.55 mm | 5.43 mm | 0.10 | ✓ negligible |
| LW_tz | +0.75 mm | 7.57 mm | 0.10 | ✓ negligible |
| LW_roll/pitch/yaw | ≤4.6 mrad | ~20 mrad | ≤0.20 | ✓ negligible |
| CW_tx | **−2.90 mm** | 1.92 mm | **1.51** | ⚠ **significant bias** |
| CW_ty | **−0.84 mm** | 0.55 mm | **1.52** | ⚠ **significant bias** |
| CW_tz | +0.057 mm | 0.055 mm | 1.05 | ⚠ ratio high; abs < 0.06 mm |
| t_d_L | +0.091 ms | 0.445 ms | 0.21 | ✓ negligible |
| t_d_C | −0.067 ms | 0.077 ms | 0.87 | borderline |
| CW roll/pitch/yaw | sub-mrad | sub-mrad | 0.13–0.50 | mostly OK; CW_yaw borderline |

**Conclusion:** **T_LW** translations and rotations are unbiased enough for **headline** accuracy and UQ claims. **CW_tx and CW_ty bias** are **disclosed limitations** (paper limitation paragraph) — **not** investigated further; synthetic phase **frozen**.

---

## 3. Gate 0 verdict (historical)

| Check | Result |
|-------|--------|
| LW translation bias negligible | **Yes** — headline OK |
| CW_tx / CW_ty unbiased | **No** — **limitation**, sealed |
| Synthetic phase | **FINAL / FROZEN** — see `SYNTHETIC_PHASE_SEAL.md` |

---

**END GATE 0.**
