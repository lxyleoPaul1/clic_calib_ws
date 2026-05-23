# Synthetic Phase Closeout — FINAL

**Status:** **FINAL** (2026-05-23)  
**Branch:** `probe/uq-decomposition`  
**Authoritative seal:** `doc/diagnostics/SYNTHETIC_PHASE_SEAL.md`

*(Formerly "PROVISIONAL_CLOSEOUT" — premature "UQ deferred" language withdrawn and superseded.)*

---

## Final synthetic verdict

The synthetic backend is **validated end-to-end** and **frozen**:

- Identifiability (geometry-responsive λ_min)
- Config C **T_LW** cm/mm recovery (closed-form init + refine)
- σ_yaw flat
- §2 UQ via noise-source decomposition (**10/14** zero-mean-qualified; **3/3 LW translations** + 7 rot/t_d)

**No further synthetic tuning.** Next: **real-data validation** + **`CalibrationEstimator` refactor**.

---

## Honest limitations (disclosed, not pursued)

See `doc/results/synthetic_evaluation.md` **§8** for the consolidated list (camera ~3 mm lateral bias, decoupling cost quantified, LW_roll/pitch edge ratios, MC-authoritative UQ, all-synthetic scope).

---

## Artifacts

| Document | Role |
|----------|------|
| `SYNTHETIC_PHASE_SEAL.md` | Binding boundaries + handoff |
| `uq_decomposition.md` | Full UQ tables |
| `section2_claim.md` | Paper paragraphs |
| `synthetic_evaluation.md` | Evaluation + §8 limitations |
| `two_stage_solver_blueprint.md` | Refactor spec (next) |

---

**END FINAL CLOSEOUT.**
