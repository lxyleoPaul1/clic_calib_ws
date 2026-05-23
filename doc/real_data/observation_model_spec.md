# Real-Data Observation Model Specification

> **FINAL (2026-05-23):** Synthetic §2 UQ validated via noise-source decomposition (`doc/diagnostics/uq_decomposition.md`). Field campaign is the submission gating work.

Consolidates synthetic-phase **requirements** so the real-data campaign does not rediscover known failure modes.

**Prerequisite:** Refactor `CalibrationEstimator` per `doc/architecture/two_stage_solver_blueprint.md`.

---

## 1. Mandatory observation streams

### 1.1 RTK position

| Item | Requirement |
|------|-------------|
| Quality | **FIXED** or better; reject FLOAT for calibration passes |
| Rate | ≥ 10 Hz usable; 20 Hz preferred |
| Covariance | Use reported σ_h, σ_v (typ. **1 cm / 2 cm**) — do not unit-scalar inflate |
| Lever arm | **L_{B→A}** measured (total station); apply consistently in RTK factor |

### 1.2 PSDK fused attitude — **NOT optional**

Synthetic proof: RTK-only Stage-1 → **~130°** mean orientation error (`TwoStageProbe.Step1Stage1TrajectoryRtkOnly`).

| Item | Requirement |
|------|-------------|
| Rate | **50 Hz** on flight-controller clock |
| Alignment | Same clock as RTK (**zero relative offset** after sync); transport delay estimated separately |
| Σ_att | Anisotropic diagonal — **measure on hover** per axis |

**Recommended nominal (validate on hover):**

| Axis | σ (deg) |
|------|---------|
| roll | **0.2** |
| pitch | **0.2** |
| yaw | **1.0 – 2.0** |

Factor stride: ~2 Hz on 50 Hz stream (synthetic: stride 25) sufficient if smoothness regularizes.

### 1.3 LiDAR — reflective sphere

| Item | Requirement |
|------|-------------|
| Target | Single reflective sphere; one **p_G^L** per scan via sphere fit |
| Noise | σ_r ~ **2 cm** (adjust from target size/residual QA) |
| Lever arms | **L_{B→G}**, **L_{G→M_j}** from total station |

### 1.4 Camera — AprilTag tetrahedral markers

| Item | Requirement |
|------|-------------|
| Detection | Four corners per tag; outlier rejection |
| Init geometry | **Tag-local IPPE PnP** — **not** world EPnP (mirror ambiguity) |
| Noise | σ_pix ~ **1 px** |
| Intrinsics | Pre-calibrated K, dist; frozen during extrinsic pass |

---

## 2. Time synchronization

| Quantity | Handling |
|----------|----------|
| RTK ↔ attitude | Common FC clock; **no** ad-hoc constant offset without estimation |
| Transport delay | Constant time offset (vehicle vibration path) — **estimate** |
| t_d^L, t_d^C | Sensor-vs-FC clock delays — **estimate** in Stage 2 (yaml nominal as init only) |
| World time pairing | LiDAR/camera timestamps paired to world time via t_d at observation |

---

## 3. Flight design (identifiability)

Validated near-field geometry (`FimAssemblyFix.Gate3FinalRoutingNearFieldVs200m`):

| Parameter | Value |
|-----------|-------|
| Horizontal standoff | **15 – 40 m** |
| Altitude layers | **{2, 6, 11} m** with sensor height **h_s ≈ 6 m** (layers **cross** sensor height) |
| Azimuth spread | **±30°** |
| Layer duration | ~15 s per layer (~45 s total minimum) |
| Target Δθ_pitch @ sensor | **20 – 33°** (synthetic achieved **33.4°**) |

**Avoid:** coplanar single-altitude orbits @ 200 m — λ_min ≈ 0, pitch unobservable.

---

## 4. Initialization strategy

| Do | Don't |
|----|-------|
| Umeyama on LiDAR sphere centers | Manual coarse ±10° yaml only |
| Tag-local PnP on AprilTag corners | World-frame EPnP init |
| Then prior-free Ceres refine | Skip geometric init |

Coarse yaml may seed **t_d** only; extrinsic rotation must come from closed-form geometry.

---

## 5. Pre-flight calibration checklist

- [ ] Hover log: estimate σ_roll, σ_pitch, σ_yaw from PSDK
- [ ] Verify RTK FIXED ratio > 95% on calibration run
- [ ] Confirm lever arms L_{B→A}, L_{B→G}, L_{G→M} loaded
- [ ] Confirm knot_dt ≥ max(0.05 s, min obs interval)
- [ ] Confirm three altitude layers cross h_s
- [ ] Magnetometer / steel structure check near mount (yaw may exceed 2° near gantries)

---

## 6. Open real-data risks (from synthetic findings)

| Risk | Synthetic evidence | Mitigation |
|------|-------------------|------------|
| Uneven / dropped obs | F_θθ rank 54/554, indefinite λ_min | Obs-support trim; knot_dt rule; gap detection |
| FLOAT / multipath RTK | Not fully swept synthetically | Reject epochs; inflate Σ or segment |
| Magnetometer yaw near steel | σ_yaw nominal 1–2° may be low | Measure yaw on-site; avoid hovering beside gantries |
| Missing attitude | 130° traj yaw error | **Hard fail** if attitude stream absent |
| FIM-as-UQ in field | Schur FIM↔MC not closed (joint) | Config C: MC decomposition UQ **validated**; fixed FIM for fixed sub-problem; no Schur-marginal pass/fail |

---

## 7. Deliverables for first real-data pass

1. Preprocessed archive: RTK, attitude, sphere centers, tag corners (time-aligned).
2. Stage-1 trajectory QA: RMS vs RTK < 3 cm; attitude residual ≈ Σ_att.
3. Stage-2 extrinsic report: \|ΔT\|, pitch error, t_d — UQ via field MC repeatability + decomposition protocol (`uq_decomposition.md`); not Schur-marginal FIM.
4. Optional: repeatability flight → empirical std (bucket B only).

**END OF OBSERVATION MODEL SPEC.**
