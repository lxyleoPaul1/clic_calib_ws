# Phase 3 — u_B aspect bias (b_const(u_B))

**Status:** flight 戊 @ seed **13025** — **simulation sign-off → Phase (B)**. PW + NE-only POI roll; SW 42 mm accepted as synthetic-sector limit.

## 10 Hz regression — root cause (confirmed)

| Check | Result |
|-------|--------|
| `N_pts` 0.5 vs 10 Hz | **45.2 vs 45.1** mean (frame-rate invariant ✓) |
| All-frame uniform σ, 10 Hz | NE/SW **worse** than 0.5 Hz → **temporal correlation** |
| Stride + reuse 0.5 Hz streams | **Rejected** — discards √N and Ruby-rate information |

## Fix (v2): full-frame correlated GLS

1. **v2 field:** `centroid_cov = σ_r² / N_pts`; refine `use_centroid_cov_whitening = true` on 10 Hz path.
2. **`TemporalDecorrelationConfig`:** AR(1) uniform scale `√((1−ρ)/(1+ρ))` on `body_temporal_sqrt_info_scales`; ρ from bias-norm lag-1 with **POI dwell floor ρ≥0.88** when `u_B` std ≤ 5°; applied in observed-mean + refine.
3. **POI roll lock:** `poi_sector0_attitude_scale = 0.35` on **NE sector only** (SW full roll).
4. **Removed:** `uniform_temporal_stride`, `match_streams_from`, aspect-quota subsample on 10 Hz calib.

## Stage-1 RTK @ native rate (Prais–Winsten v3)

PW: `r̃₁=√(1−ρ²)·r₁/σ`, `r̃ₜ=(rₜ−ρ·rₜ₋₁)/(σ√(1−ρ²))`; ρ from axis-mean lag-1 on attitude-pass residuals. PW auto **off** when RTK spacing ≥ 0.2 s.

| Rate | Antenna RMSE vs GT |
|------|-------------------|
| 0.5 Hz | **23.7 mm** (unchanged; independent RTK) |
| 10 Hz (native) | **27.3 mm** (was 36.0 mm @ v2 uniform scale) |

**Finding:** PW decouples dense RTK drift; 10 Hz still ~3.6 mm above 0.5 Hz floor — residual spline/attitude knot mismatch, not Σ inflation.

## NE/SW asymmetry

| Quantity | NE | SW |
|----------|----|----|
| mean range | 32.48 m | 32.48 m |
| varying_RMS | 25.4 mm | 21.7 mm |
| `lever_h` (‖R_LW R_WB b_const‖_h) | 236 mm | 227 mm |
| simple `u_B×b_const` projection | 0.13 mm | 0.31 mm |

**1.51× obs gap (legacy geometry) is *not* explained by lever_h alone** (ratio ≈1.04). Residual asymmetry: sector box-face visibility + coupled spline near t=45 s when NE roll is tightened.

## 戊 POI results @ seed 13025 (post-rework)

| tier | NE obs | SW obs | rel rot | center-reg |
|------|--------|--------|---------|------------|
| **0.5 Hz** | **12.2 mm** | **42.0 mm** | **0.12°** | 41 mm |
| **10 Hz full** | **58.8 mm** | **117.7 mm**† | 0.18° | 134 mm |

† SW 10 Hz: observed-mean **fallback** (same as 76cf1d6). **fbe701f regression** (NE 149 / SW 200) was **both-sector roll 0.35**, not PW Stage-2.

## Simulation sign-off vs field target

| Tier | Criterion | Threshold | @ 13025 |
|------|-----------|-----------|---------|
| **Simulation** | max(obs) | ≤ **~43 mm** | NE 12 / SW **42** ✓ |
| **Simulation** | rel rot | ≤ **0.15°** | **0.12°** ✓ |
| **Simulation** | PW + mechanisms | tidal lock, gate, PW RMSE | ✓ (10 Hz RMSE **27 mm**) |
| **Field** | each obs | ≤ **35 mm** @ 10 Hz | deferred to real dual-Ruby |

**SW 42 mm:** `poi_sector0` scales body **Rx** on **sector 0 only** (intentional). Per-frame audit: sector-1 **is** wired when `poi_roll_scale_all_sectors`; u_B→1.26° but obs unchanged → **synthetic POI sector coupling**, not observed-mean hook bug. **No further sim SW iteration.**

## Phase (B) — entered

Proceed: multi-LiDAR `CalibrationEstimator` + §7.2 relative extrinsic covariance / RTK common-mode (trajectory-perturbation MC). cm-level absolute accuracy validated on **real** diagonal dual-Ruby, not synthesis.

## Serial POI (flight 戊)

- **t ∈ [0, 45) s:** POI → **NE** LiDAR  
- **t ∈ [45, 90) s:** POI → **SW** LiDAR  
- One POI lock at a time; full RTK throughout.
