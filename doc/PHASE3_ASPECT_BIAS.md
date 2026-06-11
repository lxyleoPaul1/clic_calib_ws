# Phase 3 — u_B aspect bias (b_const(u_B))

**Status:** flight 戊 full-frame 10 Hz GLS rework @ seed **13025** (no stride/subsample).

## 10 Hz regression — root cause (confirmed)

| Check | Result |
|-------|--------|
| `N_pts` 0.5 vs 10 Hz | **45.2 vs 45.1** mean (frame-rate invariant ✓) |
| All-frame uniform σ, 10 Hz | NE/SW **worse** than 0.5 Hz → **temporal correlation** |
| Stride + reuse 0.5 Hz streams | **Rejected** — discards √N and Ruby-rate information |

## Fix (v2): full-frame correlated GLS

1. **v2 field:** `centroid_cov = σ_r² / N_pts`; refine `use_centroid_cov_whitening = true` on 10 Hz path.
2. **`TemporalDecorrelationConfig`:** AR(1) uniform scale `√((1−ρ)/(1+ρ))` on `body_temporal_sqrt_info_scales`; ρ from bias-norm lag-1 with **POI dwell floor ρ≥0.88** when `u_B` std ≤ 5°; applied in observed-mean + refine.
3. **NE POI tighten:** `poi_sector0_attitude_scale = 0.35` (roll only on sector 0) → `u_B` std NE **1.3°** vs SW **3.6°**.
4. **Removed:** `uniform_temporal_stride`, `match_streams_from`, aspect-quota subsample on 10 Hz calib.

## Stage-1 RTK @ native 10 Hz

| Rate | Antenna RMSE vs GT |
|------|-------------------|
| 0.5 Hz | **23.7 mm** |
| 10 Hz (native) | **36.0 mm** (slightly worse; not fixed by per-sample Σ inflation) |

**Finding:** dense RTK is **not** subsampled; residual gap likely spline DOF vs 50 Hz attitude knot driver (`knot_dt≈0.05 s`) with extra RTK constraints — needs rate-aware RTK information (future), not decimation.

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
| **10 Hz full** | **53.9 mm** | **117.6 mm**† | 0.31° | 152 mm |

† SW 10 Hz: observed-mean **fallback** (iterative path diverges under dense correlated dwell).

## (B) entry @ 10 Hz full-frame

| Metric | Threshold | Status |
|--------|-----------|--------|
| Each obs `\|trans\|` | ≤ 35 mm | NE **54** ✗ SW **118** ✗ |
| rel rot | ≤ 0.1° | **0.31°** ✗ |

**Not entering (B).** 0.5 Hz NE already **12 mm**; 10 Hz still needs joint Cholesky whitening or Stage-1 RTK rate normalization.

## Serial POI (flight 戊)

- **t ∈ [0, 45) s:** POI → **NE** LiDAR  
- **t ∈ [45, 90) s:** POI → **SW** LiDAR  
- One POI lock at a time; full RTK throughout.
