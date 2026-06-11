# Phase 3 — u_B aspect bias (b_const(u_B))

**Status:** flight 丁 diagnosis + flight 戊 POI @ seed **13025**; 10 Hz regression **fixed**.

## Hypothesis

Range-only models **(a) p_B(r)** and **(c) face-window** are excluded on flight 丁.
**Range down-weight (b)** also fails (weak |r|, 5× `range_coeff` ineffective).

The remaining variable is **look-direction azimuth in body** `u_B`: unit vector from
body toward the active LiDAR post, expressed in body frame B. View-dependent centroid
bias `bias_B` should correlate with `u_B` azimuth (near-sinusoidal), not range alone.

## Tidal lock (NearFieldHighAspect / Phase 1.5)

Single-sensor `PoseWbFromGeometry` with `high_attitude_variation`:

- Orbit azimuth: `atan2(y, x)` from standoff arc around sensor at world origin.
- Body yaw: `atan2(−y, −x)` — nose toward the sensor (velocity/POI style).

These are **linearly coupled** (`r ≈ 1`): orbit sweep and body yaw are **tidally
locked**. `u_B` azimuth is **not** constant on Phase 1.5, but bias varies smoothly
with aspect → `varying_RMS ≈ 22 mm`, observed-mean `p_B` works (18.5 mm).

## Flight 丁 (decoupled attitude)

Translation: dual-sector wide orbit (35–50 m). Attitude: Phase-1.5
`NearFieldHighAspect` on sector-local time — **not** tied to dual-orbit `−p_WB`.

- Tidal-lock audit on 丁 sector-0: **low** orbit_az ↔ body_yaw correlation.
- `u_B` azimuth **sweeps** with decoupled attitude → large `bias_B` vs `u_B` structure.
- `varying_RMS` on 丁 ≫ Phase 1.5 (~22 mm reference).

## Flight 戊 (POI per sector)

`kPerSectorTidalLockPOI`: in each LiDAR sector, arc orbit + **nose locked toward
that sector's post** (DJI POI). Pitch/roll sweep retained.

- `u_B` azimuth std ≈ **3.6°** per sector (tidal lock to post).
- Gate via POI branch (`u_B` stable + pitch_std≥14°).

## 10 Hz regression — diagnostic (seed 13025)

| Check | Result |
|-------|--------|
| Per-scan `N_pts` 0.5 Hz vs 10 Hz | **45.2 vs 45.1** mean (frame-rate invariant ✓) |
| 10 Hz, cov OFF (legacy σ), all 300 fr | NE **78** / SW **140** mm — does **not** return to 0.5 Hz |
| Conclusion | **centroid_cov overweight** (partial); **temporal correlation** (primary) |

Correlated dwell at 10 Hz overweighted repeated `u_B`/bias directions when means
were treated as independent.

## Fix (calibration prepare policy)

1. **v2 field:** `centroid_cov = σ_r² / N_pts` (per scan; `N` independent of frame rate).
2. **Refine whitening:** `use_centroid_cov_whitening = false` → legacy `σ(r,N)` (restores 乙 ≈ **165 mm**).
3. **High-rate calib:** `BodyObsPreparePolicy` on 10 Hz sim:
   - Rebuild body cluster at **0.5 s** (`stride = lidar_dt_10 / lidar_dt_05`) with same RNG seed → identical frames to native 0.5 Hz.
   - Copy RTK / attitude / tag streams from **0.5 Hz reference** scenario (dense 10 Hz RTK otherwise shifts Stage-1 spline).
   - Optional aspect-quota cap (60 fr/sector); POI lock uses orbit-azimuth bins when `u_B` is tight.

After fix, **戊@10 Hz = 戊@0.5 Hz** (bit-identical @ seed 13025).

## Serial POI mission (flight 戊)

Real flight = **two time-multiplexed POI orbits** on shared RTK trajectory:

- **t ∈ [0, 45) s:** nose locked to **NE** roadside LiDAR (POI).
- **t ∈ [45, 90) s:** nose locked to **SW** roadside LiDAR (POI).

One POI lock at a time (DJI POI mode); diagonal dual-LiDAR geometry requires serial
sectors, not simultaneous POI to both posts.

## Measured @ seed 13025

### 丁 aspect scatter (GT)

| sensor | u_B az std | varying_RMS | r(u_B,bias_x) | r(u_B,bias_y) |
|--------|------------|-------------|---------------|---------------|
| NE | 14.1° | 110 mm | +0.53 | −0.68 |
| SW | 17.6° | 124 mm | −0.64 | +0.69 |
| P1.5 ref | — | **22 mm** | — | — |

Tidal lock: P1.5 `yaw−orbit` std **0°**; 丁 sector **110°** (decoupled).

### 戊 POI (post-fix)

| tier | NE obs | SW obs | rel rot | center-reg |
|------|--------|--------|---------|------------|
| 0.5 Hz | 55.1 mm | **36.5 mm** | **0.09°** | 42 mm |
| 10 Hz (fixed) | **55.1 mm** | **36.5 mm** | **0.09°** | 42 mm |

**NE/SW asymmetry (1.51×):** post heights both **5.0 m**; mean range **32.48 m** both
(Δ < 1 mm). Residual gap from **sector POI geometry** — differing box-face visibility
and `b_const` lever per diagonal sector (not legacy 4.5/5.5 m post skew).

## (B) entry criterion (updated)

Deprecated: `rel |trans| ≤ 35 mm` (dominated by `Δθ × baseline`, e.g. 甲 0.06°×70.7 m).

**New @ flight 戊 10 Hz:**

| Metric | Threshold | Status |
|--------|-----------|--------|
| Each sensor obs `\|trans\|` | ≤ 35 mm | NE **55** ✗ SW **37** ✗ |
| Relative rotation | ≤ 0.1° | **0.09°** ✓ |
| Center registration (report) | — | 42 mm |

10 Hz regression **resolved**; (B) blocked on absolute obs (especially NE), not frame rate.

See `multi_lidar_board_free_blueprint.md`.
