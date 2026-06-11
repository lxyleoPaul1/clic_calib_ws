# Phase 3 — u_B aspect bias (b_const(u_B))

**Status:** flight 丁 diagnosis + flight 戊 POI @ seed **13025**.

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

- `u_B` azimuth std ≈ 0 per sector (tidal lock to post).
- Expected: per-sensor obs ~20–30 mm; mechanism 2×2 closes like Phase 1.5.

## Whitening

v2 `centroid_cov` on simulated observations: isotropic `σ² I` with
`σ = σ_range / √N`. `ExtrinsicRefiner` uses `BodyCentroidSqrtInformationFromObservation`.
Arbitrary `range_coeff × 5` down-weight **removed**.

## Measured @ seed 13025

### 丁 aspect scatter (GT)

| sensor | u_B az std | varying_RMS | r(u_B,bias_x) | r(u_B,bias_y) |
|--------|------------|-------------|---------------|---------------|
| NE | 14.1° | 110 mm | +0.53 | −0.68 |
| SW | 17.6° | 124 mm | −0.64 | +0.69 |
| P1.5 ref | — | **22 mm** | — | — |

Tidal lock: P1.5 `yaw−orbit` std **0°**; 丁 sector **110°** (decoupled).

### 戊 POI

| tier | NE obs | SW obs | rel rot | center-reg |
|------|--------|--------|---------|------------|
| 0.5 Hz (~60 fr) | 55 mm | **36.5 mm** | **0.09°** | 42 mm |
| 10 Hz (~300 fr) | 90 mm | 123 mm | 0.40° | 167 mm |

u_B az std ≈ **3.6°** (≪ 丁). Gate via POI branch (`u_B` stable + pitch_std≥14°).

## (B) entry criterion (updated)

Deprecated: `rel |trans| ≤ 35 mm` (dominated by `Δθ × baseline`, e.g. 甲 0.06°×70.7 m).

**New @ flight 戊 10 Hz:**

| Metric | Threshold |
|--------|-----------|
| Each sensor obs `\|trans\|` | ≤ 35 mm |
| Relative rotation | ≤ 0.1° |
| Center registration (report) | separate |

60-frame tier: relative rotation ≤ 0.15° (reference).

**Status:** 10 Hz **NOT met**; 0.5 Hz SW trans borderline, rel rot passes 0.15° tier.

See `multi_lidar_board_free_blueprint.md`.
