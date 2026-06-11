# Board-free mechanism — oral-depth argument chain

**Status:** Production modules @ Phase 1 (`body_gated_calibration`,
`multi_lidar_body_calibration`, `observed_mean_gate`, `relative_extrinsic`).
Frozen numbers: `doc/board_free_results_frozen.md` @ `9eff261`.

**Punchline:** aspect-dependent body-centroid bias is a **vector field on u_B**.
Under concentrated aspect it **degenerates with t_LW** and silently pollutes
extrinsics; **active 3D attitude excitation (POI tidal lock)** breaks the
degeneracy so the bias decorrelates and averages in world frame — **active
excitation is necessary**, not optional, for unbiased board-free calibration.

---

## 1. bias_B is a u_B vector field (not constant)

**Flight 丁** (wide u_B sweep, decoupled orbit) @ seed 13025:

| Sensor | u_B az std [°] | varying RMS [mm] | r(u_B, bias_x) | r(u_B, bias_y) |
|--------|----------------|------------------|----------------|----------------|
| NE | 14.1 | **110** | +0.53 | **−0.68** |
| SW | 17.6 | **124** | −0.64 | **+0.69** |
| P1.5 ref | — | **22** | — | — |

**Formula (per frame):** `bias_B,i = centroid_empirical_B,i − L_B_nominal` with
empirical centroid from **visible sampled points** (not box center + noise).
Observed-mean lever `p_B` estimates the **mean of this field** over flown aspect.

**Test:** `test_phase3_dual_lidar_experiment_a` (Flight 丁 block).

---

## 2. 2×2 degeneracy table (u_B × R_WB)

|  | **R_WB sweeps** | **R_WB near-locked** |
|--|-----------------|----------------------|
| **u_B sweeps** (丁) | NE **60.3** / SW **89.9** mm | — |
| **u_B POI-locked** (戊) | NE **12.2** / SW **42.0** mm†; rel rot **0.12°** | 甲 gate OFF: **31.5** mm |

**Concentrated aspect (211 mm floor):** `NearFieldHighAspect` concentrated column
centroid-only **211.4** mm; diverse aspect **63.5** mm; observed-mean **18.5** mm;
joint-opt **20.4** mm (`test_phase15_main_table`).

**Joint p_B no-op:** on concentrated aspect, joint optimization does not remove
the 211 mm floor — bias and t_LW remain coupled.

---

## 3. Frame-wise mechanism: why excitation is necessary

`v_i = R_LW R_WB(t_i) b_const` in world frame:

| Regime | ‖mean(v)‖_horizontal [mm] | pitch_std [°] | yaw_cv |
|--------|---------------------------|---------------|--------|
| Concentrated | **~220** | ~0 | ~0 |
| Diverse (P1.5) | **~51** | ~16 | ~0.78 |

Horizontal components **average under yaw diversity**; vertical **~19.5 mm**
remains (body-Z component of `b_const` — yaw cannot average it; `p_B` removes it).

**Tidal lock:** P1.5 yaw−orbit std **0.0°** (locked); 丁 sector-0 **110.4°** (not locked).

---

## 4. Production path (Phase 1)

| Module | Role |
|--------|------|
| `observed_mean_gate` | yaw_cv ≥ 0.65 (+ POI path); fallback centroid-only |
| `body_gated_calibration` | gated observed-mean ×3 + cost/trans guard |
| `multi_lidar_body_calibration` | per-sensor independent Stage-2; compose `T_rel` |
| `relative_extrinsic` | §7.2 cov ratio, center-reg metric |
| `Stage1TrajectoryFitter` | Prais–Winsten RTK whitening (`use_rtk_prais_winsten=true`) |

**Multi-LiDAR:** shared Stage-1 trajectory; **no shared Ceres blocks** in Stage-2;
`T_{SW\leftarrow NE} = T_{SW,W} · T_{NE,W}^{-1}` post hoc.

**Guards:** observed-mean applied only if Ceres cost ≤ centroid cost and
`|trans|_obs ≤ |trans|_cent` (simulation GT); never output worse than centroid
(乙 **165 mm** rollback).

**§7.2 (sim, sealed):** coherent RTK bias **0.17×** rel/abs; white noise **3.10×**.
Scope: injected coherent systematic bias — not a field guarantee.

---

## 5. Deployment notes

- **Ceres `num_threads=1`:** set on all production solves @ Phase 0 gflags guard;
  correct for test isolation; **revisit for large field bags** (performance).
- **PW Stage-1:** 10 Hz RMSE **27.3 mm** (target 23.7 mm @ 0.5 Hz independent RTK);
  ρ single scalar; uniform Σ inflation **~125 mm — rejected**.

---

## 6. Gate references

| Gate | Test / script |
|------|----------------|
| Frozen baseline + isolation | `scripts/verify_frozen_baseline_isolation.sh` |
| 戊 0.5 Hz NE/SW | `test_phase3_dual_lidar_experiment_a` |
| §7.2 0.17× / 3.10× | `test_phase3_dual_lidar_phase_b` |
| λ_min 1.074 / threshold 0.003 | `test_observability_synthetic` |
