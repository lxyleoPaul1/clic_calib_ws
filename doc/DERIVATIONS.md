# Mathematical Specification (canonical)

**All factor implementations MUST match this document exactly.**  
Code review will reject any residual that paraphrases or shortcuts these formulae.  
Naming: `T_XY` maps `p_X = T_XY * p_Y` (see `include/clic_calib/sensor_data/frame_definitions.h`).

## Paper supplementary (LaTeX)

Clean LaTeX for §4.1–§4.8 is in [`doc/supplementary_section4.tex`](supplementary_section4.tex) — copy into the paper supplementary `\input{}`.

---

## LaTeX (§4.1–§4.8)

```latex
% See doc/supplementary_section4.tex for the full standalone file.
\subsection{Frame and lever-arm compositions}
\mathbf{p}_A^W(t) = \mathbf{p}_{WB}(t) + \mathbf{R}_{WB}(t)\,\mathbf{L}_{B\to A}

\subsection{RTK position residual}
\mathbf{r}_k^R = \tilde{\mathbf{p}}_A^k - [\mathbf{p}_{WB}(t_k^R) + \mathbf{R}_{WB}(t_k^R)\,\mathbf{L}_{B\to A}]

\subsection{LiDAR point-to-sphere residual}
r_{k,\ell}^L = \|\mathbf{q}_{k,\ell}^L - \mathbf{p}_G^L\| - R_{\mathrm{ball}}

\subsection{AprilTag reprojection}
\mathbf{r}_{k,j}^C = \tilde{\mathbf{u}}_{k,j} - \pi(\mathbf{K}\,(\mathbf{R}_{CW}\mathbf{p}_{M_j}^W + \mathbf{t}_{CW}))

\subsection{Total cost and observability}
J(\boldsymbol{\theta}) = \sum \|\mathbf{r}^R\|^2 + \sum \rho_C(\|r^L/\sigma_r\|^2) + \sum \rho_H(\|\mathbf{r}^C/\sigma_{\mathrm{pix}}\|^2) + R_{\mathrm{smooth}} + \sum \|\mathbf{r}^{\mathrm{prior}}\|^2
\mathbf{F}_{\mathrm{ext}} = \mathbf{F}_{ee} - \mathbf{F}_{er}\mathbf{F}_{rr}^{-1}\mathbf{F}_{re}
```

---

## Markdown reference (implementation)

## §4.1 Frame & lever-arm compositions

Given the B-spline state at time `t`:

- `p_WB(t) ∈ R³` — UAV body position in world `W`
- `R_WB(t) ∈ SO(3)` — UAV body orientation in world `W`

World-frame positions of derived points:

```
p_A^W(t)     = p_WB(t) + R_WB(t) * L_{B→A}           // RTK antenna phase center
p_G^W(t)     = p_WB(t) + R_WB(t) * L_{B→G}           // sphere center
p_{M_j}^W(t) = p_G^W(t) + R_WB(t) * L_{G→M_j}        // AprilTag j center
```

All lever arms `L_*` are expressed in body frame `B`, constants from `config/lever_arms.yaml`.

---

## §4.2 RTK position residual

**Observation:** `tilde_p_A^k` at clock `t_k^R` (RTK clock = world clock), reported covariance `Sigma_k^R ∈ R^{3×3}`.

```
r_k^R = tilde_p_A^k - [ p_WB(t_k^R) + R_WB(t_k^R) * L_{B→A} ]   ∈ R³
```

**Whitened:** `e_k^R = chol(Sigma_k^R)^{-1} * r_k^R`.

**Robust kernel:** NONE. RTK FIXED outliers filtered upstream (`fix_status == FIXED`, `multipath_metric < threshold`).

**Parameter blocks:** only B-spline control points adjacent to `t_k^R` (4 position knots + 4 orientation knots, cubic spline).

**Analytic Jacobians required:**

- `∂r/∂p_k` (each of 4 R³ control points): scalar weight `-B_j(u) * I_3`
- `∂r/∂ξ_k` (each of 4 SO(3) control points, right-trivialized): cumulative B-spline derivative on Lie group; follow Sommer et al., *Efficient Derivative Computation for Cumulative B-Splines on Lie Groups* (CVPR 2020). Uses `Adj` of partial products of `Exp(B̃_j(u) * Ω_j)`.

**Implementation:** `include/clic_calib/factor/rtk_position_factor.h`

---

## §4.3 LiDAR point-to-sphere-surface residual

For each LiDAR point `q_{k,l}^L` (raw sensor-frame) at scan timestamp `bar_t_k^L`:

```
t_k^L = bar_t_k^L - t_d^L                                 // recover RTK-clock time
p_G^W(t_k^L) = p_WB(t_k^L) + R_WB(t_k^L) * L_{B→G}        // sphere center in world
p_G^L = R_LW * p_G^W(t_k^L) + t_LW                        // sphere center in LiDAR frame
r_{k,l}^L = || q_{k,l}^L - p_G^L || - R_ball              // scalar residual
```

- `R_ball` — sphere radius, default `0.10` m (`config/target_geometry.yaml`)
- **Whitening:** divide by `sigma_r` (LiDAR ranging std, default `0.02` m)
- **Robust kernel:** Cauchy, scale `1.0`

**Parameter blocks:**

- `T_LW` (SE(3), 6 DoF) — calibration variable
- `t_d^L` (scalar)
- 4 position + 4 orientation B-spline control points adjacent to `t_k^L`

**Analytic Jacobians required** for all of the above.

**Time-offset derivative (continuous-time contribution):**

```
∂r/∂t_d^L = -(q - p_G^L)^T / ||q - p_G^L|| * R_LW * d_p_G^W/dt
```

where

```
d_p_G^W/dt = dot_p_WB(t) + R_WB(t) * [omega_WB(t)]_× * L_{B→G}
```

Both `dot_p_WB` and `omega_WB` are **analytic spline temporal derivatives**, NOT finite differences.

**Implementation:** `include/clic_calib/factor/sphere_implicit_factor.h`

---

## §4.4 AprilTag reprojection residual

For each AprilTag `j` at image timestamp `bar_t_k^C`, observed pixel `tilde_u_{k,j}` (`Eigen::Vector2d`):

```
t_k^C = bar_t_k^C - t_d^C
p_{M_j}^W = p_WB(t_k^C) + R_WB(t_k^C) * (L_{B→G} + L_{G→M_j})
p_{M_j}^C = R_CW * p_{M_j}^W + t_CW                       // marker in camera frame
u_pred = pi(K * p_{M_j}^C)                                // perspective projection
r_{k,j}^C = tilde_u_{k,j} - u_pred                        // 2D residual
```

- `K` — intrinsics from `config/sensor_rig.yaml`, **fixed** (not optimized in this refactor)
- `pi([X,Y,Z]) = [X/Z, Y/Z]`
- Lens distortion per config (`radtan` or `equidistant`, OpenCV convention)

**Whitening:** divide by `sigma_pix` (default `1.0` pixel).  
**Robust kernel:** Huber, `delta = 2.0` pixels.

**Parameter blocks:**

- `T_CW` (SE(3), 6 DoF)
- `t_d^C` (scalar)
- 4 position + 4 orientation B-spline control points adjacent to `t_k^C`

**Multiple AprilTags per frame:** each marker is an **independent** residual. Do **NOT** collapse to a single PnP-derived sphere center.

**Implementation:** `include/clic_calib/factor/apriltag_reproj_factor.h`

---

## §4.5 Trajectory smoothness regularizer

Integrated penalty on translation acceleration and angular velocity:

```
R_smooth = alpha_p * sum_k ||ddot_p_WB(t_k_sample)||^2 * Δt
         + alpha_R * sum_k ||omega_WB(t_k_sample)||^2 * Δt
```

Sample at knot midpoints. `alpha_p`, `alpha_R` from config (defaults `0.01`, `0.01`). Critical when RTK is sparse.

**Implementation:** `include/clic_calib/factor/trajectory_smoothness_factor.h`

---

## §4.6 Extrinsic prior factor

For each roadside sensor `X ∈ {C, L}`, prior `T_XW^prior` from mounting plan:

```
r^prior_X = Log( T_XW^{-1} * T_XW^prior )                 // ∈ R^6
```

Whitened by diagonal covariance: large rotation std (e.g. `5°`), translation std (e.g. `0.5` m). Corresponds to patent S5 `λ ||T - T_prior||²` on the manifold.

**Implementation:** `include/clic_calib/factor/prior_factor.h`

---

## §4.7 Total cost

```
J(θ) =   sum_k       || r_k^R ||^2_{Sigma_k^R}
       + sum_{k,l}   rho_C( || r_{k,l}^L / sigma_r ||^2 )            // LiDAR
       + sum_{k,j}   rho_H( || r_{k,j}^C / sigma_pix ||^2 )           // camera
       + R_smooth
       + sum_X       || r^prior_X ||^2_{Sigma^prior_X}
```

where

```
θ = { T_CW, T_LW, t_d^C, t_d^L, {p_k_WB}, {R_k_WB} }
rho_C = Cauchy(1.0)
rho_H = Huber(2.0)
```

---

## §4.8 Observability monitor

Marginal information on extrinsic block only (Schur-complement trajectory and time offsets):

```
F_ext = F_{ext,ext} - F_{ext,θ_rest} * F_{θ_rest,θ_rest}^{-1} * F_{θ_rest,ext}
```

Report:

- `λ_min(F_ext)`
- `condition_number(F_ext)`
- `PDOP_ext = sqrt( trace( F_ext^{-1}[t_x, t_y, t_z, t_z, ...] ) )`

If `λ_min(F_ext)` below threshold OR `PDOP_ext` above threshold → recommend additional flight along worst eigenvector direction.

**Implementation:** `include/clic_calib/estimator/observability_analyzer.h`

---

## Validation metrics: trajectory RMS vs 200 m Z-extrapolation error

These quantify different things and are **not contradictory** when both are small:

| Metric | What it measures | Typical test |
|--------|------------------|--------------|
| **Trajectory RMS** | Mean 3-D body-position error `\|p_WB_est(t) - p_WB_gt(t)\|` over the **observed flight window** (all axes, ~0–5 m local or 200 m standoff). | `test_full_pipeline_synthetic` |
| **200 m Z-error** | `\|T_LW.t.z - T_LW_gt.t.z\|` after full calibration at **long standoff** — dominated by **pitch / vertical lever-arm uncertainty × horizontal range** (first-order: `\delta z \approx R \cdot \delta\theta_pitch` at the roadside rig). | `test_patent_z_accuracy`, Python E2E |

A cm-level trajectory RMS over 4–5 s of multi-layer motion confirms the spline fits RTK + vision + LiDAR jointly. Sub-decimetre Z at 200 m confirms the **extrinsic vertical component** of `T_LW` is observable under multi-layer excitation (§2 smoothness / FIM). Coplanar flight removes pitch observability → Z-error grows (> 1 m), as in the coplanar ablation.

---

## Unified calibration setup (GTest = Python E2E)

All production paths share:

- **Prior:** `ExtrinsicPriorFactor` from `config/sensor_rig.yaml` (`prior_rot_std_deg=5`, `prior_trans_std_m=0.5`; mean = `initial_T_*`).
- **Time offsets:** released; hard clamp `\|t_d\| ≤ t_d_max_abs_s` (default **0.1 s** in `config/spline.yaml`), intersected with spline-support bounds.
- **Trajectory init:** `initialize_trajectory_from_rtk()` inside `solve()` (RTK knot seed + RTK-only pre-solve).

**Intentional differences:**

| Path | Standoff | Purpose |
|------|----------|---------|
| `test_full_pipeline_synthetic` | ~0–5 m local | Fast strict regression (rotation, translation, t_d, traj RMS). **Noise-free RTK** (LiDAR/camera also exact) — avoids cross-modal mismatch that widened tolerances masked. |
| Python E2E / `test_patent_z_accuracy` | 200 m | Long-range Z claim; **noisy RTK** (σ_xy=1 cm, σ_z=2 cm, cov matches factor weighting). |
| `test/diagnostic/test_td_single_variable` | isolated | Jacobian / observability only (no RTK, fixed knots). |

Simulation GT extrinsics **must** match `sensor_rig.yaml` `initial_T_*` so the prior mean equals ground truth.

---

## References

- Sommer et al., CVPR 2020 — cumulative B-spline derivatives on Lie groups (§4.2 rotation Jacobians).
- Basalt / clic split SO(3)+R³ uniform cubic B-spline (implementation in `include/clic_calib/spline/`).
