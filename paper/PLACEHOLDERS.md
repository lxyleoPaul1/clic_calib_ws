# Placeholder index — field data fill guide

Every `\TODO{...}` in `paper/*.tex` is listed here.
**Rule:** never substitute fabricated field numbers; only `\TODO` until data exist.

## Summary counts (regenerate after edit)

Run: `rg -c '\\TODO' paper/*.tex` after changes.

---

## `sec_related.tex` (citation placeholders)

| Location | Wait for | Fills |
|----------|----------|-------|
| Roadside V2X | Survey or target paper bibkey | `\TODO{cite: roadside V2X multi-LiDAR calibration}` |
| Target survey | Survey bibkey | `\TODO{cite: target-based LiDAR extrinsic calibration survey}` |
| UAV RTK | UAV calib paper bibkey | `\TODO{cite: UAV RTK-assisted sensor calibration}` |
| TrajMatch | Trajectory-matching paper | `\TODO{cite: trajectory-matching extrinsic calibration}` |
| Coplanar FIM | Anonymous Paper I bibkey | `\TODO{cite: prior coplanar multi-LiDAR FIM degeneracy}` |

## `sec_framework.tex`

| Location | Wait for | Campaign | Fills |
|----------|----------|----------|-------|
| Fig. 1 caption | Publication-quality framework diagram | N/A (design) | `fig_framework.pdf` |

## `sec_mechanism.tex`

| Location | Wait for | Campaign | Fills |
|----------|----------|----------|-------|
| Fig. 2 fbox | Mechanism composite figure | simulation export + later field overlay | `fig_mechanism.pdf` |

## `sec_field.tex`

| Location | Wait for | Campaign | Fills |
|----------|----------|----------|-------|
| Site bullet | Site name, GPS, post survey | **A** | §Field setup |
| Platform bullet | Firmware, lever-arm | **A** | §Field setup |
| Flight bullet | Logs, POI timing, weather | **B** | §Field setup |
| Ground truth bullet | Sphere survey σ | **C** | §Sphere-anchor protocol |
| `tab:field_boardfree` NE/SW cells | Short `\TODO{campaign A: ...}` in table (layout-safe) | **A** | Table 1 |
| `tab:field_relext` rows | Short `\TODO{campaign D: ...}` in table* | **D** | Table 2 |
| RQ1 | Per-epoch bias vs $u_B$ scatter data | **B** | Fig. 4 / mechanism transfer |
| RQ2 | Table 1 all cells | **A** | Board-free accuracy claim |
| RQ3 | Table 2 all cells | **D** | Relative extrinsic field test |
| RQ4 | POI execution + yaw_cv stats | **B** | Feasibility paragraph |
| Fig. `field_layout` | Layout diagram photo/CAD | **A** | Fig. 3 |
| Fig. `field_bias` | Real scatter plot | **B** | Fig. 4 |

## Campaign definitions

- **Campaign A — accuracy sign-off:** one intersection deployment, sphere survey, single board-free solve, Δ metrics.
- **Campaign B — mechanism transfer:** repeat flights with logged attitude; export $u_B$–bias for comparison to \flightD{} / near-field profile.
- **Campaign C — truth chain:** total-station sphere coordinates + uncertainty budget (feeds A, not a separate table).
- **Campaign D — relative extrinsic field test:** intentional or natural RTK coherent segment; estimate rel/abs covariance ratio vs simulation.

## Sphere-target red line

Spheres appear only in campaigns A/C as **evaluation ruler**. Do not document spheres as calibration fallback in any filled cell.
