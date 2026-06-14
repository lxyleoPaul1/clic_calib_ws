# Placeholder index — field data fill guide

Every `\TODO{...}` in `paper/*.tex` is listed here.
**Rule:** never substitute fabricated field numbers; only `\TODO` until data exist.

## Summary counts (regenerate after edit)

Run: `rg -c '\\TODO' paper/*.tex` after changes.

---

## Citation verification placeholders

| Location | Wait for | Fills |
|----------|----------|-------|
| Anonymous coplanar FIM reference | Final double-blind citation key and bibliography entry | prior work positioning in Related Work |

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
