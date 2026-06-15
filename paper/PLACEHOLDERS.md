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

### Metadata placeholders

| Location | Wait for | Campaign | Fills |
|----------|----------|----------|-------|
| Experimental setup | Site name, coordinates, post survey, Ruby firmware | **A** | setup paragraph |
| Experimental setup | Flight logs, POI timing, weather, yaw\_cv / pitch stats | **B** | setup paragraph |
| Sphere truth protocol | Total-station sphere uncertainty budget and layout | **C** | truth-chain paragraph |
| Campaign D protocol | Base-station offset vector, replay settings, network-RTK cross-check | **D** | coherent-bias replay paragraph |
| Fig. `field_layout` | Layout diagram photo/CAD | **A** | Fig. 3 |
| Fig. `field_bias` | Real scatter plot | **B** | Fig. 4 |

### Numeric placeholders

| Location | Wait for | Campaign | Fills |
|----------|----------|----------|-------|
| `tab:field_boardfree` NE/SW cells | $\Delta$trans, $\Delta$rot, center-reg | **A** | board-free vs sphere truth |
| `tab:field_relext` base offset | known base-station coordinate shift | **D** | replay perturbation magnitude |
| `tab:field_relext` coherent/white rows | rel/abs ratios, segment correlation, c-reg residual | **D** | relative-extrinsic field test |
| RQ1 | Per-epoch bias vs $u_B$ scatter data | **B** | mechanism transfer |
| RQ2 | Table 1 all cells | **A** | board-free accuracy claim |
| RQ3 | Table 2 all cells | **D** | relative extrinsic field test |
| RQ4 | POI execution + yaw\_cv stats | **B** | feasibility paragraph |

## Campaign definitions

- **Campaign A — accuracy sign-off:** one intersection deployment, total-station sphere survey, single board-free solve, Δ metrics.
- **Campaign B — mechanism transfer:** POI serial sectors with logged attitude; export $u_B$–bias for comparison to \flightD{} / near-field profile.
- **Campaign C — truth chain:** total-station sphere-center coordinates + uncertainty budget (feeds A, not a separate table).
- **Campaign D — relative extrinsic field test:** replay the same RTK data with a known base-station coordinate offset; estimate coherent rel/abs covariance ratio vs simulation and compare with network-RTK cross-check.

## Sphere-target red line

Spheres appear only in campaigns A/C as **evaluation ruler**. Do not document spheres as calibration fallback in any filled cell.
