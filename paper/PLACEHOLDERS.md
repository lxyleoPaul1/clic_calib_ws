# Placeholder index — field data fill guide

Every `\TODO{...}` in `paper/*.tex` is listed here.
**Rule:** never substitute fabricated field numbers; only `\TODO` until data exist.

## Summary counts (regenerate after edit)

Run: `rg -c '\\TODO' paper/*.tex` after changes.

---

## `sec_intro.tex`

| Location | Wait for | Campaign | Fills |
|----------|----------|----------|-------|
| Contributions item 5 | All field campaigns summary | A–D | Intro contribution bullet |

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
| `tab:field_boardfree` NE Δtrans | Board-free vs sphere NE translation error | **A** | Table 1 |
| `tab:field_boardfree` NE Δrot | NE rotation error | **A** | Table 1 |
| `tab:field_boardfree` NE center-reg | NE center-reg | **A** | Table 1 |
| `tab:field_boardfree` SW Δtrans | SW translation error | **A** | Table 1 |
| `tab:field_boardfree` SW Δrot | SW rotation error | **A** | Table 1 |
| `tab:field_boardfree` SW center-reg | SW center-reg | **A** | Table 1 |
| `tab:field_relext` rel/abs ratio | Measured field rel/abs vs sim 0.166× | **D** | Table 2 |
| `tab:field_relext` correlation | Epoch correlation NE/SW | **D** | Table 2 |
| `tab:field_relext` center-reg | Coherent perturbation residual | **D** | Table 2 |
| RQ1 | Per-epoch bias vs $u_B$ scatter data | **B** | Fig. 4 / mechanism transfer |
| RQ2 | Table 1 all cells | **A** | Board-free accuracy claim |
| RQ3 | Table 2 all cells | **D** | §7.2 field test |
| RQ4 | POI execution + yaw_cv stats | **B** | Feasibility paragraph |
| Fig. `field_layout` | Layout diagram photo/CAD | **A** | Fig. 3 |
| Fig. `field_bias` | Real scatter plot | **B** | Fig. 4 |

## Campaign definitions

- **Campaign A — accuracy sign-off:** one intersection deployment, sphere survey, single board-free solve, Δ metrics.
- **Campaign B — mechanism transfer:** repeat flights with logged attitude; export $u_B$–bias for comparison to Flight 丁/P1.5.
- **Campaign C — truth chain:** total-station sphere coordinates + uncertainty budget (feeds A, not a separate table).
- **Campaign D — §7.2 field test:** intentional or natural RTK coherent segment; estimate rel/abs covariance ratio vs simulation.

## Sphere-target red line

Spheres appear only in campaigns A/C as **evaluation ruler**. Do not document spheres as calibration fallback in any filled cell.
