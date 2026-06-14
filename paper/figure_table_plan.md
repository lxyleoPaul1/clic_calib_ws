# Figure and table plan — ICRA oral paper

Baseline docs: `doc/board_free_mechanism.md`, `doc/board_free_results_frozen.md`
(not cited in the double-blind paper).

## Figures

| ID | File (planned) | Content | Data source | Contribution |
|----|----------------|---------|-------------|--------------|
| Fig. 1 | in-paper TikZ | Two-stage separable pipeline: RTK → Stage-1 → freeze → per-LiDAR Stage-2 → $T_{\mathrm{rel}}$ | Architecture | Separable framework |
| **Fig. 2** | `fig_mechanism.pdf` | **Oral core panel:** (a) $u_B$ sphere + bias vector field + \flightD{} scatter; (b) horizontal mean bias driver 220→51 mm + 19.5 mm vertical residual; (c) yaw\_cv sweep and gate operating point | **Simulation** | Necessary-condition punchline |
| Fig. 3 | `fig_field_layout.pdf` | Intersection diagonal Ruby posts, UAV path, POI sectors | **Field campaign A** | Field setup |
| Fig. 4 | `fig_field_bias.pdf` | Real $u_B$–bias scatter vs simulation overlay | **Field campaign B** | Mechanism transfer |

Current LaTeX: Fig. 1 = TikZ; Fig. 2 = generated PDF; Fig. 3/4 = field placeholders.

## Tables (in paper)

| Label | Section | Content | Source |
|-------|---------|---------|--------|
| `tab:ding_correlation` | mechanism | Flight 丁 $r(u_B,\mathrm{bias})$ | simulation |
| `tab:degeneracy_2x2` | mechanism | $u_B \times R_{WB}$ | simulation |
| `tab:concentrated` | mechanism | 211 mm / 20 mm ablation | simulation |
| `tab:frame_norm` | mechanism | 220→51 mm | simulation |
| `tab:relext_mc` | relext | coherent 0.247±0.12× / white 3.05±0.16× | simulation |
| `tab:p15` | sim | P1.5 mean±std: 55.7±10.7 / 30.0±13.0 / 26.9±10.1 mm | simulation |
| `tab:wu_dual` | sim | \flightW{} NE/SW statistically comparable: 29.7±9.7 / 29.0±10.8 mm | simulation |
| `tab:lambda` | sim | λ_min 1.07357 vs ≈0 | simulation |
| `tab:field_boardfree` | field | **PLACEHOLDER** Δ vs sphere | field campaign A |
| `tab:field_relext` | field | **PLACEHOLDER** coherent residual | field campaign D |

## Campaign → artifact map

| Campaign | Collect | Fills |
|----------|---------|-------|
| **A** | Dual-Ruby intersection flight + sphere survey + board-free run | Table `tab:field_boardfree`, Fig. 3 |
| **B** | POI flight logs, per-epoch $u_B$–bias audit | Fig. 4, RQ1, RQ4 |
| **C** | Total-station sphere positions | Ground-truth ruler protocol |
| **D** | RTK perturbation / rel-abs MC on field bags | Table `tab:field_relext`, RQ3 |
