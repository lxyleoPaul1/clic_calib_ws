# Figure and table plan — ICRA oral paper

Baseline docs: `doc/board_free_mechanism.md`, `doc/board_free_results_frozen.md`.
Commits: `f03c399`, `3fb7ddf`, `7f5c27c`.

## Figures

| ID | File (planned) | Content | Data source | Contribution |
|----|----------------|---------|-------------|--------------|
| Fig. 1 | `fig_framework.pdf` | Two-stage separable pipeline: RTK → Stage-1 → freeze → per-LiDAR Stage-2 → $T_{\mathrm{rel}}$ | Architecture (code modules) | Separable framework |
| **Fig. 2** | `fig_mechanism.pdf` | **Oral core panel:** (a) $u_B$ sphere + bias vector field + Flight 丁 scatter; (b) $2\times2$ degeneracy table; (c) $\|\mathrm{mean}(R_{LW}R_{WB}\mathbf{b})\|$ 220→51 mm + 19.5 mm vertical bar | **Simulation** seed 13025 | Necessary condition punchline |
| Fig. 3 | `fig_field_layout.pdf` | Intersection diagonal Ruby posts, UAV path, POI sectors | **Field campaign A** | Field setup |
| Fig. 4 | `fig_field_bias.pdf` | Real $u_B$–bias scatter vs simulation overlay | **Field campaign B** | Mechanism transfer |

Current LaTeX: Fig. 1 = TikZ stub; Fig. 2/3/4 = `\TODO` fboxes.

## Tables (in paper)

| Label | Section | Content | Source |
|-------|---------|---------|--------|
| `tab:ding_correlation` | mechanism | Flight 丁 $r(u_B,\mathrm{bias})$ | simulation |
| `tab:degeneracy_2x2` | mechanism | $u_B \times R_{WB}$ | simulation |
| `tab:concentrated` | mechanism | 211 mm / 20 mm ablation | simulation |
| `tab:frame_norm` | mechanism | 220→51 mm | simulation |
| `tab:relext_mc` | relext | 0.166× / 3.101× | simulation |
| `tab:overlap` | relext | +10 s overlap 0.15× | simulation |
| `tab:pw` | pw | 36→27.3 mm PW | simulation |
| `tab:p15` | sim | P1.5 63.5/18.5 mm | simulation |
| `tab:wu_dual` | sim | 戊 NE 12.15 / SW 42.04 | simulation |
| `tab:lambda` | sim | λ_min 1.074 vs ≈0 | simulation |
| `tab:field_boardfree` | field | **PLACEHOLDER** Δ vs sphere | field campaign A |
| `tab:field_relext` | field | **PLACEHOLDER** coherent residual | field campaign D |

## Campaign → artifact map

| Campaign | Collect | Fills |
|----------|---------|-------|
| **A** | Dual-Ruby intersection flight + sphere survey + board-free run | Table `tab:field_boardfree`, Fig. 3 |
| **B** | POI flight logs, per-epoch $u_B$–bias audit | Fig. 4, RQ1, RQ4 |
| **C** | Total-station sphere positions | Ground-truth ruler protocol |
| **D** | RTK perturbation / rel-abs MC on field bags | Table `tab:field_relext`, RQ3 |
