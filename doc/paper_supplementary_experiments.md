# Paper supplementary experiments (P1)

**Status:** simulation-only; frozen regression anchors unchanged in `doc/board_free_results_frozen.md`.

Reproduce: `scripts/compile_local_tests.sh` then run `build/local_tests/test_paper_*`.

| Test | Profile | Seeds | Key outputs | vs frozen anchor (13025) |
|------|---------|-------|-------------|--------------------------|
| `test_paper_yawcv_sweep` | P1.5 geom, azimuth span sweep | 8/level × 8 levels | `paper/figures/data/yawcv_sweep.csv` | anchor 63.5 / 18.4 mm @ 165° |
| `test_paper_quadrant_fill` | u_B sweep × locked $R_{WB}$ | 13025 | `quadrant_fill.csv`, `ding_bias_scatter.csv` | gate OFF; NE 106.3 / SW 84.9 mm |
| `test_paper_multiseed` | P1.5 + 戊 0.5 Hz | 10 (13025–13034) | `multiseed_summary.csv` | means within 2σ of frozen |
| `test_paper_mirror_control` | 戊 POI sector mirror | 13025 | `mirror_control.csv` | gap 29.9→15.1 mm |

## P1.1 yaw_cv sweep (identifiability transition)

- Low platform (yaw_cv ≲ 0.05): centroid ~240–450 mm, obs tracks centroid.
- High platform (yaw_cv ≳ 0.75): centroid ~48–58 mm, obs ~26–27 mm (mean).
- Transition ~0.05–0.65: smooth S-curve; gate 0.65 is conservative shoulder, not a kink.

## P1.3 multiseed means (Table VIII / tab:p15)

| Metric | mean ± std [mm] | frozen @ 13025 |
|--------|-----------------|----------------|
| P1.5 centroid | 55.7 ± 10.7 | 63.5 |
| P1.5 observed-mean | 30.0 ± 13.0 | 18.5 |
| P1.5 joint-opt | 26.9 ± 10.1 | 20.4 |
| 戊 NE obs | 29.7 ± 9.7 | 12.15 |
| 戊 SW obs | 29.0 ± 10.8 | 42.04 |

## P1.4 mirror control

- Baseline 戊 @ 13025: NE 12.15 mm, SW 42.04 mm (anchor only).
- POI-sector mirror did not produce a clean NE↔SW swap; consistent with no stable sector asymmetry across seeds.

## Figures

`paper/figures/make_fig2.py` → `fig_mechanism.pdf` (panels a–c).
