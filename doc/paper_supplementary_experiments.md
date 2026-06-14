# Paper supplementary experiments (P1)

**Status:** simulation-only; frozen regression anchors unchanged in `doc/board_free_results_frozen.md`.

Reproduce: `scripts/compile_local_tests.sh` then run `build/local_tests/test_paper_*`.

| Test | Profile | Seeds | Key outputs | vs frozen anchor (13025) |
|------|---------|-------|-------------|--------------------------|
| `test_paper_yawcv_sweep` | P1.5 geom, azimuth span sweep | 8/level × 8 levels | `paper/figures/data/yawcv_sweep.csv` | anchor 63.5 / 18.4 mm @ 165° |
| `test_paper_quadrant_fill` | u_B sweep × locked $R_{WB}$ | 13025 | `quadrant_fill.csv`, `ding_bias_scatter.csv` | gate OFF; NE 106.3 / SW 84.9 mm |
| `test_paper_multiseed` | P1.5 + 戊 0.5 Hz | 10 (13025–13034) | `multiseed_summary.csv` | means within 2σ of frozen |
| `test_paper_mirror_control` | 戊 POI sector mirror | 13025 | `mirror_control.csv` | gap 29.9→15.1 mm |
| `test_paper_relext_multiseed` | §7.2 RTK perturbation MC | 10 (13025–13034), 20 MC/seed | `relext_multiseed.csv` | anchor reproduces 0.166 / 3.101 |
| `test_paper_prop1_nulldir` | Prop.1 null-direction FIM | locked→diverse attitude sweep | `prop1_nulldir.csv` | locked direction at machine zero |

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

## P1.7 §7.2 relative-extrinsic multiseed

Anchor seed 13025 reproduces the frozen §7.2 ratios (coherent 0.166187×,
white 3.101100×). Across 10 seeds the qualitative dichotomy is stable: all
coherent ratios remain <1, all white-noise ratios remain >1.

| seed | coherent rel/abs | white rel/abs | coherent corr | white corr |
|------|------------------|---------------|---------------|------------|
| 13025 | 0.166187 | 3.101100 | 1.000000 | 0.401548 |
| 13026 | 0.183648 | 2.986231 | 1.000000 | 0.401548 |
| 13027 | 0.214920 | 2.850399 | 1.000000 | 0.401548 |
| 13028 | 0.231852 | 3.287243 | 1.000000 | 0.401548 |
| 13029 | 0.244346 | 2.775567 | 1.000000 | 0.401548 |
| 13030 | 0.581656 | 3.264951 | 1.000000 | 0.401548 |
| 13031 | 0.201408 | 3.036457 | 1.000000 | 0.401548 |
| 13032 | 0.216251 | 3.155022 | 1.000000 | 0.401548 |
| 13033 | 0.220157 | 3.064077 | 1.000000 | 0.401548 |
| 13034 | 0.205111 | 3.005378 | 1.000000 | 0.401548 |

Summary:

| Metric | mean ± std |
|--------|------------|
| coherent rel/mean(abs) | 0.2466 ± 0.1199 |
| white rel/mean(abs) | 3.0526 ± 0.1625 |
| coherent corr | 1.0000 ± 0.0000 |
| white corr | 0.4015 ± 0.0000 |

Paper consequence: report coherent attenuation as ≈0.25× (not 0.166×);
retain 0.166× only as the frozen anchor seed.

## P1.8 Proposition 1 null-direction FIM

The independent paper test builds a 6D Fisher block over
`[b_bar(3), t_LW(3)]` with per-frame Jacobian `[R_LW R_WB_i, I]`.
For the Proposition 1 direction `d = [η; -R_LW Rbar_WB η]`, the locked-attitude
quadratic form is at machine precision and rises monotonically with attitude
dispersion.

| attitude span [deg] | rotation dispersion [deg] | mean dᵀF d | max dᵀF d | min eig(F) |
|---------------------|---------------------------|------------|-----------|------------|
| 0 | 0.000 | -1.65e-16 | 3.07e-16 | -3.14e-16 |
| 2 | 1.472 | 2.27e-04 | 3.12e-04 | 5.51e-05 |
| 5 | 3.679 | 1.42e-03 | 1.95e-03 | 3.45e-04 |
| 10 | 7.357 | 5.67e-03 | 7.79e-03 | 1.38e-03 |
| 20 | 14.708 | 2.25e-02 | 3.09e-02 | 5.52e-03 |
| 35 | 25.708 | 6.79e-02 | 9.29e-02 | 1.69e-02 |

Paper consequence: cite this as numerical confirmation of the singular limit and
continuous recovery; it supports, but does not replace, Fig. 2c's accuracy curve.

## Pending (pre-submission)

- **Field Section VIII:** still placeholder-only until real dual-Ruby data arrive.
- **Anonymous coplanar FIM citation:** final double-blind bibkey still needs author-side confirmation.
