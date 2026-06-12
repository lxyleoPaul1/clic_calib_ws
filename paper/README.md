# ICRA oral paper scaffold (`clic_calib`)

LaTeX skeleton for board-free multi-LiDAR calibration (ICRA, oral target).
**Does not modify calibration code or frozen baselines.**

## Dependencies

- TeX Live (or MiKTeX) with `IEEEtran` conference class
- Packages: `amsmath`, `graphicx`, `xcolor`, `booktabs`, `tikz`, `cite`

Install (Ubuntu/Debian):

```bash
sudo apt-get install texlive-latex-extra texlive-fonts-recommended texlive-science
```

`IEEEtran.cls` is provided by `texlive-publishers`. If missing:

```bash
sudo apt-get install texlive-publishers
```

## Build

```bash
cd paper
make pdf
# output: main.pdf
```

Or:

```bash
bash build.sh
```

## Content map

| File | Status |
|------|--------|
| `sec_framework.tex` | Written (simulation architecture) |
| `sec_mechanism.tex` | Written (oral core, simulation numbers) |
| `sec_relext.tex` | Written (§7.2, simulation + scope) |
| `sec_pw.tex` | Written (simulation) |
| `sec_sim.tex` | Written (frozen baseline numbers) |
| `sec_field.tex` | Placeholders only (`\TODO`) |
| `sec_intro.tex` | DRAFT |
| `sec_related.tex` | DRAFT |
| `sec_conclusion.tex` | Written + future work |

See `PLACEHOLDERS.md` and `figure_table_plan.md`.

## Simulation flight labels

| Macro | Repo label | Role |
|-------|------------|------|
| `\flightD` | 丁 | Wide $u_B$ sweep, decoupled orbit |
| `\flightW` | 戊 | POI-locked dual-sector serial |
| `\flightJ` | 甲 | Gate OFF, near-locked attitude |
| `\flightY` | 乙 | Low yaw\_cv guard rollback |

## Frozen baseline refs

Docs: `doc/board_free_mechanism.md`, `doc/board_free_results_frozen.md`.
Commits: `f03c399`, `3fb7ddf`, `7f5c27c`.
