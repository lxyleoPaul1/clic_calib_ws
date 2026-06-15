# Submission Notes

## Target

- Venue target: ICRA, oral-track ambition.
- Current manuscript state: submission-polish draft with simulation evidence complete
  and a concrete field protocol; field result cells remain placeholders.
- Double-blind posture: anonymous authors, no acknowledgments, and no identifying
  source-history or institution markers in the manuscript.

## Scope Boundaries

- Do not fabricate or infer field numbers. Section VIII protocol is concrete, but all
  measured result cells remain placeholders until hardware data are available.
- Sphere targets are evaluation rulers in this paper, not a calibration fallback.
- The withdrawn NE/SW sector-effect interpretation must not be revived; current paper
  conclusion is that the sectors are statistically comparable across 10 seeds.
- The relative-extrinsic attenuation claim is simulation-scoped: coherent systematic
  bias attenuates to about 0.25x on average over seeds, but field cancellation may be
  weaker.

## Pre-Submission Must-Do Checklist

- Fill Section VIII result cells with real dual-Ruby field data: site/setup metadata,
  sphere-survey uncertainty, board-free per-post accuracy, center-reg, field bias
  diagnostics, and POI/gate statistics.
- Fill the Campaign D base-station-offset replay table and compare against the
  simulation multiseed ratios (coherent 0.247±0.12x; white 3.05±0.16x).
- Resolve the remaining citation verification item in `PLACEHOLDERS.md`: the
  anonymous coplanar-FIM reference needs its final double-blind bibliography entry.
- Rebuild the PDF after field insertion and confirm the ICRA 6+2 page budget.
- Re-run the full paper grep for old anchor-only numbers after field insertion.
- Repeat the double-blind grep: no author names, institutions, source URLs,
  revision identifiers, or identifiable acknowledgments.

## Formatting / Compliance Checks

- IEEEtran conference class.
- Main text currently targets seven pages before final field data insertion.
- All field TODOs must either be filled with measured data or remain explicit placeholders; no red TODO should survive in the submitted PDF.
