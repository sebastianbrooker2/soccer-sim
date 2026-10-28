# Evaluation pipeline spec (shared contract for all agents)

Goal: reproduce the headline results of the cooperative-tracking paper claim:
for each of N matches, fraction of time a robot's estimate of the ball / each
opponent is "accurate", for (a) BASELINE = each robot tracks alone with a simple
Kalman filter (NUbots-style), (b) SHARED = cooperative filter (existing
`run_5v5_decentralised.m` machinery: info-form filter, bearing+range, data
association, lossy links). Results -> CSV + LaTeX (pgfplots / tables).

Language: MATLAB (R2026a). Run headless: `/usr/local/MATLAB/R2026a/bin/matlab -batch "..."`.
All new code lives in `/home/seb/soccer-sim/eval/` (package-free plain functions; add
`addpath(genpath(<repo root>))` as needed). Do NOT break existing scripts/tests.
Do not run docker or touch ~/test3v3, ~/NUbots*, ~/.soccer-train (read-only elsewhere).

## Ground truth CSV schema (long format, one row per object per tick)
`t,id,team,x,y,theta`
- t: sim seconds (monotonic, uniform grid preferred, e.g. 0.05 s).
- id: 0 = ball; 1..3 = team 1 (OUR team, the one whose robots observe); 4..6 = team 2 (opponents).
- team: 0 for ball, 1 or 2.
- x,y metres, absolute field frame (x forward, y left, field 14 x 9 m, goals at x=+-7,
  field centre origin); theta rad (0 for ball). Source = raw MuJoCo world / `/team1/*` topics.
MATLAB struct from loader: `gt.t (Nt x1)`, `gt.ball (Nt x2)`, `gt.pos (Nt x2 x6)`,
`gt.theta (Nt x6)`, `gt.team (1x6)`.

## Sensing
Each OUR-team robot (ids 1..3) has a head camera: FOV, Range, gaze sweep, rate; detects
the ball and the OTHER 5 robots (opponents AND teammates; teammates are identifiable
but treat as additional unlabeled/labelled per the tracker's existing handling) giving
bearing + range measurement with noise (existing `simulateRangeBearing` model; tuning later).
Self-pose uncertainty: observer position covariance + heading variance as in existing code.
All sensor params in ONE file `eval/config/evalParams.m` (struct), including K1-specific
values (FOV, Range, rate, noise stds, link drop prob, thresholds, eval grid dt,
confidence level 0.95, fixed radius list [0.25 0.5 1.0]). Detection packet format =
the one already used by `run_5v5_decentralised.m` (see :235-293).

## Trackers (same packet stream feeds both)
- SHARED: extracted from `run_5v5_decentralised.m` into callable functions
  (`processPacket`, `gateStats`, `spawnTrack`, `fuseDetection`, `mergeDuplicates`, ...) in
  `eval/tracker/`; original script must keep working (call the extracted functions or
  leave untouched if extraction is copy-only). Interface:
  `copies = runSharedTracker(packets, delivered, params)`; `copies(r).tracks` as existing.
- BASELINE: `copies = runBaselineTracker(packets, params)`. Per-robot, ONLY that robot's own
  packets, linear-ish constant-velocity KF (EKF on range/bearing or converted Cartesian with
  R from range/bearing), greedy NN, 0.5 m gate, delete after N misses; Q/R from the
  NUbots K1 report. Output has same `tracks` shape so metrics are tracker-agnostic.
- Common track-history accessor: `est = trackEstimates(copy, tEval)` -> for each class
  ('ball','robot') list of live tracks at tEval with `mean(1:2)`, `cov(2x2)`, `name`.

## Metrics (`eval/metrics/`)
Evaluate on grid tEval (params.evalDt) for every OBSERVING robot copy r, every true target
(ball; each of the 3 opponents = ids 4..6; teammates optional, off by default):
- estimate = live track of the right class with smallest Euclidean distance to truth
  (evaluation-side oracle matching only; trackers never see truth). No track => miss.
- err = ||mean - truth||.
- inRegion95 = (e' * inv(P_pos) * e) <= chi2inv(0.95, 2).
- withinR(k) = err <= radius(k) for params.radii.
- Report per match, per class (ball, opponent), per tracker, per robot, then mean over
  robots and over matches (mean +- std across matches). Also RMSE of matched tracks, and
  availability (fraction of time any track exists).
Primary headline = inRegion95 rate; "within X m" is secondary (X from params.radii).
Be careful: a track that is highly overconfident but wrong will score badly on
inRegion95; a huge-covariance track scores well. ALSO report median 95%-region area
so this cannot be gamed.

## Output (`eval/results/<runname>/`)
- `per_match.csv`: match,tracker,class,robot,inRegion95,within_0p25,within_0p5,within_1p0,rmse,availability,region_area_median
- `summary.csv`: tracker,class,metric,mean,std,nMatches
- `timeseries_match<k>.csv`: t,tracker,class,robot,target,err,inRegion95,hasTrack (downsampled option)
- `results_table.tex` (booktabs) and `fig_*.tex` pgfplots (use `\addplot table[col sep=comma]{...csv}`)
  reading the CSVs, so LaTeX re-renders without MATLAB. Plain `.tex` text; matlab2tikz
  not required (not installed).
- `results.mat` too.

## Driver
`eval/runEvaluation.m(opts)`: opts.gtFiles (cell of CSV paths; if empty -> synthetic
generator), opts.nMatches, opts.duration, opts.seed, opts.runName. For each match:
load GT -> make packets -> run shared + baseline -> metrics -> save. `eval/runDemo.m`
= short smoke test (2 matches x 60 s synthetic) that must pass end-to-end.

## Performance
Real runs: 10 matches x 20 min (existing sim matches are 10 min; concatenate/loop or
accept 10 min and document). MAP update per detection may be slow: profile a 60 s run,
report per-sim-minute wall time, and add `opts.parfor` / decimation knobs if needed.

## Ownership (avoid file collisions)
- Agent GT: eval/gt/*, eval/config/*, tools/replay_to_csv.py, tools/README_groundtruth.md
- Agent TRK: eval/tracker/* (shared extraction + trackEstimates) + eval/sensing/*
  (packet generation from gt: `packets = makePackets(gt, params)`)
- Agent BASE: eval/baseline/* (runBaselineTracker)
- Agent MET: eval/metrics/*, eval/export/*  (+ unit tests in eval/tests/ using synthetic fake copies)
- Integrator (later): eval/runEvaluation.m, eval/runDemo.m
