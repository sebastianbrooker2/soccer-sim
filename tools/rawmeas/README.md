# Raw measurement behaviour (no tracking)

Fourth behaviour next to NUbots-faithful, own SRIF (no sharing) and joint SRIF: **no filter, no prediction, no
association memory, no sharing**. Every detection of every YOLO packet of robot r becomes an instantaneous
position estimate; if the target is not detected in the latest packet there is no estimate (out of sight, out of mind).

* `eval/rawmeas/runRawMeasurement.m (packets, params) -> copies(r)` : per-robot per-packet points + covariances.
* `eval/rawmeas/rawEstimates.m (copy, tEval, class, params)` : the estimates of the most recent packet with time in
  (tEval - freshness, tEval], `params.rawmeas.freshness` (default 0.25 s ~ one YOLO frame interval). Mean = measured point
  as-is. Same fields as `trackEstimates` (+ `lastUpdate`), so it plugs into `evaluateMatch`:
  `fn = @(c,t,cl) rawEstimates(c,t,cl,params)`. Not a fastCopies object; no conversion needed.
* Measurement math = `@MeasurementBallBearing/noiseDensity.m` pushed to a point: mean = ObserverPosition + range*u;
  tangential var sT2 = DetectionNoiseStd^2 + ObserverHeadingVariance + t'C t/range^2, range var = RangeStd^2 + u'C u
  (RangeStd = LastDetection.RangeStd if finite, else packet RangeNoiseStd; C = ObserverCovariance);
  cov = range^2*sT2 t t' + sR2 u u'. No obsCovInflate, no prior / process noise.
* Packets keep their YOLO confidence thresholds and teammate drop; false positives are kept.
* `tools/rawmeas/run_rawmeas.m` computes metrics (real01, same packets, rng(1), evalDt 0.2, maxAreaForScore 3, one-to-one
  matching, gate 3 m) -> `results.csv` (columns identical to `tools/compare3/results.csv` + `false_tracks_per_sample`).
  `false_tracks_per_sample` = live estimates of the class not matched one-to-one to a true target
  (evaluateMatch's own `falseTracks` counts all 5 robot targets, so it is recomputed). `live_minus_true` is the usual
  nLive - nTargets (negative = under-reporting).
* Viewer: 4th minimap row (orange x = ball, cyan x = robot, 95% ellipses); `tools/viz/export_tracks.m` exports `src=raw`
  (set `rawOnly=true` in the workspace to only recompute raw rows and keep the other sources; tracks.csv written via temp + movefile).
