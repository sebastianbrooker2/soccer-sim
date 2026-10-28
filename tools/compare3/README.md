# compare3: NUbots vs own-only SRIF vs joint SRIF (+ raw measurement, tuned baseline) on real01

One game (real01, `packetSource='yolo'`, `rng(1)`), identical packets and ground truth for every behaviour, 0.2 s eval grid, one-to-one
matching (`evaluateMatch`, gate 3 m), 95% regions, `maxAreaForScore=3` for the capped column. Parameters: `eval/config/evalParams.m`
only (file mtime 2026-10-28 23:55:54; also unchanged: `makePacketsYolo.m` 23:41:35, `detectionsToPackets.m` 22:35, `yoloErrorModel.m` 22:36; the final run
started 2026-10-29 00:08 after >10 min of no changes; includes `spawnAnalyticBall=true`, `ballSingle`, `obsCovInflate=16`, `obsCovInflateBall=1`).

* NUbots = `runBaselineTracker`, mode faithful (raw YOLO boxes -> ground projection, constant R=0.1 I, no carried covariance model).
* Own SRIF = `runSharedTracker` with `delivered` true only where `packets(p).RobotID==r` (same code/params/packets as joint).
* Joint SRIF = `runSharedTracker`, all packets, `delivered = rand >= linkDropProb` (0.2), `rng(1)` as in `tools/viz/export_tracks.m` (delivered frac 0.801).
* Raw measurement = `eval/rawmeas` (`rawEstimates`, freshness 0.40 s, no tracking/prediction).
* Tuned baseline (reference) = baseline mode 'tuned'.
Files: `run_compare3.m` (everything), `results.csv` (all metrics, per tracker x class x scope all/robot1-3), `calib.csv`, `purity.csv`, `ghosts.csv`,
`window_analysis.m`/`window_pairs.m`/`dig_windows.m` (time-window diagnosis), `bias_autocorr.m`. Raw numbers: `results.csv`.
Note: samples before ~t=100 s have no detections at all, so availability is diluted equally for all trackers.
"live_minus_true" = mean live tracks - true count (ball 1, opponents 3); `false_tracks_per_sample` = live estimates not matched one-to-one to a target.
For opponents NUbots carries ~19 live robot tracks (13.5 ghosts/sample), so its high availability/in95 is bought with ghosts; capped in95 (area <= 3 m^2) shows it.

## Ball (all robots)
| tracker | availability | inRegion95 | inRegion95_cond | inRegion95_capped | within05 | rmse | median_err | median_area95 | mean_nees_cond | mean_live_tracks | live_minus_true | false_tracks_per_sample |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| NUbots (faithful) | 0.681 | 0.176 | 0.258 | 0.077 | 0.161 | 1.287 | 0.865 | 0.504 | 42.1 | 0.858 | -0.142 | 0.177 |
| Own SRIF | 0.442 | 0.412 | 0.932 | 0.261 | 0.325 | 0.788 | 0.248 | 1.791 | 5.440 | 0.461 | -0.539 | 0.018 |
| Joint SRIF | 0.669 | 0.611 | 0.914 | 0.475 | 0.534 | 0.728 | 0.194 | 1.178 | 7.059 | 0.680 | -0.320 | 0.011 |
| Raw measurement | 0.356 | 0.318 | 0.894 | 0.281 | 0.288 | 0.569 | 0.214 | 0.469 | 7.271 | 0.386 | -0.614 | 0.030 |
| Tuned baseline | 0.523 | 0.399 | 0.763 | 0.325 | 0.345 | 0.870 | 0.309 | 0.784 | 8.853 | 0.580 | -0.420 | 0.057 |


## Opponents (all robots)
| tracker | availability | inRegion95 | inRegion95_cond | inRegion95_capped | within05 | rmse | median_err | median_area95 | mean_nees_cond | mean_live_tracks | live_minus_true | false_tracks_per_sample |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| NUbots (faithful) | 0.812 | 0.768 | 0.946 | 0.176 | 0.436 | 0.892 | 0.460 | 28.2 | 2.638 | 19.6 | 16.6 | 17.2 |
| Own SRIF | 0.306 | 0.298 | 0.975 | 0.146 | 0.189 | 0.758 | 0.372 | 3.070 | 1.336 | 1.018 | -1.982 | 0.100 |
| Joint SRIF | 0.543 | 0.526 | 0.969 | 0.335 | 0.391 | 0.620 | 0.298 | 1.734 | 1.576 | 1.782 | -1.218 | 0.154 |
| Raw measurement | 0.159 | 0.154 | 0.972 | 0.109 | 0.127 | 0.475 | 0.274 | 1.543 | 1.710 | 0.485 | -2.515 | 0.009 |
| Tuned baseline | 0.292 | 0.285 | 0.974 | 0.144 | 0.184 | 0.670 | 0.370 | 2.928 | 1.356 | 0.940 | -2.060 | 0.064 |


## Per robot (ball)
| tracker | robot | avail | in95 | in95 capped | within 0.5 m | rmse | med err | NEES(cond) |
|---|---|---|---|---|---|---|---|---|
| NUbots (faithful) | 1 | 0.657 | 0.186 | 0.076 | 0.160 | 1.27 | 0.85 | 35.91 |
| NUbots (faithful) | 2 | 0.650 | 0.177 | 0.068 | 0.153 | 1.27 | 0.83 | 45.97 |
| NUbots (faithful) | 3 | 0.735 | 0.163 | 0.086 | 0.170 | 1.32 | 0.97 | 44.10 |
| Own SRIF | 1 | 0.415 | 0.390 | 0.247 | 0.314 | 0.79 | 0.23 | 3.27 |
| Own SRIF | 2 | 0.458 | 0.427 | 0.269 | 0.322 | 0.79 | 0.26 | 3.96 |
| Own SRIF | 3 | 0.455 | 0.419 | 0.268 | 0.337 | 0.78 | 0.25 | 8.92 |
| Joint SRIF | 1 | 0.671 | 0.616 | 0.475 | 0.536 | 0.72 | 0.19 | 6.18 |
| Joint SRIF | 2 | 0.669 | 0.609 | 0.472 | 0.530 | 0.73 | 0.20 | 7.18 |
| Joint SRIF | 3 | 0.666 | 0.610 | 0.478 | 0.538 | 0.73 | 0.19 | 7.83 |
| Raw measurement | 1 | 0.306 | 0.268 | 0.247 | 0.252 | 0.54 | 0.20 | 7.16 |
| Raw measurement | 2 | 0.352 | 0.311 | 0.277 | 0.293 | 0.54 | 0.21 | 9.91 |
| Raw measurement | 3 | 0.410 | 0.377 | 0.318 | 0.318 | 0.61 | 0.23 | 5.08 |
| Tuned baseline | 1 | 0.482 | 0.362 | 0.291 | 0.316 | 0.88 | 0.32 | 8.68 |
| Tuned baseline | 2 | 0.539 | 0.419 | 0.317 | 0.362 | 0.88 | 0.31 | 10.05 |
| Tuned baseline | 3 | 0.548 | 0.417 | 0.368 | 0.358 | 0.85 | 0.29 | 7.83 |


## Per robot (opponents)
| tracker | robot | avail | in95 | in95 capped | within 0.5 m | rmse | med err | NEES(cond) |
|---|---|---|---|---|---|---|---|---|
| NUbots (faithful) | 1 | 0.813 | 0.781 | 0.172 | 0.455 | 0.79 | 0.43 | 1.45 |
| NUbots (faithful) | 2 | 0.847 | 0.810 | 0.188 | 0.401 | 0.95 | 0.54 | 2.66 |
| NUbots (faithful) | 3 | 0.776 | 0.713 | 0.170 | 0.453 | 0.92 | 0.40 | 3.86 |
| Own SRIF | 1 | 0.283 | 0.279 | 0.135 | 0.179 | 0.82 | 0.35 | 1.14 |
| Own SRIF | 2 | 0.321 | 0.315 | 0.163 | 0.218 | 0.73 | 0.32 | 1.24 |
| Own SRIF | 3 | 0.314 | 0.301 | 0.140 | 0.169 | 0.72 | 0.46 | 1.61 |
| Joint SRIF | 1 | 0.546 | 0.530 | 0.335 | 0.388 | 0.62 | 0.30 | 1.57 |
| Joint SRIF | 2 | 0.542 | 0.525 | 0.338 | 0.395 | 0.62 | 0.30 | 1.55 |
| Joint SRIF | 3 | 0.540 | 0.523 | 0.332 | 0.391 | 0.61 | 0.30 | 1.61 |
| Raw measurement | 1 | 0.133 | 0.130 | 0.106 | 0.118 | 0.39 | 0.23 | 1.74 |
| Raw measurement | 2 | 0.169 | 0.165 | 0.118 | 0.143 | 0.46 | 0.27 | 1.65 |
| Raw measurement | 3 | 0.174 | 0.168 | 0.104 | 0.121 | 0.55 | 0.34 | 1.75 |
| Tuned baseline | 1 | 0.258 | 0.254 | 0.134 | 0.175 | 0.66 | 0.33 | 1.11 |
| Tuned baseline | 2 | 0.313 | 0.305 | 0.161 | 0.215 | 0.64 | 0.33 | 1.28 |
| Tuned baseline | 3 | 0.305 | 0.295 | 0.139 | 0.161 | 0.71 | 0.46 | 1.64 |


## Ghosts, duplicates and identity swaps (0.5 s grid, truth used for evaluation only)
| tracker | ghost balls/sample | dup balls/sample | ghost robot tracks/sample | dup robot tracks/sample | opponent track identity swaps (per copy, 0.5 s grid) |
|---|---|---|---|---|---|
| NUbots (faithful) | 0.351 | 0.000 | 13.541 | 4.007 | 215 |
| Own SRIF | 0.063 | 0.000 | 0.121 | 0.037 | 48 |
| Joint SRIF | 0.068 | 0.000 | 0.151 | 0.081 | 99 |
| Tuned baseline | 0.114 | 0.000 | 0.068 | 0.026 | 40 |


## Association purity of the SRIF copies (event log vs `TrueLabel`, diagnostics only)
| copies | class | fusions | purity (dominant-label match) | frac of fused dets that are false positives | spawns | duplicate spawns | tracks dominated by FP | label changes inside a track |
|---|---|---|---|---|---|---|---|---|
| joint (3 copies) | ball | 10437 | 0.979 | 0.079 | 824 | 220 | 381 | 0 |
| joint (3 copies) | robot | 13826 | 0.902 | 0.001 | 379 | 156 | 8 | 682 |
| own (3 copies) | ball | 3682 | 0.983 | 0.083 | 272 | 46 | 108 | 0 |
| own (3 copies) | robot | 5310 | 0.907 | 0.001 | 258 | 60 | 2 | 197 |


Robots: purity in joint 0.90 vs own 0.91 overall (own robot 3 only 0.84); but joint has 3.5x the label changes (682 vs 197 over 3x more fusions), 2.7x duplicate spawns and 2x the identity swaps on the grid.
Ball: purity is 0.98 for both; the ball problem is not mixing balls but false positives: ~8% of fused ball detections are false positives and ~35-40% of all ball tracks spawned are dominated by false positives (joint 381/824).

## Findings: why the joint SRIF does not beat the individual filters everywhere
Overall the joint filter wins clearly: of paired (robot, time) samples, ball is within 1 m for joint but not own in ~2480 samples vs the reverse in ~170; opponents ~8100 vs ~470.
Joint is worse than own in no 20 s window on within-1 m (see `windows.csv`), but the following 10 s windows are where it is worse (own good, joint not / joint error larger):

1. Ball t=460-470 (worst ball window; 23-26 own-good-joint-not samples; joint error 1.5 m vs own 0.4 m when both track). The ball is at (-7.1,-3.5) at the far end;
   robot 2 and 3 only produce false-positive ball detections (12 and 36 in the window, 1 true ball). Robots 2 and 3's rays corroborate each other (`spawnPending`
   ray-intersection corroboration), so each joint copy spawned 23-28 duplicate ball tracks (29-32 spawns in 10 s) and sat on a ghost 1.8-2.8 m from the ball. Cause: data association / spawn of false positives, amplified by cooperation (two observers agreeing on a false target).
2. Ball t=250-260. Robot 2 sees 20 false positives vs 15 true; joint copy 2 follows a ghost 2.4 m away (sd 0.79) while joint copies 1,3 are 0.3-0.4 m off. Same cause (FP corroboration), plus joint copies spawned 24-26 tracks vs 2-11 in own copies.
3. Ball t=720-730 (over-confidence): truth ball moves ~4.7 m in this window. Joint tracks are 1.0-1.7 m off with sd 0.13-0.19 m (NEES ~50-100); own tracks are equally lagging but with sd 0.14-1.2. 3x the information makes the
   covariance 2-3x tighter while the ball process model lags a kick/relocation, so the track is confidently wrong, and robot 3 fused 18 wrong-label (fp) detections.
4. Opponents t=430-440 (worst opponent window; 33-36 samples each of "worse error" and "lost"): opp1 and opp2 are 1.3 m apart (opp3 3 m away), robot 3 sees them 35/23/18 times. Joint copies log 54-63 wrong-label fusions each (opp1<-opp2, opp3<-opp1 ...);
   tracks swap identities, sd 0.5-3.2 m, and tracks 3 m off persist. Own r3 has the same problem (57 wrong-label) but own r1/r2 simply have no tracks; so joint inherits r3's association errors and spreads them to all three copies.
5. Opponents t=190-200 and 580-590: again opponents 0.3-1.3 m apart (opp1/opp2); joint 48-54 wrong-label fusions per copy in 190-200 (own r1 18, own r3 39). At 580-590 labels are mostly right (1 wrong) but all three joint copies delete their three tracks as 'lost' at t=587.8-588.8
   (sd grew past `lostStd`=4) simultaneously, because the only observer seeing them (robots 1 and 2, 5-13 detections) stops; joint then loses what own copies also lose.

Hypothesis check:
* Data association is the main culprit (not a calibration problem): cooperation multiplies the number of fused detections (3.3x) so every association error is replicated in all three copies, and corroborated false positives survive `spawnPending`. Robot-track label changes 682 vs 197, duplicate robot spawns 156 vs 60, identity swaps 99 vs 48.
* Per-observer measurement calibration (`calib.csv`, z = error / sigma from the packet incl. observer pose cov, evaluated at the true target, per detection) shows NO observer that is over-confident:
  ball z_tang rms 0.76-0.87, z_range rms 0.83-0.95 (nominal 1), NEES2 1.27-1.64 (nominal 2) for observers 1-3; opponent z_tang 0.72-0.78, z_range 0.84-0.95 with a systematic range bias of -0.27 to -0.55 sigma (measured range too short), NEES2 1.26-1.45 stated and 1.01-1.21 with the x16 inflation.
  Observer 3 (goalkeeper, far: most opponent detections at >7 m) is if anything the most pessimistic for the ball (NEES2 1.27) and the most biased in range (-0.55 sigma) for robots; observer 2 is the least conservative (ball NEES2 1.64; 4.8% of |z_range|>2 at 4-7 m). Single-detection sigmas are slightly too large (partly the unsimulated 2 deg heading variance), so per-detection over-confidence is not the cause.
* Track-level over-confidence does exist for the ball: conditional NEES 5.4 (own), 7.1 (joint), 8.9 (tuned), nominal 2, and in95 conditional is only 0.91-0.93 despite tight 95% regions. This is the white-noise treatment of AR(1) pose error (ball `obsCovInflateBall=1`), plus process-model lag, and it is worse for joint (3x tighter) -- the 720 s window.
  For opponents NEES is 1.3-1.6 (close to 2, slightly conservative) thanks to inflate 16; joint in95_cond 0.969.
* Counter-effects in favour of joint: capped in95 for ball 0.475 vs 0.261 (own) vs NUbots 0.077; opponents 0.335 vs 0.146 vs 0.176; rmse 0.73/0.62 vs 0.79/0.76.
* Raw measurement: lowest rmse (ball 0.57, opp 0.48) and best conditional in95, smallest region area, but it only has an estimate when a detection exists (availability 0.36/0.16): the filters trade accuracy for coverage. Joint beats raw on availability and capped in95 for both classes; raw beats joint on rmse/median err for the cases it covers (it is conditioned on being detected, so not a like-for-like error).
  Joint ball within 0.5 m: 0.534 (best of all); opponents within 0.5 m: NUbots 0.436 (with 17 ghosts/sample), joint 0.391.
* NUbots (faithful) ball: availability 0.68, but in95 0.18 (NEES 42; constant R=0.1 gives tiny covariance for 1-3 m errors), rmse 1.29. Opponent: 19.6 live tracks vs 3 true.

Suggested remedies (not applied; eval/ is not mine to edit): require corroboration of ball spawns by two *different* observers with a consistency check against `TrueLabel`-free evidence (e.g. both detections must also pass the YOLO conf/size test, and ignore pairs that repeatedly create tracks at the same field point = static FP location);
gate robot association on identity-aware cues when two opponents are within ~1.5 m (do not fuse a detection into either track if the two innovation gates both pass: keep the pair as ambiguous / use JPDA-style soft weighting) and cap per-scan fusions;
inflate ball `ObserverCovariance` for repeat fusions from the same observer within a correlation time (`obsCovInflateBall` > 1, e.g. 4) or add a process-noise floor to avoid the 0.13 m sd.

## How observer-pose uncertainty enters the SRIF (verified in code)

Chain: `evalParams.sensor.observerPosStd (0.1 m)`, `headingStd (2 deg)`, `posCorrTime (5 s)`
-> `eval/sensing/makePacketsYolo.m` -> packet -> `eval/tracker/makeMeasurement.m` -> `@MeasurementBallBearing/noiseDensity.m`.

1. Simulation of the error (`makePacketsYolo.m`, per observer): `bias = a*bias + sqrt(1-a^2)*pStd*randn(2,1)`, `a = exp(-dt/posCorrTime)`;
   `ObserverPosition = true position + bias` (believed pose). The camera geometry that turns boxes into (bearing, range)
   uses the believed pose, so the bias lands in every measured world position. The heading is NOT perturbed
   (`poses.theta = gt.theta`), so the stated heading variance is extra (conservative) slack. Head yaw/pitch and IMU tilt get
   `headStd`/`tiltStd` noise that feeds the per-detection bearing/range sigma.
2. What the packet states (`detectionsToPackets.m` lines ~104-107): `ObserverCovariance = posStd^2 * I2` (a fixed configured
   number, the same for every packet of an observer), `ObserverHeadingVariance = headingStd^2`, `DetectionNoiseStd = hypot(pix/fx, headStd)` (+ per-detection
   `bearingStd`), per-detection `LastDetection.RangeStd`. None of this is derived from ground truth; the realised bias is never in the packet.
   (TrueLabel/TrueDelta are diagnostics only; `makeMeasurement`/`noiseDensity` never read them.)
3. `makeMeasurement.m`: `m.ObserverCovariance = pkt.ObserverCovariance * infl` with `infl = tracker.obsCovInflate (16)` for robot-class
   detections and `obsCovInflateBall (1)` for balls; `RangeNoiseStd` = per-detection `LastDetection.RangeStd`.
4. `noiseDensity.m` (linearised at the prior mean, t = unit tangent, u = unit line of sight, r = range to the current estimate):
   `R = blkdiag(sT2*I2, sR2)` with `sT2 = DetectionNoiseStd^2 + ObserverHeadingVariance + t'*C*t/r^2`,
   `sR2 = RangeNoiseStd^2 + u'*C*u`, `C = infl*ObserverCovariance`. So the observer covariance enters (a) tangentially divided by r^2 (bearing),
   (b) along the ray un-divided (range). Heading variance is NOT divided by r.
5. Own-SRIF and joint use IDENTICAL code, parameters and packets (`runSharedTracker` -> `processPacket` -> `fuseDetection` -> `makeMeasurement`);
   each packet is processed with its own sender's covariance in both. The only difference is which packets a copy receives.
6. AR(1) correlation: the filter treats each measurement's pose error as independent white noise. Measured on real01 packets
   (`bias_autocorr.m`): std 0.09-0.10 m/axis, autocorrelation 0.89 (0.5 s), 0.81 (1 s), 0.67-0.71 (2 s), 0.41-0.44 (5 s), 0.14-0.26 (10 s), as configured.
   At 15 Hz a robot delivers ~75 samples per correlation time carrying nearly the same bias, so repeated fusion of one observer's ball
   detections shrinks the covariance as if the bias averaged out (like 1/sqrt(N)) while the real error floor is the bias itself (0.1 m/axis).
   For robots this is patched by `obsCovInflate=16` (sigma x4); for the ball it is 1, hence ball NEES is above nominal (see tables).
   Cooperating observers have independent biases, so joint fusion DOES average them down (floor 0.1/sqrt(3)) -- the white-noise assumption is
   less wrong across observers than within one observer; but it is also more confident (tighter ellipses) when the ball moves/jumps and the filter lags.
