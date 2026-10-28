# CLAUDE.md — soccer-sim

Cooperative ball and opponent tracking for a RoboCup humanoid team (Booster K1), evaluated offline on
ground truth and detections from the Booster Studio simulator. `README.md` has the setup and a short
overview; this file records the models, the decisions and how the project got to where it is, so a new
session can continue without re-deriving anything. Read `eval/SPEC.md` for the evaluation contract.

## 1. What this project is

Question: does fusing every robot's observations into one **joint square-root-information-form (SRIF)
filter** track the ball and opponents better than each robot tracking alone (the NUbots approach)?
Four behaviours are compared on the same ground truth and the same YOLO detections; only the tracking differs:

| Behaviour | What it is | Code |
|---|---|---|
| **NUbots (faithful)** | Per-robot linear Kalman filter. Position measurement from ground-projecting the YOLO box, constant R = 0.1·I, vision covariance ignored. NUbots main deletion logic. | `eval/baseline/runBaselineTracker.m` (mode `faithful`), `eval/sensing/nubotsMeasurement.m` |
| **Tuned baseline** | Same, but R built per detection from range/bearing/pose noise, Mahalanobis gating, tracks age out after 3 s. Strongest fair per-robot competitor. | `eval/baseline/runBaselineTrackerTuned.m` |
| **Own SRIF** | The SRIF tracker, each robot receiving only its own packets (no sharing). | `eval/tracker/runSharedTracker.m` with a diagonal `delivered` mask |
| **Joint SRIF** | The same tracker, every robot's packets, 20 % simulated link drop. Expected to be the best. | `eval/tracker/runSharedTracker.m` |
| **Raw measurement** | No tracking. Every detection is an instantaneous estimate with its own covariance; no prediction, estimate valid 0.40 s. | `eval/rawmeas/` |

The SRIF stack itself (`@GaussianInfo`, `@SystemBall`, `@MeasurementBallBearing`, `@Event`, `+funcmin`)
predates this work and is shared with the course scripts `run_5v5_*.m` in the repo root. Do not break
those; the evaluation code copies and extends their functions rather than editing them.

## 2. Pipeline

```
Booster sim (docker, MuJoCo)  --record-->  recordings/<id>/data  (replay jsonl: ball, 6 robots, joints)
   tools/record/record_match(es).sh                |
                                                   v
                              tools/replay_to_csv.py  ->  eval/gt/<id>.csv      (ground truth)
                                                   |
        tools/yolo/detect_replay.py (renders head cameras with MuJoCo EGL, runs NUbots YOLO11n ONNX)
                                                   v
                              recordings/<id>/yolo/dets.csv, dets_all.csv, dets_frames.csv
                                                   |
   eval/sensing/makePacketsYolo.m  (boxes -> bearing+range packets with calibrated noise)
                                                   v
   trackers (NUbots / tuned / own SRIF / joint SRIF / raw)  ->  eval/metrics  ->  eval/export (CSV, LaTeX)
```

Only `real01` has YOLO detections and a tracked ground-truth CSV; the other nine recorded matches
(`real02..10`, ~10–12 min of sim time each) have raw recordings only and are not in git.

## 3. Ground truth and the Booster sim

- Sim: Booster virtual-robot MuJoCo image (docker), 3v3 on a 14 × 9 m pitch, scene `football_match_pitch_6_K1.xml`.
  Home team = robots 1–3 (the observers), away = 4–6 (the opponents). Home defends x = −7.
- Replay recorder (`MATCH_REPLAY_ENABLED=1`): output must go into a **subdirectory** of the bind mount
  (the writer does `rmtree` on its output dir). Replay frames arrive at only ~6.6 Hz in sim time with jitter
  (median gap 0.154 s, max 0.37 s), not the nominal 20 Hz.
- GT CSV: `t,id,team,x,y,theta,z,roll,pitch,headYaw,headPitch` (id 0 = ball, 1–3 ours, 4–6 opponents).
  Loader: `eval/gt/loadGroundTruth.m` → `gt.t, ball, pos (Nt×2×6), theta, team, z, roll, pitch, headYaw, headPitch, hasHead`.
- **The robots' heads are essentially fixed** (yaw/pitch std ~0.01 rad): the camera looks along the trunk
  heading. The old sinusoidal gaze sweep in the synthetic/cone models is unrealistic for this sim.
- Camera: head camera 320 × 240, fovy 58° (hfov ≈ 74°), fx = fy = 216.5, cx = 160, cy = 120, mounted on the head
  (camera chain: trunk → yaw joint → pitch joint → camera at (0.0613, 0, 0.108)); `eval/sensing/cameraPoseK1.m`.
- The sim's built-in "YOLO" (`detection_extension.py`) is **not** a neural net. It projects ground truth
  through the camera, reports balls (and field markers) only, score always 1.0, ball error ~0.02 m. It never
  detects robots. We use it only to validate our visibility rule (99.5 % agreement on 12 272 robot-frames).
- The sim has **no referee signal in our recordings**. `/soccer/game_controller` (GameController v19 JSON,
  `setPlay`, penalties) exists live but was not logged. The replay's per-robot `active` flag marks removals.
- Teleports in `real01`: **no ball teleports** (max ball speed 6.2 m/s; the ball rests in the net or on
  the touchline then is rolled back); **10 robot teleports** (5 at the kickoff after the goal, t = 135.55 s).
  Events: `tools/ballstudy/events_real01.csv`.

## 4. Detections: YOLO

- Models (copied from `~/NUbots_K1/module/vision/Yolo/data`, in `tools/yolo/models/`): `yolo_hsl_ft.onnx`
  (fine-tune; best for ball, almost no robot recall), `yolo.onnx` (robot), `old.onnx` (YOLOv8; worse).
  Final pipeline: ball and field features from `yolo_hsl_ft`, robots from `yolo`. Classes: ball, goal post, robot, L/T/X crosses.
- Pre/post-processing follows NUbots `Yolo.cpp`: letterbox to 640, RGB/255, class-agnostic NMS 0.5, per-class
  thresholds (ball 0.6, goal post 0.5, robot 0.2, intersections 0.65). `onnxruntime-gpu` needs the venv's NVIDIA
  libs on `LD_LIBRARY_PATH` (`tools/yolo/ldpath.sh`), otherwise it silently falls back to CPU.
- Frames are rendered offline from the replay (`tools/render`, MuJoCo EGL, ~25–30 fps/robot). Robots render white.
  A frame pass over `real01` at 5 Hz, 3 robots takes ~80 s.
- Measured quality on `real01` against projected ground truth: ball recall 0.98 (0–2 m) → 0.21 (8 m+), precision
  0.86; robot recall 0.82 → 0.36, precision 0.93; ~18 ball and ~25 robot false positives per minute per robot.
  The ball box is **saturated for small balls** (a 4–5 px ball comes back 7–8 px wide), so size-based range is
  30–47 % short at 8 m+.
- CSV schema: `t,robot,class,u_px,v_px,w_px,h_px,conf` (box centre, 320 × 240 px).

## 5. Measurement model

**SRIF measurement** (`@MeasurementBallBearing`): for a target at p seen from observer o (believed pose):
`y = [u; r]`, `u = (p−o)/|p−o|`, `r = |p−o|` (range-bearing; bearing-only if `RangeNoiseStd` is NaN).
Noise (`noiseDensity.m`) is block-diagonal on `[u; r]`:

- tangential variance `sT² = DetectionNoiseStd² + ObserverHeadingVariance + tᵀ C t / r²`
- range variance `sR² = RangeNoiseStd² + uᵀ C u`
- `C = ObserverCovariance · obsCovInflate` (a fixed configured pose covariance; **never from ground truth**).

**Packets** (`makePacketsYolo.m`, format of `run_5v5_decentralised.m`): `time, RobotID, ObserverPosition,
ObserverCovariance, DetectionNoiseStd, RangeNoiseStd, ObserverHeadingVariance, LookHeading, FOV, Range,
detections{y, Class, LastDetection{RangeStd, BearingStd, LowQuality, Pixel}}`, plus `RawBoxes`, `CamPose`, `Cam`
used by the NUbots measurement. `TrueLabel` (`ball`/`team<j>`/`opp<o>`) exists for diagnostics only; **no tracker
may read it** (the baseline's teammate filter uses it as a stand-in for the teammate id that comes over the network).

**Observer pose error:** the believed pose is truth + an AR(1) bias (std 0.1 m per axis, correlation time 5 s)
generated in `makePacketsYolo`; the packet carries the *fixed* configured covariance, not the realised bias.
The filter treats each measurement's pose error as independent, which makes repeated fusion from one observer
slightly overconfident (ball NEES above nominal). Heading error is **not** simulated; the packet's heading
std is 0.5° (`sensor.yolo.headingStd`). If heading noise is ever injected, put 2° back.

**Calibrated YOLO error model** (`yoloErrorModel.m`, `detectionsToPackets.m`; fitted on `real01`, regenerate with
`tools/yolo/fit_tables.py`, `fit_k.py`, `gen_model.py`; data in `tools/yolo/calib/`): subtracts the ball box-size
bias and row bias; fuses ground-plane and size-based range by inverse variance using calibrated variances;
inflates by a heavy-tail factor so ~99 % of errors fall inside 3σ; adds confidence factors; flags
`LowQuality` (ball box < 8 px wide, not round, or low confidence when small) with extra uncertainty rather than
dropping the detection. Ball range sd: 0.09 / 0.18 / 0.51 / 0.87 / 1.2 m at 2 / 4 / 6 / 8 / 10 m.
Validation (true positives): normalised range error sd 0.92 ball, 0.91 robot; 99 % inside 3σ.
The calibration is in-sample on `real01` only.

**NUbots measurement** (`nubotsMeasurement.m`, from `~/NUbots_K1/module/vision/Yolo/src/Yolo.cpp`): ball = the
box **centre** ray intersected with the ground plane (so range is ~13–15 % long because the ball centre is
0.107 m up; angular size is *not* used); robot = box **bottom-centre** ray to the ground (foot point). Box coords
truncated to integers, strict `>` confidence thresholds, no range filters. The localisers fuse `Hwc · rBCc` with a
**constant R = 0.1·I and ignore any vision covariance**.

## 6. Process model and filter

- State per target: `[px, py, vx, vy]` (`@SystemBall`): `f = [vx; vy; −κ vx; −κ vy]`, κ = `kappaBall` 0.2 (ball
  friction; robots κ = 0), white acceleration noise `AccelNoiseStd` on the velocity states, scaled by √dt.
  Current values: ball 2.0 m/s² (was 2.5), robot 0.5 m/s². Each target is its own `SystemBall`, not a joint state.
  Time update: RK4 propagation of the SDE plus process noise (`@SystemEstimator/predict.m`).
- **Measurement update is a MAP minimisation** (`@Measurement/update.m`, `funcmin.BFGSTrustSqrt`): minimise
  `−log prior − log likelihood`, because `h` (bearing/range) is nonlinear and the posterior is not Gaussian;
  the posterior covariance is the inverse Hessian at the minimum (kept as the square-root information `Xi`).
  Pitfall: BFGS builds its Hessian from the steps it takes. If the prior mean already sits on the measurement it
  takes no step and the covariance is unchanged. That produced a 3 m round ellipse after the first fusion
  (long axis *across* the ray). Fix: **analytic initial covariance** at spawn (`spawnAnalytic`, `spawnAnalyticBall`):
  deep along the line of sight, narrow across it.
- Tracker logic (`eval/tracker/processPacket.m`; copied from `run_5v5_decentralised.m` with knobs added, all
  defaults in `eval/config/evalParams.m` block `tracker`): predict every live track to packet time; χ² gate (3σ,
  3 dof, association cost d² + log det S); greedy one-to-one assignment; unmatched detections spawn tracks;
  cannot-link between tracks that took different detections in one scan; delete after `maxMisses` = 5
  expected-visible misses, or when position sd > `lostStd` (robots 4 m) / `ballLostStd` (8 m); merge duplicates
  (`mergeDuplicates`, `mergeSigma` 2, `mergeMaxStd` 0.6); no robot spawns closer than `minSpawnRange` 1.5 m.
  Ball: `ballSingle` (one live ball track per copy), `ballReplaceStd`, `ballResetN` restart after repeated gate
  failures, `spawnMaxRangeStd` 0.6 + `spawnPending` (a poor detection waits for a corroborating second robot,
  confirmed at the ray intersection). Off by default and **not enabled**: `ballResetTimes`, `robotResetEvents`
  (referee stand-ins: they hurt or did nothing on `real01`).
- Current settings worth knowing: `obsCovInflate` 2 (was 16), `obsCovInflateBall` 1, `sensor.yolo.headingStd` 0.5°,
  `linkDropProb` 0.2, `dropTeammateDetections` true.

## 7. NUbots baseline details

Ported from `~/NUbots` **main** `RobotLocalisation.cpp` / `BallLocalisation.cpp` (config values identical in K1):
nearest-track association at 0.5 m, otherwise spawn; a miss counts only while the track is predicted inside the
view (stand-in for the green horizon: packet FOV and range), out-of-view tracks count as seen; delete after
> 25 misses, outside field + 1 m, or within 0.5 m of any already-kept robot (**proximity rule**). Ball: one filter
never deleted, nearest candidate always fused, predict only on measurement, optional team-ball fallback.
**`~/NUbots_K1` (branch `jmontano/nusim-local`) has the proximity rule broken** (commented out and replaced by a
dead-code inner-loop `continue`), so do not port logic from K1 blindly; `b.k1ProximityBug = true` reproduces it.
Even with the main rule the faithful baseline keeps ~20 surplus robot tracks for 3 opponents on this data.

## 8. Metrics

`eval/metrics/evaluateMatch.m` (spec in `eval/SPEC.md`): per observer copy, per eval time, per target, one-to-one
(Hungarian, `matchpairs`) matching of live tracks to true targets of the same class with a 3 m gate; ball and the
three opponents are scored (teammates optional). Reported: availability, **inRegion95** (truth inside the track's
own 95 % χ²(2) region, 5.991), `inRegion95_capped` (also region area ≤ 3 m²: stops huge-covariance ghosts scoring),
within 0.25/0.5/1 m, RMSE, median error, median region area, NEES, false tracks (live tracks minus true targets).
Always report the capped and within-radius numbers beside inRegion95: ghosts and huge regions inflate the plain one.
`evaluateMatch` is slow on a whole match (~30 min); `tools/ballstudy/fast*.m` + `ballMetrics.m`/`oppMetrics.m`
give the same numbers on a 0.2 s grid for quick loops.

## 9. Results to date (`real01`, 730 s, mean over 3 robots; details: `tools/compare3/README.md`)

| Ball | NUbots | Own SRIF | Joint SRIF | Raw |
|---|---|---|---|---|
| Availability | 0.68 | 0.44 | 0.67 | 0.36 |
| in 95 % region, area ≤ 3 m² | 0.08 | 0.26 | **0.48** | 0.28 |
| within 0.5 m | 0.16 | 0.33 | **0.53** | 0.29 |
| RMSE (m) | 1.29 | 0.79 | 0.73 | 0.57 |

| Opponents | NUbots | Own SRIF | Joint SRIF | Raw |
|---|---|---|---|---|
| Availability | 0.81 | 0.31 | 0.54 | 0.16 |
| in 95 % region, area ≤ 3 m² | 0.18 | 0.15 | **0.34** | 0.11 |
| within 0.5 m | 0.44 | 0.19 | 0.39 | 0.13 |
| RMSE (m) | 0.89 | 0.76 | 0.62 | 0.48 |
| live tracks − true | +16.6 | −2.0 | −1.2 | −2.5 |

Reading it: joint is best on accuracy and calibration-aware coverage; NUbots' opponent availability comes from
~20 ghost tracks; raw has the lowest RMSE only because it answers on 16–36 % of samples. Joint loses in some 10 s
windows because of **data association and corroborated false positives**, not measurement calibration: two robots'
false-positive balls corroborate each other through `spawnPending` (23–28 duplicate ball tracks, t = 460–470 s);
two opponents within ~1.3 m swap identities (190–200, 430–440 s); a fast ball leaves tight tracks confidently
wrong (720–730 s). Cooperation triples the fused detections, so each association error is copied into every copy.

## 10. How we got here (decision log)

1. Built the eval pipeline (ground truth loader, sensing, extracted SRIF tracker, per-robot KF baseline, metrics, LaTeX/CSV export).
2. Found the metric gameable (nearest-track matching + ghost tracks) → one-to-one matching, false-track count, area cap.
3. Recorded 10 real matches from the sim; found heads fixed; built a camera visibility model validated against the sim detector.
4. Ran YOLO on re-rendered frames for imperfect detections; calibrated range/bearing noise (box bias, ground vs size range).
5. Ported NUbots logic; corrected it to `~/NUbots` main after finding the K1-branch proximity rule broken; implemented NUbots' own measurement model.
6. Diagnosed double ball tracks and huge ball covariance (spawn on any gate failure, no single-ball rule, no cap) → single-ball logic, pending spawn, analytic spawn covariance.
7. Fixed the first-fusion ellipse orientation (`spawnAnalyticBall`); tightened trust (`obsCovInflate` 16→2, heading 2°→0.5°, ball accel 2.5→2.0); robot accel stays 0.5 (lower hurt RMSE).
8. Added the raw-measurement behaviour; widened its freshness 0.25→0.40 s (frame gaps up to 0.37 s left holes).
9. Built the 4-row viewer and the three-way/four-way comparison with association and per-observer calibration diagnostics.

## 11. Viewer

```
cd /home/seb/soccer-sim/tools/viz
DISPLAY=:0 ~/miniconda3/bin/python viewer.py              # opens at t = 100 s; --t 179 to start elsewhere
~/miniconda3/bin/python viewer.py --t 179 --png out/f.png        # static PNG instead of a window
~/miniconda3/bin/python viewer.py --t0 100 --dur 60 --mp4 out/x.mp4   # clip (needs ffmpeg)
~/miniconda3/bin/python viewer.py --topdown                       # straight-down render instead of broadcast view
```

Needs `matplotlib` with TkAgg (`~/miniconda3/bin/python` has it) and the cached frames below.
Layout: columns = robots 1, 2, 3. Rows: **YOLO view** (head camera + boxes with class and confidence), **NUbots**
(faithful baseline, filled circles: ball 0.07 m, robot 0.5 m radius, no covariance), **Own SRIF, no sharing**
(`+` marks with 95 % outline ellipses), **Raw measurement** (small `x`). Right column: **broadcast view** (sim
render from a sideline camera, 42° elevation) above the tall **joint SRIF** minimap. Every minimap shows the
ground-truth ghost (soccer-ball glyph, top-down humanoid glyphs: blue teammates, red opponents) and two short FOV edge lines.
Keys: `Left/Right` ±1 s, `PageUp/PageDown` ±10 s, `Space` play/pause, `1/2/3` which robot's copy of the joint
filter is shown, `g` ghost on/off, `q` quit. Key frames are every whole second; track states are exported every 0.5 s.

Refreshing the data the viewer reads (all gitignored caches except `data/*.csv`):

```
# 1. tracks + ground truth for all rows (MATLAB, ~4 min; reflects current evalParams.m)
matlab -batch "cd('/home/seb/soccer-sim'); addpath(genpath(pwd)); cd tools/viz; export_tracks"
# rawOnly=true; before export_tracks recomputes only the raw rows
# 2. camera frames, bird's-eye and broadcast renders (MuJoCo EGL, ~2 s each)
MUJOCO_GL=egl tools/render/venv/bin/python tools/viz/render_keyframes.py 1.0
MUJOCO_GL=egl tools/render/venv/bin/python tools/viz/render_top.py
MUJOCO_GL=egl tools/render/venv/bin/python tools/viz/render_broadcast.py
```

The renderers print a harmless `EGLError` traceback at exit; check the files, not the output.
**If the viewer looks wrong, first check that `tracks.csv` is fresh**: a stale export once showed double ball
tracks that the current code no longer produces.

## 12. Running things

```
# evaluation (MATLAB R2026a; headless)
/usr/local/MATLAB/R2026a/bin/matlab -batch "cd('/home/seb/soccer-sim'); addpath(genpath(pwd)); runDemo"      # 2 x 60 s synthetic smoke test
# real01 with YOLO packets (slow: evaluateMatch is the bottleneck)
... runEvaluation(struct('gtFiles',{{'eval/gt/real01.csv'}}, 'packetSource','yolo', 'parallel',false, 'runName','real01_yolo', 'params',struct('maxAreaForScore',3)))
# four-way comparison + diagnostics
... run('tools/compare3/run_compare3.m')
# tests
... runtests('eval/tests')
```

Recording new matches (docker; needs the Booster sim image and `~/test3v3` agents): `tools/record/record_matches.sh`
(env `N`, `SPEED`, `DURATION`, `PFX`; `LOG_DET=1` also logs the sim's own detections); then
`python tools/replay_to_csv.py`, then `tools/yolo/detect_replay.py` (set `LD_LIBRARY_PATH` via `tools/yolo/ldpath.sh`).
Prerequisites that live outside the repo: `~/test3v3`, `~/.soccer-train/sim`, `~/NUbots_K1` and `~/NUbots` (reference code),
the sim XML via `$SOCCER_MODEL_XML`.

## 13. Gotchas and conventions

- **Never `pkill -f` to stop the viewer from a shell whose own command line contains the pattern** — it kills the shell.
  Use `ps -eo pid,comm,args | awk '$2=="python" && /viewer.py/ {print $1}' | xargs -r kill`.
- MATLAB runs are long; run them in the background and wait. `parfor` in `runEvaluation` can be disabled with `parallel=false`.
- Parameter lookup uses `evalParamOr` (flat or one sub-struct deep, with aliases); a misspelled parameter silently takes the default.
- Anything that changes `eval/config/evalParams.m`, `yoloErrorModel.m` or `detectionsToPackets.m` changes every result:
  re-run exports and comparisons afterwards, and report the file's mtime with the numbers.
- Keep tracker behaviours behind params with the original behaviour as default, so the original `run_5v5_*.m` scripts and tests stay valid.
- Ground truth and `TrueLabel` are for evaluation and diagnostics. Referee-style options (`ballResetTimes`, `robotResetEvents`) use GT
  jumps as a stand-in and must stay separate from pure-tracking results.
- Tuned values are calibrated on one match (`real01`). Check on another recorded match before trusting any threshold.

## 14. Open items

- Stronger corroboration before a ball spawn (two robots' false positives confirm each other); ambiguous association for opponents < 1.5 m apart.
- `obsCovInflateBall` > 1 (ball NEES above nominal because pose bias is treated as white); a bias state or effective-sample inflation would be the principled fix.
- State-dependent ball process noise (low while rolling, high around kicks) instead of one value.
- Game-controller logging (set plays, penalties) in new recordings to give a real restart signal.
- Ground-truth CSVs for `real02..10` are not in git; YOLO detections exist only for `real01`.
- Real-robot detector: noise and robot ranging here come from the sim renders; fit them on real K1 detections if available.
- Camera FOV and vision rate on the real K1 are unknown (the repo reads them from a shared-memory header at runtime).
