# Multi-robot bearing-only tracking: how it works (technical notes for report writing)

Everything below is read from the MATLAB code in this repo (`soccer-sim`). File names are given so claims can be checked. Where something is a simplification or a quirk, it is flagged under "Limitations / things to be honest about".

## 1. Overview

- Five stationary robots on our half of a soccer field (x < 0) each carry a camera with a limited field of view (FOV) that pans left and right.
- They track the ball (moving) and ten other robots (five opponents who random-walk, plus the five teammates, who are stationary).
- Each robot reports only a **bearing** (a direction), never a range. Position is recovered by **fusing bearings from several robots** in a Bayesian filter.
- The estimator is a Gaussian filter in **square-root information form**. The nonlinear measurement update is a MAP optimisation (BFGS trust region).
- Two scripts:
  - `run_5v5_stationary_robots.m`: target identity is given (`TargetName` is set when each measurement is built).
  - `run_5v5_data_association.m`: detections are unlabelled and a nearest-neighbour gated data-association step decides which track each belongs to.

## 2. Tracked state and process model (`@SystemBall`)

- State `x = [px; py; vx; vy]` in the **field frame** (metres, m/s).
- Process model is constant velocity: `f(x) = [vx; vy; 0; 0]` (`dynamics.m`).
- Process noise acts on the velocity states only: `SQ = diag([0.1, 0.1])` scaled by `sqrt(dt)` (`processNoise.m`). This is continuous-time white acceleration noise.
- Default prior: mean 0, sqrt-covariance `diag([5, 5, 2, 2])/3`.
- **Each tracked object has its own independent `SystemBall` filter** (ball, `opp1..5`, `team1..5`). There is no joint state, because nothing couples their dynamics or measurements, so a joint covariance would be full of zero cross-blocks.
- The robots themselves are **not** part of the filter state. They are stationary with a known position and a small covariance.
- The `SystemBall` object has two states: `x_sim` (ground truth, used only for simulation and the visibility check) and `density` (the estimate, a `GaussianInfo`).

## 3. The measurement (`@MeasurementBallBearing`)

### 3.1 What a measurement is

A bearing measurement is a **2x1 unit vector in the field frame**, not a scalar angle:

```
d = p_ball - p_obs        (p_ball = x(1:2), p_obs = ObserverPosition)
r = ||d||
u = d / r                 (unit line-of-sight vector, observer -> ball)
y = u + noise
```

Reasons for a unit vector:
- It matches what a real detector reports (a direction from pixels).
- It has no angle wrap-around or branch cut, unlike `atan2`.
- It is **not** relative to the robot's heading. The observer's heading, FOV and range are used only to decide whether the target was visible at all (section 4).

### 3.2 What one measurement object carries

One `MeasurementBallBearing` object is created per robot, per detection:

| Field | Meaning |
|---|---|
| `time` | timestamp |
| `y` | measured unit vector (field frame) |
| `ObserverPosition` (2x1) | believed robot position in the field frame. This is effectively the "transform": the model only needs it to form `d = x(1:2) - ObserverPosition` |
| `ObserverCovariance` (2x2) | uncertainty in the robot's own position |
| `ObserverHeadingVariance` | uncertainty in the robot's heading (0 in the scripts) |
| `DetectionNoiseStd` | per-robot detector tangential noise, in radians |
| `RobotID`, `TargetName` | bookkeeping only; the math never reads them |
| `TrueObserver*`, `ObserverFOV`, `ObserverRange` | ground truth used only for the visibility gate |

There is no separate rotation or transform to apply because the bearing is already expressed in the field frame. The observer's pose enters through `ObserverPosition`, plus its covariance in the noise model.

### 3.3 Measurement function (`predict.m`)

```
h(x) = u = (x(1:2) - p_obs) / ||x(1:2) - p_obs||
dh/dx = [ (I - u u^T)/r ,  0_{2x2} ]        (2x4)
```

- The Jacobian shows only **position** is observed (the velocity columns are zero).
- `(I - u u^T)` is the projector onto the direction **perpendicular** to the line of sight. A small sideways move of the ball changes the bearing by about (sideways distance)/r. A move along the line of sight changes nothing.
- Second derivatives (`d2hdx2`) are also provided, so a full Newton method can be used.

### 3.4 Noise model (`noiseDensity.m`)

Let `t = [-u2; u1]` be the unit tangent perpendicular to `u`.

```
sigma_t^2 = DetectionNoiseStd^2
          + ObserverHeadingVariance
          + (t^T * ObserverCovariance * t) / r^2

R = sigma_t^2 * t t^T  +  assumedDepthStd^2 * u u^T        (assumedDepthStd = 1)
```

Physical reasoning:
- **Detector noise** is a fixed angular error. For small angles it equals the tangential displacement of the unit vector.
- **Heading error** rotates the bearing by the same angle at any range, so it is not divided by range.
- **Observer position error** shifts the line of sight sideways by an absolute distance, which is a bearing error of (distance)/r. So it matters up close and is negligible far away, hence the `/r^2` on the variance.
- Only the **tangential** component of the observer covariance matters, because moving along the line of sight does not change the bearing.
- The `u u^T` term exists because a unit vector has one true degree of freedom. The tangential-only noise would be rank 1 (singular). The depth term makes `R` invertible and is described as a rough "assumed depth uncertainty" of 1.
- `R` is linearised about the **current prior mean** of the target (`system.density.mean()`), so it depends on the filter's own estimate.

`R` is converted to `GaussianInfo.fromMoment(R)`. The code then takes its square-root information matrix `Xi_v`.

## 4. Detection model: when does a robot report a bearing? (`computeBallDetection.m`)

The target is detected only if all of the following hold, evaluated with the **true** robot pose and **true** target position. This is deliberate: what a camera can physically see depends on where it actually is, not on what it believes.

1. The target is within `Range`.
2. The target's bearing is within `FOV/2` of the look direction. The look direction is `Heading + GazeOffset`, where `GazeOffset = GazeAmplitude * sin(GazePhase0 + GazeAngularSpeed * t)`. This is the head-pan sweep, 80 degrees amplitude with a 6 s period. Each robot has a different phase.
3. A simulated YOLO-style confidence is at least the cutoff:
   ```
   conf = 0.99 - 0.35*(d/Range) - 0.35*(|angleErr|/(FOV/2)) + 0.02*randn,  clipped to [0, 0.99]
   ```
   Cutoff is **0.20** (changed from 0.60 in the current uncommitted edits).

If not detected, nothing is fused and the density is left as the time update produced it. Robots measure at their own fixed `Rate` (2 to 5 Hz).

## 5. How bearings from different robots are fused

### 5.1 Fusion is a product of likelihoods in one filter

For one target there is **one `SystemBall` filter**. All robots' measurements of that target are put in **one time-sorted queue** and processed in order. Each `Event.process`:

1. `system.predict(t)` propagates the density to the measurement time (constant-velocity prediction plus process noise).
2. `update` fuses the measurement.

So Bayes' rule is applied repeatedly:

```
p(x | y_1..y_k) ∝ p(x | y_1..y_{k-1}) * N(y_k ; h_k(x), R_k)
```

Each robot's bearing is just another factor. **There is no explicit "sharing" step.** Information from robot A at t = 0.5 and robot B at t = 0.6 ends up in the same density because they update the same filter. This is equivalent to a **centralised fusion centre receiving every detection with zero latency, no dropout and no bandwidth limit**.

### 5.2 Why several robots localise the target

- One bearing constrains the target tightly **across** the line of sight and only loosely **along** it. The loose direction is bounded only by the prior, the process model and the assumed depth.
- A second robot at a different angle has a different tangential direction. The two constraints intersect, which is **triangulation**.
- The result is an information-form sum. Each measurement adds roughly `J^T R^-1 J` to the information matrix, with `J = dh/dx`. A sideways-constraining rank-1 term from each viewpoint accumulates into a full-rank position constraint.
- Bearing accuracy converts to position error as `r * sigma_angle`, so far-away robots contribute less. The `/r` in the Jacobian captures this.

### 5.3 The update itself (`@Measurement/update.m`, `@MeasurementGaussianLikelihood`)

The measurement likelihood is Gaussian in `y` with a mean that is nonlinear in `x`:

```
p(y | x) = N( y ; h(x), R )
```

Implemented in square-root information form: the predicted density is `GaussianInfo.fromSqrtInfo(Xi_v * h(x), Xi_v)`, where `Xi_v` is the sqrt-info matrix of `R`.

The update is a MAP estimate:

```
x* = argmin_x  [ -log p(x) - log p(y | x) ]
```

- Solved with `funcmin.BFGSTrustSqrt`, a BFGS quasi-Newton method with a trust region that works directly on a **square-root Hessian** (`Xi`).
- The initial point is the prior mean and the initial sqrt-Hessian is the prior sqrt-info matrix.
- The posterior is `GaussianInfo.fromSqrtInfo(Xi*x*, Xi)`: mean at the MAP, with covariance from the optimiser's final Hessian approximation. This is a Laplace-style approximation.
- Gradient and Hessian of the log-likelihood come from the chain rule through `h`: `dldx = -dhdx' * dldy` and `d2ldx2 = dhdx' * d2ldy2 * dhdx - sum(dldy .* d2hdx2)`.
- Other options exist in the code: `'affine'` (linearise and condition, an EKF-like update), `SR1TrustEig`, `NewtonTrustEig`, `BFGSLMSqrt`. The scripts use `BFGSTrustSqrt`. The comment says `affine` and `NewtonTrustEig` were not used for reasons recorded in `run_track_ball_stationary_robots.m`.
- If the optimiser fails to converge, `MeasurementBallBearing/update.m` catches the error, issues a warning and **skips that fusion** (the density stays at the prediction) instead of crashing.

## 6. Teammates as targets

Robot i also reports bearings to teammate j (i != j) with the same detector, and these update a `SystemBall` filter for j. Robot j's true position is already known exactly, so this is redundant now. It is the groundwork for **cooperative localisation**, where a robot's teammates refine its position estimate. In the current code the observer covariance enters each measurement's noise, but it is **not fed back** into the robot's own estimate.

## 7. Data association (`run_5v5_data_association.m`)

Targets are unlabelled. Detections are built (using true geometry for visibility and noisy `y` generation) into one global chronological queue of raw detections, then each is processed online by `associateAndUpdate`:

1. Predict every live track to the detection time.
2. For each track, build the predicted bearing distribution using `MeasurementBallBearing`'s own `predictDensity` (so it uses the same `h` and `R` linearised at the track mean).
3. **Gate**: keep tracks where `py.isWithinConfidenceRegion(y, 3)`, a 3-sigma region.
4. **Rank** gated tracks by the squared whitened residual (Mahalanobis distance), `w = Xi*y - nu`, `d2 = w'w`. The smallest wins (nearest neighbour).
5. **Fuse** into the winner using the same bearing update.
6. If **nothing gates**, spawn a new track. Its position is the observer position plus 1 m along the bearing, velocity 0, and prior sqrt-covariance `diag([1.5, 1.5, 2.5, 2.5])`.

Notes:
- No track deletion, so every track lives to the end.
- Mis-association and fragmentation are possible and expected. The ground-truth `TrueLabel` is attached to each track only for plotting and evaluation, and the associator never reads it.
- In this script `AlreadyDetected = true` skips the visibility re-check inside `update.m`, because a re-check would redraw the random confidence jitter and could flip the earlier decision.
- This is global nearest neighbour in its simplest form (greedy per detection), not JPDA or MHT.

## 8. Simulation setup

- Ball: scripted kicks at t = 1.5, 3.5, 6.5, 10 s (instantaneous velocity changes added to `x_sim`). T_total = 15 s.
- Opponents: smooth random walk (`createPlayer` / `stepPlayerWalk`) on a 0.02 s grid, with true state written into `x_sim` at each event.
- Teammates: stationary.
- Random seed fixed (`rng(42)`).
- Scale: 5 robots x (1 ball + 5 opponents + 4 teammates) = 50 (robot, target) pairs, so several thousand events.
- Robots are all stationary with FOV 100 to 140 degrees, ranges 10 to 15 m (the data-association script scales range by 1.8), rates 2 to 5 Hz, and detection noise 0.4 to 1.5 degrees.
- Tooling: `stepThroughMultiTargetTracking.m` (interactive event-by-event viewer showing which robot each measurement came from), `plotStatesGrid`, `plotSigma`, `plotFieldTrajectory`. The script runs the unit tests (`runtests('tests')`) before each run.

## 9. Limitations / things to be honest about in the report

1. **Communication is idealised.** Fusion is centralised, with perfect, instantaneous, lossless sharing. There are no messages, delays, drops or bandwidth limits. The measurement object is handed to the filter directly. A real system would send a compact packet (robot ID, timestamp, observer pose and covariance, unit vector, sigma) and rebuild `h` and `R` on the receiver.
2. **No decentralised fusion yet.** There is no per-robot estimate, covariance intersection, or information-filter message passing. The information form makes that a natural extension (information from independent measurements is additive).
3. **The "1 m depth std" is an unusual device.** It is added as variance `1^2` along `u` on a **unit vector** in `R`. The unit vector is dimensionless, so that is really a radial noise of std 1 on a quantity of magnitude 1, not 1 metre. Also, `dh/dx` has no component along `u`, so the radial part of `y` carries essentially no information about `x` whatever its noise. Its real role is to keep `R` full rank. The "depth cue" description in the code comments is more of an interpretation than what the math does. In the data-association script `simulateBearingDetection` also adds `u * 1 * randn` to the simulated `y`, so the simulated measurement is not actually a unit vector. A report should not claim the system uses a genuine range cue.
4. **Observer self-localisation is not coupled.** Observer position error is treated as noise on each measurement, but the observer's own estimate is not updated, and errors from the same observer's position error across successive measurements are treated as independent although they are in fact correlated. This is an optimistic assumption.
5. **Known target identity** in `run_5v5_stationary_robots.m`, and a simple greedy nearest-neighbour association in the other script.
6. **Visibility uses ground truth.** The FOV and confidence gate uses the true target position and true robot pose, so detection is simulated, not computed from images. The YOLO-style confidence model is a hand-made heuristic.
7. **Process model is crude.** The constant-velocity model is overridden with ground truth for opponents (`x_sim` only; the estimate is not touched) and the ball's kicks are unmodelled by the filter and appear only as process noise.
8. **Robots are stationary.** No robot motion or SLAM.
9. **Detector cutoff changed from 0.60 to 0.20** in uncommitted changes (`computeBallDetection.m`). This is global, so it affects every script that uses the detector.
10. **Linearisation point.** `R` is linearised at the prior mean, so a poor prior gives a poor noise model. Optimiser non-convergence is handled by skipping the update with a warning, which can silently reduce information.

## 10. Quick glossary

- **Bearing-only**: measurement gives direction, not distance.
- **Information form**: the Gaussian is stored as `(Xi*mu, Xi)` where `Xi` is the square root of the inverse covariance (`Xi'Xi = P^-1`). Independent measurement information adds, and square-root form improves numerical conditioning.
- **MAP**: maximum a posteriori estimate, the mode of prior times likelihood.
- **Tangential/radial**: perpendicular to / along the line of sight.
- **Data association**: deciding which track (or a new one) a detection belongs to.
- **Gating**: only considering tracks whose predicted measurement distribution contains the detection within a confidence region.
- **Mahalanobis distance**: distance normalised by the predicted uncertainty.

---

# Update: decentralised, range-bearing version (`run_5v5_decentralised.m`)

This supersedes the "centralised / bearing-only / constant-velocity ball / no deletion" description above for the new script. The earlier scripts are unchanged and still behave as described in sections 1 to 10. **None of this has been run** (no MATLAB where it was written), so treat results and parameter values as untested.

## Changes versus the report-vs-code gaps

1. **Decentralised.** Each of the 5 robots has its own copy of every track and its own data association. Each robot sends one packet per scan containing its pose and covariance, FOV, range, look direction and its detections (possibly none). Each packet reaches each other robot independently with probability `1 - linkDropProb` (0.2 by default); a robot always receives its own. Copies can therefore differ. Latency and out-of-order arrival are not modelled.
2. **Range is a measurement.** `MeasurementBallBearing` has a new `RangeNoiseStd` property. When it is finite, `y = [unit bearing (2x1); range]` (3x1) and `h(x) = [u; r]`, with Jacobian rows `(I-uu')/r` and `u'`, and range Hessian `(I-uu')/r`. When it is `NaN` (the default) the old bearing-only behaviour with the 1 unit depth term is unchanged.
   - Noise is `R = blkdiag(sigma_t^2 * I2, sigma_r^2)`. The bearing block is isotropic because the radial part of a unit vector carries no information about the target, so no invented depth term is needed. `sigma_r^2 = RangeNoiseStd^2 + u'*ObserverCovariance*u`.
   - Bearing and range noise are treated as uncorrelated (the shared observer-position error that would correlate them is ignored).
   - Per-robot fixed range std: 0.2 to 0.5 m.
3. **Ball process model.** `SystemBall` has new `Kappa` (default 0, constant velocity) and `AccelNoiseStd` (default 0.1) properties. For the ball, `dynamics` gives `dv/dt = -kappa*v` (exponential rolling-friction decay, `kappaBall = 0.2` 1/s) and `ballAccelStd = 0.5`. Robots keep `Kappa = 0`, std 0.1. Ground truth for the ball is integrated with the same decay. The detector reports a class (`ball` or `robot`) but not an identity, so a new track takes its process model from its first detection's class, and detections only gate against same-class tracks.
4. **Deletion only when expected but unseen.** After a copy fuses a packet, each live track that was not fused counts a miss only if its estimated position was inside that scan's FOV and range (within `expectedMargin = 0.8` of the half-angle and range, and not within 0.5 m of the sender). A fused detection resets the count, and `maxMisses = 5` consecutive misses delete the track. Out-of-view tracks are never penalised.
5. **Spawn.** A new track is placed at the measured range along the measured bearing, with covariance implied by the measurement (tangential `r*sigma_t`, radial `sigma_r`) and velocity std 2.5.

## Still not modelled (be explicit in the report)

- Robots are stationary with a known field-frame position. There is no world-frame vs field-frame transform or localisation, only `ObserverPosition`/`ObserverCovariance` given in the field frame.
- Packet latency and reordering. Only loss.
- False-positive detections, so deletion mostly removes fragmented or duplicate tracks.
- Unseen-ness is judged from the sender's FOV/range geometry only (no occlusion), using the track's estimated mean rather than its full uncertainty.
- Parameters (`ballAccelStd`, `kappaBall`, `maxMisses`, `expectedMargin`, `linkDropProb`) are first guesses, not tuned.
- The cost is roughly 5x the earlier data-association script (five copies).

## Update 2: association fixes after the first run

The first run of `run_5v5_decentralised.m` spawned 41 to 59 tracks per copy for 11 real objects (24 to 44 deleted). Cause: the gate used `predictDensity`, which is `N(h(mu), R)` only, so it ignored the track's own uncertainty and rejected targets that had drifted between scans, spawning duplicates. Changes:

- **Gate on the full innovation covariance** `S = H*P*H' + R` (chi-square threshold, 3 dof).
- **One detection per track per scan** (`oneToOne`, greedy global nearest neighbour).
- **Duplicate merge** (`mergeSigma`): same-class live tracks whose position estimates agree within the gate (sum of covariances) are merged by dropping the one with fewer fused detections.
- **Diagnostics** printed at the end of the run: per-copy fusion, spawn (first / respawn / duplicate), deletion and merge counts; d2 percentiles for correct and wrong associations (should be near chi-square(3): 2.37 / 6.25 / 11.34); spawns per real object; live tracks; and an event log. Ground-truth labels are used only for these diagnostics, never by the associator.

## Update 3: fixes after the second run

Second run: the gate was well calibrated (correct associations had d2S percentiles [2.40 6.73 11.73] against chi-square(3) [2.37 6.25 11.34], versus [3.03 9.23 19.14] for R only, which confirmed the earlier diagnosis). Remaining problems and fixes:

- **Bad merges (89 of 164)** came from opponents walking past each other. Fix: *cannot-link*. Any two tracks that took different detections from the same scan are different objects (one object gives one detection per scan) and are never merged.
- **Spawned tracks were never given a first update**, so they had empty histories (this crashed `plotSigma`) and the first measurement was baked into the prior. Fix: spawn with a vague prior (`spawnPosStd = 3` m) and fuse the detection into it, so nothing is counted twice.
- **Duplicate spawns at manoeuvres** (ball after kicks, opponents turning) were outside the gate because process noise was too low. `ballAccelStd` 0.5 to 2.5, `robotAccelStd` 0.1 to 0.5 (opponents turn up to 45 deg/s at about 0.8 m/s).
- **"wrongLabel" overstates errors.** Two anonymous constant-velocity tracks that cross legitimately swap identity, and `TrueLabel` is fixed at spawn.
- **Ghost tracks remain by design.** A track whose target manoeuvred away is not deleted while its predicted position is out of everyone's view (e.g. one opp1 track ended 5.7 m off).

## Update 4: after the third run

Third run: ball and opponents good (1 to 3 spawns each; gate calibrated, correct-association d2S percentiles [2.22 6.10 11.05]). Teammates broken: in robot 1's copy the track for team1 (robot 1's own position) swallowed team4's detections, and the real team1 respawned about 37 times, giving 202 bad merges out of 232. Hypothesis (to be confirmed by the new diagnostics): a robot-class track sitting on the sender's own position gives range about 0, so `u = d/r` and `H = (I-uu')/r` blow up and the gate accepts anything.

Changes: robot-class tracks within `selfExclusion = 1 m` of the sender are excluded from association (and from the deletion check); merging now requires both tracks' position std to be below `mergeMaxStd = 1 m` (two vague tracks always "agree"); failed (non-converged) fusions are counted and logged as `FUSION FAILED`; spawn events log the new track's std and error after its first fusion, and merges log the metres apart.

## Update 5: after the fourth run

Fourth run: 13 to 17 tracks spawned per copy for 11 real objects (first spawns 11 per copy, 2 to 6 duplicates), 0 failed fusions, 12 merges of which 1 was bad, correct-association d2S percentiles [2.12 5.95 10.92] against chi-square(3) [2.37 6.25 11.34]. Teammates fixed, confirming the self-exclusion fix. Position errors 0.05 to 0.46 m for all 11 objects in all 5 copies.

Remaining, in order of importance:
- **New tracks reported std = 3.00 m after their first fusion.** The prior mean was placed exactly on the measurement, so both gradients were zero, the optimiser took no step and returned the prior's sqrt-Hessian as the posterior. Fixed by starting the prior 1 m short of the measured range (bias about 0.03 m). Check the next run's "New tracks after their first fusion" line: std should now be a few tenths of a metre.
- **Identity swaps when opponents cross** (opp2/opp3/opp4/opp5; "wrongLabel" up to 125 in one copy). Inherent to anonymous constant-velocity tracking with no appearance cue; tracks keep following *an* opponent. `TrueLabel` is fixed at spawn, so the count overstates real tracking error.
- **A few duplicate spawns at manoeuvres** (opponent turns, ball kicks), cleaned up by merging.
