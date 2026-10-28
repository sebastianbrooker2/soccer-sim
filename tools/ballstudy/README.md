# Ball / teleport study (real01)

Everything is param-gated in `params.tracker` (defaults in `trackerParamsFill` = OFF = old behaviour).
`eval/config/evalParams.m` now switches on the recommended set (see bottom).

## Current logic: how a second ball track is born and why it survives
1. `processPacket` predicts all tracks, gates each ball detection on `D2S` (full innovation covariance,
   3-sigma, 3 dof) against every live ball track; greedy one-to-one assignment.
2. A detection that fails every gate SPAWNS a new ball track. Nothing checks that a live ball track exists,
   so a far/low-confidence detection (range std 1-2 m, range 3-5 m off) fails the 3-sigma gate of the
   good track (std 0.1 m) and becomes a second track.
3. Deletion: only `maxMisses` (5) scans in which the track was "expected visible" but unfused. A ghost whose
   predicted position is outside every scan never accumulates misses. `lostStd`/`lostTime` applied to ROBOTS only.
4. `mergeDuplicates` needs both std <= `mergeMaxStd` (0.6 m) -> vague ghosts are never merged; `distinct`
   (cannot-link) also blocks merges.
5. Ball process model (kappa 0.2, accel std 2.5) lets covariance explode; a vague track then passes every gate
   and is re-fused with junk. Measured on real01 (default tracker): 148-156 ball tracks per copy, live ball
   tracks 0/1/2/3+ = 15/11/26/48 % of time (copy 1), mean 2.8, max 11; median std of live ball tracks 6.7-33 m,
   p90 > 88 m (stale >3 s tracks: median 48-69 m).
t=199 case (copy 1): trk23 (spawned 180.85) is good (std 0.1). robot3 ball detections at 188.02 (range 9.35, std 2.05),
195.04 (8.15, 1.16, conf 0.64, 3.8 m off), 195.35, 198.06 (range 6.28, std 0.92, 5.1 m off), 198.22 failed the gate
-> spawned trk28 (187.56), trk31 (198.06; 2 fusions, std 3.0 then 0.7, deleted 199.58 by 'missed'); copy 2 trk30 (195.04),
copy 3 trk27. Truth ball (5.9,2.1) while ghosts sat at (0.9..3.3, 0.5..1.8). Scripts: `seq199.m`, `diagnose.m`.

## Ground-truth events (real01, `gtEvents.m`, `events_real01.csv`, `loadEvents.m`)
* Robot teleports: 10 (ids 1,2,4,5,6; five at t=135.55 kickoff after goal; 405.2 r2, 491.6 r2, 520.35 r4, 623.85 r1, 682.1 r2;
  all coincide with `active=false` in the replay jsonl) -> sent to start spots (+-4..6, +-5).
* Ball: NO hard discontinuity (max speed 6.2 m/s; the ball is dragged/rolled back at ~2.5 m/s after dwelling in the goal
  net or on the line). 17 out-of-play stretches >= 1.5 s (goal-net dwell up to 25 s), exports as `exit`/`restart` rows.
  Real01 therefore has robot teleports but not ball teleports; ball "reset" benefit cannot be measured on true jumps.
  `synthTeleportGT.m` scripts real ball/opponent teleports (only a short synthetic check was run, see below).

## Options (all in `eval/tracker/processPacket.m`)
A `ballSingle`; B `ballLostStd`/`ballLostTime`; C `ballResetN` (+`ballResetD2`, `ballResetVmax`); spawn policy
`spawnMaxRangeStd`/`spawnMaxArea`, `spawnPending`, `spawnPendWindow`, `spawnPolicyRobots`, `ballReplaceStd`;
EXTERNAL info: D `ballResetTimes`, robots `robotResetEvents` [t x y] with `robotResetGate` (1.5 m or 95% region; DeleteReason 'event').
Pending hypotheses are stored as dead track records (`DeleteReason='pending'`), invisible to metrics.

## real01 results (same packets, 3 copies, 0.2 s grid; `study.m`, tables in `study_real01_*.csv`)
b_* ball, o_* opponent. avail/in95 are all-sample rates; excess = mean(live ball tracks - 1).
| variant | b_avail | b_in95 | b_in95cond | b_rmse | b_med | excess | b_area | o_in95 | o_rmse |
|---|---|---|---|---|---|---|---|---|---|
| base (old default) | 0.77 | 0.75 | 0.97 | 0.87 | 0.21 | 1.24 | 2.22 | 0.52 | 0.61 |
| A single | 0.74 | 0.71 | 0.95 | 0.90 | 0.21 | 0 | 2.11 | 0.52 | 0.61 |
| A+B(std3)+C | 0.63 | 0.60 | 0.95 | 0.53 | 0.18 | 0 | 1.78 | 0.52 | 0.61 |
| spawn+A+C+replace (no B) | 0.73 | 0.70 | 0.95 | 0.90 | 0.21 | 0 | 2.07 | 0.52 | 0.61 |
| FULL20 (B std 20) | 0.68 | 0.65 | 0.95 | 0.67 | 0.20 | 0 | 1.88 | 0.52 | 0.61 |
| **FULL8 (recommended)** | 0.67 | 0.63 | 0.95 | 0.63 | 0.19 | 0 | 1.80 | 0.52 | 0.61 |
| FULL(3)+D (ball events)+R | 0.57 | 0.54 | 0.94 | 0.57 | 0.18 | 0 | 1.66 | 0.51 | 0.61 |
| FULL(3)+R (robot events only) | 0.60 | 0.57 | 0.94 | 0.57 | 0.18 | 0 | 1.74 | 0.51 | 0.61 |
| baseline faithful | 0.68 | 0.18 | 0.26 | 1.29 | 0.86 | 0 | 0.50 | 0.77 | 0.89 |
| baseline tuned | 0.52 | 0.40 | 0.76 | 0.88 | 0.31 | 0 | 0.94 | 0.28 | 0.68 |
Caveat: evaluation matches each target to the nearest track (oracle, 3 m gate), so the old tracker's 2.2 live ball tracks
act as free extra hypotheses and inflate its availability/in95; with one track those numbers fall by construction while
rmse, median error and region area improve. The cost of B is real availability (long unseen ball -> no track).
* Earlier single-option runs (tracker identical but sensing before recalibration, same trend): C alone (N=2/3) changes little
  (the gate rarely fails twice in a row on the same track); with D2>40 / v>12 immediate triggers: excess 1.4 -> 0.2;
  B std1.5/3/time3: rmse 0.70 -> 0.52/0.55/0.61 but availability 0.73 -> 0.58/0.63/0.70.
* Forced stops of robot tracks (R): opponent in95 0.52 -> 0.51, rmse unchanged, opponent reacquire time unmeasurable
  (opponents are rarely seen: availability 0.53; 30 s cap). Not worth enabling; only 3 of the 10 teleports involve opponents.
* D (ball events, external) hurt reacquire (0.15 -> 2.25 s median, fail 19 -> 33): the ball sits in the net/on the line
  and is re-observed anyway; ending the track at 'exit' discards good info. Not recommended on real01.
* Synthetic (synthTeleportGT, 300 s, 7 ball + 3 robot teleports): base ball excess only 0.07, reacquire 0 s; D cuts excess to 0.02;
  B costs availability 0.93 -> 0.68 (std3). Teleports are not a problem for the joint tracker when the ball is seen often.
* Process noise (synth): kappaBall 0.5 / accel 1.5 slightly better rmse (0.71/0.73 vs 0.78); accel 4 worse. Not changed.

## Fairness: baselines at ball jumps
* faithful (NUbots): one UKF never deleted, nearest detection ALWAYS fused (accept is always true): after a jump it drags toward
  the new ball at the rate of its tiny Q (0.01*dt, R=0.1) -> very slow; `ballMaxRej` reset only zeroes a counter; relies on the
  team-ball guess. Result: in95 0.18, rmse 1.29. No edits made to eval/baseline.
* tuned: single track re-initialised after `ballReacquire`=3 consecutive gated-out detections (= option C), aged out after 3 s.

## Recommended defaults (now in evalParams.m)
Pure tracking (no external info): ballSingle=true, ballLostStd=8, ballReplaceStd=8, ballResetN=3,
spawnMaxRangeStd=0.6 + spawnPending=true. External-info options (ballResetTimes, robotResetEvents) stay OFF.
Re-run: `STUDY_NAME=real01 matlab -batch "run('tools/ballstudy/study.m')"` (STUDY_ONLY, STUDY_TAG env filters).
