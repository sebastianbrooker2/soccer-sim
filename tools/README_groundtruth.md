# Recording ground truth from the sim

Output schema (see eval/SPEC.md): CSV `t,id,team,x,y,theta,z,roll,pitch,headYaw,headPitch`
(columns after theta are optional; old 6-column CSVs still load, with `gt.hasHead=false`); id 0 = ball, 1..3 team 1 (ours),
4..6 team 2; field frame x forward, y left, centre origin (= raw MuJoCo world = `/team1/*`).

## Option A: replay recorder (preferred)
Set these env vars for the sim process:

    MATCH_REPLAY_ENABLED=1
    MATCH_REPLAY_HZ=20                 # >= 20 recommended (sim GT runs at 200 Hz)
    MATCH_REPLAY_OUTPUT_DIR=/path/to/replays
    MATCH_ID=match01

NOTE: `/home/seb/test3v3/tools/training/run_match.sh` does NOT pass these through yet;
add them to its docker/env invocation (or export them and add `-e` / `--env` forwarding)
before relying on it. Output: `manifest.json` + `chunks/frames_*.jsonl`
(per frame: time, ball.position, robots[] named robot1..6 with basePosition,
baseQuaternion [w,x,y,z], gameControlPosition, gameControlYaw). robot1-3 = team1, robot4-6 = team2.

Convert:

    python3 tools/replay_to_csv.py /path/to/replays/match01 eval/gt/match01.csv --dt 0.05
    python3 tools/test_replay_to_csv.py     # unit test

## Option B: ROS2 topics
Ground-truth topics (geometry_msgs/Pose2D): `/team1/soccer/sim/ground_truth/ball` and
`/team1/robot{1..6}/soccer/sim/ground_truth/robot_pose`. Log with a small logger
(`ros2 topic echo` or rosbag2 -> CSV) into columns `t,id,x,y,theta`, then:

    python3 tools/replay_to_csv.py --ros2-csv log.csv eval/gt/match01.csv --dt 0.05

## Notes
- Sim matches are 10 min; GT published at 200 Hz; the converter resamples to a uniform grid
  (default 0.05 s) over the common time span, with t starting at 0.
- Load in MATLAB: `gt = loadGroundTruth('eval/gt/match01.csv', 0.05);`
- No sim handy: `gt = synthGroundTruth(600, 0.05, seed, 'out.csv');`

## Head / trunk columns
- `z` trunk (base) height in m (ball rows: ball z). `roll`,`pitch`: trunk tilt in rad from baseQuaternion
  (ZYX Euler, body->world; +pitch = nose down). `headYaw`,`headPitch`: joints `AAHead_yaw`, `Head_pitch`
  from `robots[].joints` (rad; 0 for ball). `theta` is trunk yaw, so camera heading = theta + headYaw.
- MATLAB: `gt.z, gt.roll, gt.pitch, gt.headYaw, gt.headPitch` (Nt x6), `gt.hasHead`.
  Sanity check: `checkGroundTruthHead('eval/gt/real01.csv')`.
- K1_22dof.xml: `AAHead_yaw` axis +z, range [-1,1] (+ = turn left, CCW from above); `Head_pitch` axis +y,
  range [-0.349, 0.855] (+ = look down). Chain: trunk -> Head_1 at (0.0056,0,0.2149) [yaw joint]
  -> Head_2 at +(0,0,0.033) [pitch joint] -> camera `rgbd_camera` pos (0.0613,0,0.108) in Head_2,
  euler "0 -1.5708 -1.5708" (default xyz intrinsic) => optical axis = Head_2 +x (forward), image right = -y,
  image up = +z; fovy 58 deg, 320x240 (hfov ~ 74.6 deg).
  Camera world pose: T_trunk * Rz(headYaw) * [0.0056,0,0.2149+0.033] * Ry(headPitch) * [0.0613,0,0.108].
- Old (6-column) versions of the real01..10 CSVs are kept in eval/gt/old/.
