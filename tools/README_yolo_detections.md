# Logging camera detections for tracking evaluation

Goal: CSV `t,robot,class,u_px,v_px,w_px,h_px,conf` (+ per-run camera struct and
robot poses) -> `eval/sensing/detectionsToPackets.m` -> tracker packets.

## What the simulator publishes (verified from source, not run)

Source: `~/.soccer-train/sim/extensions/detection_extension.py` (a copy of what the
Studio sim container runs). IMPORTANT: this is NOT a neural YOLO. It is a geometric
projection of MuJoCo ground truth through the head camera (no image rendered), so it
has no false positives/negatives except FOV/depth/50 %-visibility, conf is always 1.0,
and it cannot be used as an independent "ground truth" of robots (see below).

- Topic: `[/robotN]/soccer/sim/vision/detections` (prefix `/robotN` in multi-robot mode,
  none in single mode), type `vision_msgs/msg/Detection2DArray`, default reliable QoS depth 10.
  Also `.../soccer/sim/vision/field_lines` (`geometry_msgs/PoseArray`).
- header.stamp = sim time; header.frame_id = `<prefix>/odom` (misleading).
- Per `Detection2D`: `bbox.center.position.{x,y}` = bbox centre in pixels (u right, v down),
  `bbox.size_{x,y}` = width/height px, `results[0].hypothesis.class_id` in
  {Ball, Goalpost, L_cross, T_cross, X_cross, Penalty_point}, `.score` (=1.0),
  `results[0].pose.pose.position.{x,y}` = target position in the robot TRUNK frame (m,
  x fwd, y left; z forced 0; trunk frame includes tilt, not yaw-only).
- Rate: `detection_fps` param (default and max 30), cameras served round-robin by a process
  pool, so per-robot rate with 6 robots is lower than 30 Hz (measure it from the log).
- Robots/opponents are NOT detected in sim (no Person/Opponent emitted; the class ids exist
  only in the YOLO training-export mapping). No team labels anywhere.
- Camera K1 (`~/.soccer-train/sim/mjcf/K1_22dof.xml`): 320x240, fovy 58 deg, ideal pinhole,
  fx=fy=216.5 px, (cx,cy)=(160,120), no distortion; HFOV 73.7 deg; mounted on `Head_2`
  at (0.0613,0,0.108) in Head_2, Head_2 at (0.0056,0,0.2479) above the trunk origin, so
  head yaw/pitch (`AAHead_yaw`, `Head_pitch` from LowState) move it. Trunk-origin height when
  walking is not in the MJCF (spawn 0.7 m): measure it (below).
- `--no-camera-relay` only disables the browser camera relay; detection reads physics state
  directly and `rgb_publisher_extension` still publishes `<prefix>/<cam>/rgb/image_*` and
  `rgb/camera_info` (K, width, height). No GPU rendering is needed for detections.
- Real robot: Studio's template shows `/booster_vision/detection` with `msg.detected_objects[]`
  having `xmin,ymin,xmax,ymax,label` (the only definition found on this machine; message package
  and any confidence/position fields are UNVERIFIED).

## Logging (inside the ROS 2 Humble container / any rclpy env on the same DDS domain)

```python
import csv, rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
R = [1, 2, 3]            # robots to log
class L(Node):
    def __init__(s):
        super().__init__('det_logger'); s.f = open('detections.csv', 'w', newline='')
        s.w = csv.writer(s.f); s.w.writerow('t,robot,class,u_px,v_px,w_px,h_px,conf'.split(','))
        s.w2 = csv.writer(open('trunk_rel.csv', 'w', newline=''))   # optional sim-only range/bearing check
        for r in R:
            s.create_subscription(Detection2DArray, f'/robot{r}/soccer/sim/vision/detections',
                                  lambda m, r=r: s.cb(r, m), 10)
    def cb(s, r, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        ball = [d for d in m.detections if d.results and d.results[0].hypothesis.class_id == 'Ball']
        if not ball: s.w.writerow([t, r, 'none', 0, 0, 0, 0, 1])      # empty frame marker
        for d in ball:
            h = d.results[0].hypothesis
            s.w.writerow([t, r, 'ball', d.bbox.center.position.x, d.bbox.center.position.y,
                          d.bbox.size_x, d.bbox.size_y, h.score])
rclpy.init(); rclpy.spin(L())
```

Topic prefix with a team (`/team1/robotN/...`) is unverified: run `ros2 topic list | grep vision`.

Also log, same clock: body pose (use ground truth `/team1/robotN/soccer/sim/ground_truth/robot_pose`,
or the localisation estimate if testing realistically) and head joint angles (LowState) to a
`poses` struct: `t, robot, x, y, theta, headYaw, headPitch` (pitch + = down). The camera
intrinsics/mount struct comes from `k1SimCamera(trunkHeight)`; get trunkHeight once from the sim
(`data.cam_xpos[cam_id][2] - 0.356` with head pitch 0) or from the Trunk z in the replay.

## MATLAB

```matlab
cam   = k1SimCamera(0.55);          % trunkHeight is a placeholder: measure it
poses = struct('t',..,'robot',..,'x',..,'y',..,'theta',..,'headYaw',..,'headPitch',..);
pk = detectionsToPackets('detections.csv', cam, poses, struct('rangeMethod','fuse'));
```

Packets drop straight into the same consumers as `makePackets` output (extra fields
`LastDetection.RangeStd/Conf/Pixel`). Limitations: bearing/range come purely from pixels +
believed pose + head angles (errors in those are folded into DetectionNoiseStd / posStd / range
std); `TrueLabel` is the raw class string (no identity), so it is not usable for scoring
association; opponents/teammates need a real detector (or the real robot) to appear.
