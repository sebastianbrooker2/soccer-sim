#!/usr/bin/env python3
"""Log sim detections (vision_msgs/Detection2DArray) to CSV. Runs inside the sim container.
usage: log_detections.py DURATION_S OUT_PREFIX [robots=1,2,3,4,5,6]
writes OUT_PREFIX.csv  : t,robot,class,u_px,v_px,w_px,h_px,score,trunk_x,trunk_y,t_clock
       OUT_PREFIX_frames.csv : t,robot,n_det   (one row per message incl. empty ones)
       OUT_PREFIX_caminfo.csv: robot,topic,width,height,fx,fy,cx,cy
t = header.stamp (sim time); t_clock = latest /clock at receive time (-1 if none)."""
import csv, sys, time
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import Detection2DArray

DUR = float(sys.argv[1]); PFX = sys.argv[2]
ROBOTS = [int(x) for x in (sys.argv[3] if len(sys.argv) > 3 else "1,2,3,4,5,6").split(",")]

class L(Node):
    def __init__(s):
        super().__init__("det_logger")
        s.clock = -1.0
        s.f = open(PFX + ".csv", "w", newline=""); s.w = csv.writer(s.f)
        s.w.writerow("t,robot,class,u_px,v_px,w_px,h_px,score,trunk_x,trunk_y,t_clock".split(","))
        s.f2 = open(PFX + "_frames.csv", "w", newline=""); s.w2 = csv.writer(s.f2)
        s.w2.writerow(["t", "robot", "n_det"])
        s.f3 = open(PFX + "_caminfo.csv", "w", newline=""); s.w3 = csv.writer(s.f3)
        s.w3.writerow("robot,topic,width,height,fx,fy,cx,cy".split(","))
        s.seen_ci = set()
        s.create_subscription(Clock, "/clock", lambda m: setattr(s, "clock", m.clock.sec + m.clock.nanosec * 1e-9), 10)
        for r in ROBOTS:
            s.create_subscription(Detection2DArray, f"/robot{r}/soccer/sim/vision/detections",
                                  lambda m, r=r: s.cb(r, m), 10)
        s.create_timer(2.0, s.find_ci)
    def find_ci(s):
        for name, types in s.get_topic_names_and_types():
            if name.endswith("camera_info") and "sensor_msgs/msg/CameraInfo" in types and name not in s.seen_ci:
                s.seen_ci.add(name)
                s.create_subscription(CameraInfo, name, lambda m, n=name: s.ci(n, m), 1)
    def ci(s, name, m):
        if name + "_done" in s.seen_ci: return
        s.seen_ci.add(name + "_done")
        r = next((x for x in ROBOTS if f"robot{x}" in name), -1)
        s.w3.writerow([r, name, m.width, m.height, m.k[0], m.k[4], m.k[2], m.k[5]]); s.f3.flush()
    def cb(s, r, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        s.w2.writerow([f"{t:.4f}", r, len(m.detections)])
        for d in m.detections:
            if not d.results: continue
            res = d.results[0]; p = res.pose.pose.position
            s.w.writerow([f"{t:.4f}", r, res.hypothesis.class_id, f"{d.bbox.center.position.x:.2f}",
                          f"{d.bbox.center.position.y:.2f}", f"{d.bbox.size_x:.2f}", f"{d.bbox.size_y:.2f}",
                          f"{res.hypothesis.score:.3f}", f"{p.x:.4f}", f"{p.y:.4f}", f"{s.clock:.4f}"])

rclpy.init(); n = L(); t0 = time.time()
while time.time() - t0 < DUR and rclpy.ok():
    rclpy.spin_once(n, timeout_sec=0.2)
for f in (n.f, n.f2, n.f3): f.close()
n.destroy_node(); rclpy.shutdown()
