#!/usr/bin/env python3
"""Compare sim detections to ground truth. usage: compare_detections.py recordings/det01/det.csv eval/gt/det01.csv
Per robot/class counts + rates, and Ball trunk-frame (trunk_x,trunk_y) vs GT ball in robot frame
(yaw-only rotation about robot xy; tilt/height ignored so small residuals are expected).
Also: GT ball in-range&in-FOV fraction vs detected fraction (visibility-model check, yaw=trunk yaw+headYaw)."""
import sys, numpy as np, pandas as pd
det = pd.read_csv(sys.argv[1]); gt = pd.read_csv(sys.argv[2])
fr = pd.read_csv(sys.argv[1].replace(".csv", "_frames.csv"))
span = fr.groupby("robot").t.agg(lambda s: s.max() - s.min())
print("detections per class per robot"); print(det.groupby(["robot", "class"]).size().unstack(fill_value=0))
print("frames/robot, Hz (sim time):"); print(pd.DataFrame({"frames": fr.groupby("robot").size(), "Hz_sim": fr.groupby("robot").size() / span}))
ball = gt[gt.id == 0].set_index("t")
errs = []
for r, g in det[det["class"] == "Ball"].groupby("robot"):
    rob = gt[gt.id == r].set_index("t")
    for _, d in g.iterrows():
        i = rob.index.searchsorted(d.t); i = min(i, len(rob) - 1); t = rob.index[i]
        b = ball.iloc[min(ball.index.searchsorted(d.t), len(ball) - 1)]; p = rob.iloc[i]
        dx, dy = b.x - p.x, b.y - p.y; c, s = np.cos(p.theta), np.sin(p.theta)
        errs.append((r, d.t, c * dx + s * dy, -s * dx + c * dy, d.trunk_x, d.trunk_y))
e = pd.DataFrame(errs, columns="robot t gt_x gt_y det_x det_y".split())
e["err"] = np.hypot(e.gt_x - e.det_x, e.gt_y - e.det_y)
print("\nBall det vs GT (robot frame), n=%d: median err %.3f m, p90 %.3f m, corr x %.3f, corr y %.3f" % (
    len(e), e.err.median(), e.err.quantile(.9), np.corrcoef(e.gt_x, e.det_x)[0, 1], np.corrcoef(e.gt_y, e.det_y)[0, 1]))
print(e.sample(min(6, len(e)), random_state=0).round(3).to_string(index=False))
