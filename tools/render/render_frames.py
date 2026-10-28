#!/usr/bin/env python3
"""Offline re-render of head-camera frames from a recorded replay (route B).
Usage: MUJOCO_GL=egl venv/bin/python render_frames.py --rec ../../recordings/real01 --robot robot1 --hz 2 --t0 0 --dur 20 --out out/
"""
import argparse, glob, json, os, time
import numpy as np, mujoco
from PIL import Image

ap = argparse.ArgumentParser()
ap.add_argument('--rec', default=__import__('os').path.join(__import__('os').path.dirname(__import__('os').path.abspath(__file__)),'..','..','recordings','real01'))
ap.add_argument('--model', default=__import__('os').environ.get('SOCCER_MODEL_XML', __import__('os').path.expanduser('~/.soccer-train/sim/mjcf/football_match_pitch_6_K1.xml')))
ap.add_argument('--robot', default='robot1')
ap.add_argument('--hz', type=float, default=2)
ap.add_argument('--t0', type=float, default=0)
ap.add_argument('--dur', type=float, default=20)
ap.add_argument('--w', type=int, default=320); ap.add_argument('--h', type=int, default=240)
ap.add_argument('--out', default='out')
a = ap.parse_args()
os.makedirs(a.out, exist_ok=True)
m = mujoco.MjModel.from_xml_path(a.model); d = mujoco.MjData(m)
J = lambda n: m.joint(n)
r = mujoco.Renderer(m, a.h, a.w)
nxt = a.t0; n = 0; t_start = time.time()
for f in sorted(glob.glob(a.rec + '/data/chunks/frames_*.jsonl')):
    for line in open(f):
        fr = json.loads(line); t = fr['time']
        if t < nxt: continue
        if t > a.t0 + a.dur: nxt = None; break
        nxt += 1.0 / a.hz
        for rb in fr['robots']:           # set ALL robots so others are visible
            p = m.joint(rb['name'] + '_world_joint').qposadr[0]
            d.qpos[p:p+3] = rb['basePosition']; d.qpos[p+3:p+7] = rb['baseQuaternion']  # wxyz
            for jn, v in rb['joints'].items():
                d.qpos[m.joint(f"{rb['name']}_{jn}").qposadr[0]] = v
        p = m.joint('ball-root').qposadr[0]
        d.qpos[p:p+3] = fr['ball']['position']; d.qpos[p+3:p+7] = fr['ball']['quaternion']
        mujoco.mj_kinematics(m, d); mujoco.mj_camlight(m, d)
        r.update_scene(d, camera=a.robot + '_rgbd_camera')
        Image.fromarray(r.render()).save(f'{a.out}/{a.robot}_{t:07.2f}.png'); n += 1
    if nxt is None: break
print(f'{n} frames in {time.time()-t_start:.1f}s')
