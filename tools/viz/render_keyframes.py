#!/usr/bin/env python3
"""Cache head-camera JPGs (320x240) for key times, all 3 robots. Run with tools/render/venv python, MUJOCO_GL=egl."""
import glob, json, os, sys, numpy as np, mujoco
from PIL import Image
import os as _o; REC=_o.path.join(_o.path.dirname(_o.path.abspath(__file__)),'..','..','recordings','real01'); OUT=os.path.join(os.path.dirname(os.path.abspath(__file__)),'cache')
MODEL=_o.environ.get('SOCCER_MODEL_XML', _o.path.expanduser('~/.soccer-train/sim/mjcf/football_match_pitch_6_K1.xml'))
STEP=float(sys.argv[1]) if len(sys.argv)>1 else 1.0
os.makedirs(OUT,exist_ok=True)
m=mujoco.MjModel.from_xml_path(MODEL); d=mujoco.MjData(m); r=mujoco.Renderer(m,240,320)
nxt=0.0; n=0
for f in sorted(glob.glob(REC+'/data/chunks/frames_*.jsonl')):
    for line in open(f):
        fr=json.loads(line); t=fr['time']
        if t<nxt-1e-6: continue
        key=round(nxt,2); nxt+=STEP
        for rb in fr['robots']:
            p=m.joint(rb['name']+'_world_joint').qposadr[0]
            d.qpos[p:p+3]=rb['basePosition']; d.qpos[p+3:p+7]=rb['baseQuaternion']
            for jn,v in rb['joints'].items(): d.qpos[m.joint(f"{rb['name']}_{jn}").qposadr[0]]=v
        p=m.joint('ball-root').qposadr[0]
        d.qpos[p:p+3]=fr['ball']['position']; d.qpos[p+3:p+7]=fr['ball']['quaternion']
        mujoco.mj_kinematics(m,d); mujoco.mj_camlight(m,d)
        for k in (1,2,3):
            r.update_scene(d,camera=f'robot{k}_rgbd_camera')
            Image.fromarray(r.render()).save(f'{OUT}/{key:07.2f}_r{k}.jpg',quality=88)
        n+=1
print(n,'key times')
