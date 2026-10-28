#!/usr/bin/env python3
"""Bird's-eye JPGs (640x420) per whole second of real01. MUJOCO_GL=egl tools/render/venv/bin/python tools/viz/render_top.py [t_max]
Free camera straight down; free cam az=-90 el=-90 gives x-left,y-down, so the image is rotated 180 deg here: x->right, y->up."""
import glob, json, os, sys, numpy as np, mujoco
from PIL import Image
import os as _o; REC=_o.path.join(_o.path.dirname(_o.path.abspath(__file__)),'..','..','recordings','real01'); OUT=os.path.join(os.path.dirname(os.path.abspath(__file__)),'cache_top')
MODEL=_o.environ.get('SOCCER_MODEL_XML', _o.path.expanduser('~/.soccer-train/sim/mjcf/football_match_pitch_6_K1.xml'))
TMAX=float(sys.argv[1]) if len(sys.argv)>1 else 1e9
W,H=640,420; FOVY=20.0
# visible height 9+2*0.8=10.6 m -> distance = (10.6/2)/tan(fovy/2)
DIST=(10.6/2)/np.tan(np.radians(FOVY/2))
os.makedirs(OUT,exist_ok=True)
m=mujoco.MjModel.from_xml_path(MODEL); m.vis.global_.offheight=max(m.vis.global_.offheight,H); m.vis.global_.offwidth=max(m.vis.global_.offwidth,W)
d=mujoco.MjData(m); r=mujoco.Renderer(m,H,W)
cam=mujoco.MjvCamera(); cam.type=mujoco.mjtCamera.mjCAMERA_FREE
cam.lookat[:]=[0,0,0]; cam.distance=DIST; cam.azimuth=-90; cam.elevation=-90
m.vis.global_.fovy=FOVY
# hide ceilings etc. handled by free camera being above; set clip
m.vis.map.znear=0.05; m.vis.map.zfar=100
nxt=0.0; n=0
for f in sorted(glob.glob(REC+'/data/chunks/frames_*.jsonl')):
    for line in open(f):
        fr=json.loads(line); t=fr['time']
        if t>TMAX: break
        if t<nxt-1e-6: continue
        key=round(nxt,2); nxt+=1.0
        for rb in fr['robots']:
            p=m.joint(rb['name']+'_world_joint').qposadr[0]
            d.qpos[p:p+3]=rb['basePosition']; d.qpos[p+3:p+7]=rb['baseQuaternion']
            for jn,v in rb['joints'].items(): d.qpos[m.joint(f"{rb['name']}_{jn}").qposadr[0]]=v
        p=m.joint('ball-root').qposadr[0]
        d.qpos[p:p+3]=fr['ball']['position']; d.qpos[p+3:p+7]=fr['ball']['quaternion']
        mujoco.mj_kinematics(m,d); mujoco.mj_camlight(m,d)
        r.update_scene(d,camera=cam)
        Image.fromarray(r.render()[::-1,::-1]).save(f'{OUT}/{key:07.2f}.jpg',quality=88); n+=1
print(n,'frames')
