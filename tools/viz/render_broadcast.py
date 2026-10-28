#!/usr/bin/env python3
"""Broadcast-view JPGs (720x420) per whole second of real01. MUJOCO_GL=egl tools/render/venv/bin/python tools/viz/render_broadcast.py [t_max]
Slight-angle TV camera behind the -y sideline (az=90 looks +y), x to the right. Env: FOVY AZ EL DIST LY LZ ONLY=t OUTDIR."""
import glob, json, os, sys, numpy as np, mujoco
from PIL import Image
REC='/home/seb/soccer-sim/recordings/real01'; OUT=os.environ.get('OUTDIR') or os.path.join(os.path.dirname(os.path.abspath(__file__)),'cache_broadcast')
MODEL='/home/seb/.soccer-train/sim/mjcf/football_match_pitch_6_K1.xml'
TMAX=float(sys.argv[1]) if len(sys.argv)>1 else 1e9
W,H=720,420; FOVY=float(os.environ.get('FOVY',40)); AZ=float(os.environ.get('AZ',90)); EL=float(os.environ.get('EL',-42)); DIST=float(os.environ.get('DIST',17)); LZ=float(os.environ.get('LZ',0)); LY=float(os.environ.get('LY',0)); ONLY=os.environ.get('ONLY')
os.makedirs(OUT,exist_ok=True)
m=mujoco.MjModel.from_xml_path(MODEL); m.vis.global_.offheight=max(m.vis.global_.offheight,H); m.vis.global_.offwidth=max(m.vis.global_.offwidth,W)
d=mujoco.MjData(m); r=mujoco.Renderer(m,H,W)
cam=mujoco.MjvCamera(); cam.type=mujoco.mjtCamera.mjCAMERA_FREE
cam.lookat[:]=[0,LY,LZ]; cam.distance=DIST; cam.azimuth=AZ; cam.elevation=EL
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
        if ONLY and abs(key-float(ONLY))>1e-6: continue
        for rb in fr['robots']:
            p=m.joint(rb['name']+'_world_joint').qposadr[0]
            d.qpos[p:p+3]=rb['basePosition']; d.qpos[p+3:p+7]=rb['baseQuaternion']
            for jn,v in rb['joints'].items(): d.qpos[m.joint(f"{rb['name']}_{jn}").qposadr[0]]=v
        p=m.joint('ball-root').qposadr[0]
        d.qpos[p:p+3]=fr['ball']['position']; d.qpos[p+3:p+7]=fr['ball']['quaternion']
        mujoco.mj_kinematics(m,d); mujoco.mj_camlight(m,d)
        r.update_scene(d,camera=cam)
        Image.fromarray(r.render()).save(f'{OUT}/{key:07.2f}.jpg',quality=88); n+=1
print(n,'frames')
