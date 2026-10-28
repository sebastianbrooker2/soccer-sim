"""Shared helpers: replay rendering (EGL mujoco), YOLO inference (NUbots pre/post-processing), GT projection."""
import glob, json, os
os.environ.setdefault('MUJOCO_GL', 'egl')
import numpy as np, cv2, mujoco
import onnxruntime as ort

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL_XML = os.environ.get('SOCCER_MODEL_XML', os.path.expanduser('~/.soccer-train/sim/mjcf/football_match_pitch_6_K1.xml'))
CLASSES = ['Ball', 'Goalpost', 'Robot', 'L_cross', 'T_cross', 'X_cross']
W, H, FOVY = 320, 240, 58.0
FY = H / (2 * np.tan(np.radians(FOVY) / 2)); FX = FY; CX, CY = W / 2, H / 2
TEAM_RGBA = {1: (0.15, 0.35, 0.95, 1), 2: (0.95, 0.15, 0.15, 1)}


class Scene:
    def __init__(self, recolor=False):
        self.m = mujoco.MjModel.from_xml_path(MODEL_XML); self.d = mujoco.MjData(self.m)
        self.r = mujoco.Renderer(self.m, H, W)
        if recolor:
            for g in range(self.m.ngeom):
                nm = self.m.body(int(self.m.body_rootid[self.m.geom_bodyid[g]])).name.split('_')[0]
                if nm.startswith('robot') and nm[5:].isdigit():
                    a = self.m.geom_rgba[g][3]
                    if a > 0:
                        self.m.geom_rgba[g] = TEAM_RGBA[1 if int(nm[5:]) <= 3 else 2]
                        self.m.geom_matid[g] = -1
        self.qadr = {}
        self.cams = {}

    def _qa(self, jn):
        if jn not in self.qadr: self.qadr[jn] = self.m.joint(jn).qposadr[0]
        return self.qadr[jn]

    def set_frame(self, fr):
        d = self.d
        for rb in fr['robots']:
            n = rb['name']; p = self._qa(n + '_world_joint')
            d.qpos[p:p+3] = rb['basePosition']; d.qpos[p+3:p+7] = rb['baseQuaternion']
            for jn, v in rb['joints'].items(): d.qpos[self._qa(f'{n}_{jn}')] = v
        p = self._qa('ball-root')
        d.qpos[p:p+3] = fr['ball']['position']; d.qpos[p+3:p+7] = fr['ball']['quaternion']
        mujoco.mj_kinematics(self.m, d); mujoco.mj_camlight(self.m, d)

    def render(self, robot):
        self.r.update_scene(self.d, camera=f'robot{robot}_rgbd_camera')
        return self.r.render().copy()  # RGB uint8 HxWx3

    def cam_pose(self, robot):
        if robot not in self.cams: self.cams[robot] = self.m.camera(f'robot{robot}_rgbd_camera').id
        i = self.cams[robot]
        return self.d.cam_xpos[i].copy(), self.d.cam_xmat[i].reshape(3, 3).copy()


def iter_frames(rec, hz, t0=0, t1=1e9):
    nxt = t0
    for f in sorted(glob.glob(rec + '/data/chunks/frames_*.jsonl')):
        for line in open(f):
            line = line.strip()
            if not line: continue
            fr = json.loads(line); t = fr['time']
            if t < nxt - 0.075: continue   # raw replay frames are ~6.6 Hz (0.15 s); pick the frame nearest each tick
            if t > t1: return
            nxt = nxt + 1.0 / hz
            if nxt < t - 0.075: nxt = t + 1.0 / hz
            yield fr


def project(P, cp, Rm):
    """world pts Nx3 -> (u,v,depth). MuJoCo cam: looks along -z, x right, y up."""
    q = (np.asarray(P) - cp) @ Rm  # = Rm^T (P-cp)
    dep = -q[:, 2]
    with np.errstate(divide='ignore', invalid='ignore'):
        return CX + FX * q[:, 0] / dep, CY - FY * q[:, 1] / dep, dep


def gt_boxes(fr, robot, cp, Rm, ball_r=0.107, rob_h=0.95, rob_w=0.30):
    """Projected GT list of dict(cls,u,v,w,h,depth,vis) (unclipped box + clipped-area ratio)."""
    out = []
    def mk(cls, u, v, dep):
        x0, x1, y0, y1 = u.min(), u.max(), v.min(), v.max()
        fa = (x1 - x0) * (y1 - y0)
        cx0, cx1, cy0, cy1 = max(0, x0), min(W, x1), max(0, y0), min(H, y1)
        ratio = 0 if (cx1 <= cx0 or cy1 <= cy0 or fa <= 0) else (cx1-cx0)*(cy1-cy0)/fa
        return dict(cls=cls, u=(x0+x1)/2, v=(y0+y1)/2, w=x1-x0, h=y1-y0, depth=float(np.mean(dep)), vis=ratio,
                    cu=(cx0+cx1)/2, cv=(cy0+cy1)/2, cw=max(0, cx1-cx0), ch=max(0, cy1-cy0))
    b = np.array(fr['ball']['position'])
    u, v, dep = project(b[None], cp, Rm)
    if dep[0] > 0.05:
        rx, ry = FX * ball_r / dep[0], FY * ball_r / dep[0]
        out.append(mk('Ball', np.array([u[0]-rx, u[0]+rx]), np.array([v[0]-ry, v[0]+ry]), dep))
    for rb in fr['robots']:
        i = int(rb['name'][5:])
        if i == robot: continue
        p = np.array(rb['basePosition'])
        a = np.arange(8) * np.pi / 4
        pts = np.array([[p[0] + rob_w/2*np.cos(aa), p[1] + rob_w/2*np.sin(aa), z] for z in (0, rob_h/2, rob_h) for aa in a])
        u, v, dep = project(pts, cp, Rm)
        if (dep > 0.05).all():
            g = mk('Robot', u, v, dep); g['id'] = i; out.append(g)
    return out


class Detector:
    def __init__(self, model, provider='CUDAExecutionProvider', min_conf=0.05):
        self.s = ort.InferenceSession(model, providers=[provider, 'CPUExecutionProvider'])
        self.inp = self.s.get_inputs()[0].name; self.min_conf = min_conf

    def raw(self, rgb):
        """NUbots preprocessing: pad to square at top-left with 0, resize to 640, RGB/255. Returns Nx(6) [u,v,w,h,conf,cls] in 320x240 px
        after class-agnostic NMS(0.5, score>0.1) of candidates with conf>min_conf."""
        sq = np.zeros((W, W, 3), np.uint8); sq[:H] = rgb
        x = cv2.resize(sq, (640, 640), interpolation=cv2.INTER_LINEAR).astype(np.float32) / 255.0
        o = self.s.run(None, {self.inp: x.transpose(2, 0, 1)[None]})[0][0].T  # 8400 x 10
        conf = o[:, 4:].max(1); cls = o[:, 4:].argmax(1)
        k = conf > self.min_conf
        o, conf, cls = o[k], conf[k], cls[k]
        if not len(o): return np.zeros((0, 6))
        sc = 0.5  # 640 -> 320
        boxes = [[int((c[0]-.5*c[2])*sc), int((c[1]-.5*c[3])*sc), int(c[2]*sc), int(c[3]*sc)] for c in o]
        idx = cv2.dnn.NMSBoxes(boxes, conf.tolist(), 0.1, 0.5)
        idx = np.array(idx).reshape(-1).astype(int)
        res = []
        for i in idx:
            c = o[i]
            res.append([c[0]*sc, c[1]*sc, c[2]*sc, c[3]*sc, conf[i], cls[i]])  # float (more precise than NUbots int rect)
        return np.array(res).reshape(-1, 6)


def apply_thresh(dets, th):
    if not len(dets): return dets
    keep = np.array([d[4] > th[int(d[5])] for d in dets])
    return dets[keep]


# thresholds per class index 0..5
TH_DEFAULT = [0.6, 0.5, 0.2, 0.65, 0.65, 0.65]
TH_WEBOTS = [0.3, 0.3, 0.5, 0.2, 0.2, 0.2]
