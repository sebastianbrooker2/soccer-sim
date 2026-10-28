import sys, pickle, time, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from yolo_lib import *
rec = '/home/seb/soccer-sim/recordings/real01'
t0, t1 = float(sys.argv[1]), float(sys.argv[2])
models = {n: Detector(f'{HERE}/models/{n}.onnx') for n in ['yolo_hsl_ft', 'yolo', 'old']}
out = []; t_start = time.time()
sc = {False: Scene(False), True: Scene(True)}
os.makedirs('/tmp/pilot_imgs', exist_ok=True)
n = 0
for fr in iter_frames(rec, 5, t0, t1):
    for rc in (False, True):
        s = sc[rc]; s.set_frame(fr)
        for rob in (1, 2, 3):
            img = s.render(rob); cp, Rm = s.cam_pose(rob)
            gt = gt_boxes(fr, rob, cp, Rm)
            rec_ = dict(t=fr['time'], robot=rob, recolor=rc, gt=gt, dets={k: m.raw(img) for k, m in models.items()})
            out.append(rec_)
            if n % 150 == 0: cv2.imwrite(f'/tmp/pilot_imgs/{int(rc)}_{fr["time"]:.1f}_r{rob}.png', img[:, :, ::-1])
            n += 1
print(n, time.time() - t_start)
pickle.dump(out, open('/tmp/claude-1000/-home-seb-soccer-sim/b40ef6cc-2250-4d87-a6da-409ad2cbd4ac/scratchpad/pilot.pkl', 'wb'))
