#!/usr/bin/env python3
"""Run trained YOLO (NUbots_K1 models) on frames re-rendered from a Booster-sim replay.
Usage: LD_LIBRARY_PATH=$(./ldpath.sh) venv/bin/python detect_replay.py --rec ../../recordings/real01 --robots 1 2 3 --hz 5
Outputs <rec>/yolo/: dets.csv (t,robot,class,u_px,v_px,w_px,h_px,conf; per-class NMS'd detections above the
NUbots Yolo.yaml default thresholds), dets_all.csv (same, conf>0.1 for re-thresholding), dets_frames.csv
(t,robot,nDets - every processed frame), stats.json, samples/*.png.
Models: Ball/Goalpost/L/T/X from yolo_hsl_ft.onnx, Robot from yolo.onnx (hsl_ft has ~no robot recall).
t = replay frame time (== eval/gt t, both start at 0)."""
import sys, os, csv, json, time, argparse
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from yolo_lib import *

ap = argparse.ArgumentParser()
ap.add_argument('--rec', required=True); ap.add_argument('--robots', type=int, nargs='+', default=[1, 2, 3])
ap.add_argument('--hz', type=float, default=5); ap.add_argument('--t0', type=float, default=0); ap.add_argument('--t1', type=float, default=1e9)
ap.add_argument('--model-main', default='yolo_hsl_ft'); ap.add_argument('--model-robot', default='yolo')
ap.add_argument('--recolor', action='store_true', help='recolour robot geoms by team (default: white robots as in the sim)')
ap.add_argument('--out', default=None); ap.add_argument('--samples', type=int, default=40)
a = ap.parse_args()
out = a.out or os.path.join(a.rec, 'yolo'); os.makedirs(out + '/samples', exist_ok=True)
dm = Detector(f'{HERE}/models/{a.model_main}.onnx', min_conf=0.1)
dr = dm if a.model_robot == a.model_main else Detector(f'{HERE}/models/{a.model_robot}.onnx', min_conf=0.1)
sc = Scene(a.recolor)
fd, fa, ff = (open(f'{out}/{n}.csv', 'w', newline='') for n in ('dets', 'dets_all', 'dets_frames'))
wd, wa, wf = csv.writer(fd), csv.writer(fa), csv.writer(ff)
for w in (wd, wa): w.writerow('t,robot,class,u_px,v_px,w_px,h_px,conf'.split(','))
wf.writerow(['t', 'robot', 'nDets'])

def iou(a, b):
    ix = max(0, min(a[0]+a[2]/2, b[0]+b[2]/2) - max(a[0]-a[2]/2, b[0]-b[2]/2))
    iy = max(0, min(a[1]+a[3]/2, b[1]+b[3]/2) - max(a[1]-a[3]/2, b[1]-b[3]/2))
    return ix*iy/(a[2]*a[3] + b[2]*b[3] - ix*iy + 1e-9)

BANDS = [(0, 2), (2, 4), (4, 6), (6, 8), (8, 99)]
st = {c: dict(gt=[0]*5, hit=[0]*5, det=0, tp=0, fp=0, err=[]) for c in ('Ball', 'Robot')}
nframes = 0; t_start = time.time(); tmax = 0; tmin = 1e18; nsamp = 0
for fr in iter_frames(a.rec, a.hz, a.t0, a.t1):
    t = fr['time']; tmax = max(tmax, t); tmin = min(tmin, t); sc.set_frame(fr)
    for rob in a.robots:
        img = sc.render(rob); cp, Rm = sc.cam_pose(rob)
        r1 = dm.raw(img); r2 = r1 if dr is dm else dr.raw(img)
        allc = np.vstack([r1[r1[:, 5] != 2], r2[r2[:, 5] == 2]]) if len(r1) or len(r2) else np.zeros((0, 6))
        kept = apply_thresh(allc, TH_DEFAULT)
        for d in allc: wa.writerow([f'{t:.4f}', rob, CLASSES[int(d[5])], *(f'{x:.2f}' for x in d[:4]), f'{d[4]:.3f}'])
        for d in kept: wd.writerow([f'{t:.4f}', rob, CLASSES[int(d[5])], *(f'{x:.2f}' for x in d[:4]), f'{d[4]:.3f}'])
        wf.writerow([f'{t:.4f}', rob, len(kept)]); nframes += 1
        # --- evaluation vs projected GT ---
        gts = gt_boxes(fr, rob, cp, Rm)
        for cname, ci in (('Ball', 0), ('Robot', 2)):
            s = st[cname]; ds = sorted([d for d in kept if int(d[5]) == ci], key=lambda d: -d[4]); used = set()
            gl = [g for g in gts if g['cls'] == cname]
            for d in ds:
                s['det'] += 1; best = -1; bi = 0.2
                for j, g in enumerate(gl):
                    if j in used or g['vis'] < 0.3: continue
                    v = iou(d, [g['cu'], g['cv'], g['cw'], g['ch']])
                    if v > bi: bi, best = v, j
                if best >= 0:
                    used.add(best); g = gl[best]; s['tp'] += 1
                    s['err'].append([d[0]-g['cu'], d[1]-g['cv'], d[2]-g['cw'], d[3]-g['ch'], bi, g['depth'], g['cw'], g['ch'], d[4]])
                else: s['fp'] += 1
            for j, g in enumerate(gl):
                if g['vis'] >= 0.7 and g['cw'] >= 4 and g['ch'] >= 4:
                    b = next(i for i, (lo, hi) in enumerate(BANDS) if lo <= g['depth'] < hi)
                    s['gt'][b] += 1; s['hit'][b] += (j in used)
        if nframes % max(1, int(1e9)) == 0: pass
        if nsamp < a.samples and nframes % 97 == 1:
            im = np.ascontiguousarray(img[:, :, ::-1]).copy()
            for g in gts:
                if g['vis'] > 0.3: cv2.rectangle(im, (int(g['cu']-g['cw']/2), int(g['cv']-g['ch']/2)), (int(g['cu']+g['cw']/2), int(g['cv']+g['ch']/2)), (0, 255, 0), 1)
            for d in kept:
                col = [(0, 0, 255), (255, 0, 255), (0, 165, 255), (255, 255, 0), (255, 255, 0), (255, 255, 0)][int(d[5])]
                cv2.rectangle(im, (int(d[0]-d[2]/2), int(d[1]-d[3]/2)), (int(d[0]+d[2]/2), int(d[1]+d[3]/2)), col, 1)
                cv2.putText(im, f'{CLASSES[int(d[5])][0]}{d[4]:.2f}', (int(d[0]-d[2]/2), int(d[1]-d[3]/2)-2), 0, 0.35, col, 1)
            cv2.imwrite(f'{out}/samples/{t:07.2f}_r{rob}.png', cv2.resize(im, None, fx=2, fy=2)); nsamp += 1
    if nframes % 600 < len(a.robots): print(f't={t:.0f}s frames={nframes} {nframes/(time.time()-t_start):.1f} fps', flush=True)
for f in (fd, fa, ff): f.close()
minutes = nframes / len(a.robots) / a.hz / 60
res = dict(rec=a.rec, robots=a.robots, hz=a.hz, frames=nframes, minutes_per_robot=minutes, recolor=a.recolor,
           models=dict(main=a.model_main, robot=a.model_robot), thresholds=dict(zip(CLASSES, TH_DEFAULT)),
           runtime_s=time.time()-t_start, gt_note='GT=projected ball sphere r0.107 / robot cylinder 0.30x0.95 at trunk xy; ignores occlusion; recall over GT with >=70% in image and >=4px; match IoU>0.2 vs clipped GT box',
           bands_m=BANDS)
for c, s in st.items():
    e = np.array(s['err']) if s['err'] else np.zeros((0, 9)); d = {}
    d['recall_by_band'] = [h / g if g else None for h, g in zip(s['hit'], s['gt'])]; d['gt_by_band'] = s['gt']
    d['recall'] = sum(s['hit']) / max(1, sum(s['gt'])); d['precision'] = s['tp'] / max(1, s['det'])
    d['n_det'] = s['det']; d['n_fp'] = s['fp']; d['fp_per_min_per_robot'] = s['fp'] / max(minutes * len(a.robots), 1e-9) * len(a.robots) / len(a.robots)
    d['fp_per_min_per_robot'] = s['fp'] / max(minutes * len(a.robots), 1e-9)
    if len(e):
        for k, nm in enumerate(['du', 'dv', 'dw', 'dh']):
            d[nm] = dict(mean=float(e[:, k].mean()), std=float(e[:, k].std()), median=float(np.median(e[:, k])), p95abs=float(np.percentile(abs(e[:, k]), 95)))
        d['iou_median'] = float(np.median(e[:, 4])); d['centre_err_px_median'] = float(np.median(np.hypot(e[:, 0], e[:, 1])))
        d['rel_dw_std'] = float((e[:, 2] / np.maximum(e[:, 6], 1)).std()); d['rel_dh_std'] = float((e[:, 3] / np.maximum(e[:, 7], 1)).std())
        d['conf_tp_median'] = float(np.median(e[:, 8]))
        d['err_by_band'] = {f'{lo}-{hi}m': dict(n=int(m.sum()), du_std=float(e[m, 0].std()), dv_std=float(e[m, 1].std()), dw_std=float(e[m, 2].std()), dh_std=float(e[m, 3].std())) for lo, hi in BANDS if (m := (e[:, 5] >= lo) & (e[:, 5] < hi)).sum() > 3}
    res[c] = d
json.dump(res, open(f'{out}/stats.json', 'w'), indent=1)
print(json.dumps(res, indent=1)[:3000]); print('done', time.time() - t_start)
