#!/usr/bin/env python3
"""Key-frame viewer for match real01: cameras+YOLO dets, per-robot baseline minimaps, joint SRIF minimap.
Run:  ~/miniconda3/bin/python tools/viz/viewer.py [--t 120] [--png out.png | --mp4 out.mp4 --t0 100 --dur 60]
Keys: Left/Right +-1 s | PageUp/PageDown +-10 s | Space play/pause | 1/2/3 joint copy | g ghost GT | q quit"""
import argparse, csv, os, collections
import numpy as np
ap = argparse.ArgumentParser()
ap.add_argument('--t', type=float, default=0); ap.add_argument('--png'); ap.add_argument('--mp4')
ap.add_argument('--t0', type=float, default=0); ap.add_argument('--dur', type=float, default=60)
ap.add_argument('--copy', type=int, default=1)
a = ap.parse_args()
import matplotlib
if a.png or a.mp4: matplotlib.use('Agg')
else:
    try: matplotlib.use('TkAgg')
    except Exception: pass
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse, Wedge, Rectangle, Circle
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__)); REC = os.path.join(HERE, '..', '..', 'recordings', 'real01', 'yolo')
FOV = np.degrees(1.2863); CHI2 = 5.991; FX, FY = 7.0, 4.5
# ---- data
trk = collections.defaultdict(list)       # (src,robot,t) -> list of tracks
for r in csv.DictReader(open(f'{HERE}/data/tracks.csv')):
    trk[(r['src'], int(r['robot']), round(float(r['t']), 2))].append(
        (r['name'], r['class'], float(r['mx']), float(r['my']), float(r['cxx']), float(r['cxy']), float(r['cyy'])))
gt = {}
for r in csv.DictReader(open(f'{HERE}/data/gt.csv')):
    v = {k: float(x) for k, x in r.items()}; gt[round(v['t'], 2)] = v
times = sorted(gt)
dets = collections.defaultdict(list)
for r in csv.DictReader(open(f'{REC}/dets.csv')):
    dets[(int(r['robot']), round(float(r['t']), 3))].append(r)
ftimes = collections.defaultdict(list)
for r in csv.DictReader(open(f'{REC}/dets_frames.csv')): ftimes[int(r['robot'])].append(float(r['t']))
ftimes = {k: np.array(sorted(v)) for k, v in ftimes.items()}
CLS_COL = {'Ball': '#ff8c00', 'Robot': '#00e5ff', 'Goalpost': '#ffee00', 'L_cross': '#7CFC00', 'T_cross': '#ff40ff', 'X_cross': '#ffffff'}
C_TEAM, C_OPP, C_BALLT, C_ROBT = '#1f6fe0', '#d62728', '#ff7f0e', '#7b2cbf'
OBS_COL = {1: '#1b9e77', 2: '#e6ab02', 3: '#7570b3'}
state = dict(i=0, play=False, copy=a.copy, ghost=True)

fig = plt.figure(figsize=(19, 7.6), facecolor='#15181d')
gs = fig.add_gridspec(2, 4, width_ratios=[1, 1, 1, 1.75], left=0.01, right=0.995, top=0.93, bottom=0.04, wspace=0.05, hspace=0.04)
axc = [fig.add_subplot(gs[0, k]) for k in range(3)]
axm = [fig.add_subplot(gs[1, k]) for k in range(3)]
axj = fig.add_subplot(gs[:, 3])
for x in axc: x.set_anchor('S')
for x in axm: x.set_anchor('N')
sup = fig.text(0.5, 0.965, '', color='w', ha='center', fontsize=14)
fig.text(0.01, 0.008, 'Left/Right 1 s | PgUp/PgDn 10 s | Space play | 1/2/3 joint copy | g ghost GT | q quit', color='#9aa', fontsize=9)

def field(ax, title):
    ax.clear(); ax.set_facecolor('#2e7d32')
    ax.set_xlim(-FX - .4, FX + .4); ax.set_ylim(-FY - .4, FY + .4); ax.set_aspect('equal'); ax.set_xticks([]); ax.set_yticks([])
    ax.add_patch(Rectangle((-FX, -FY), 2 * FX, 2 * FY, fill=False, ec='w', lw=1.2))
    ax.plot([0, 0], [-FY, FY], c='w', lw=1); ax.add_patch(Circle((0, 0), 0.75, fill=False, ec='w', lw=1))
    for s in (-1, 1):
        ax.add_patch(Rectangle((s * FX - (2 if s > 0 else 0), -1.5 * 1), 2, 3, fill=False, ec='w', lw=.8)) if False else None
    ax.set_title(title, color='w', fontsize=10, pad=3)
    for sp in ax.spines.values(): sp.set_color('#555')

def ghost(ax, g):
    if not state['ghost']: return
    ax.add_patch(Circle((g['bx'], g['by']), 0.16, fc='#dddddd', ec='k', lw=.5, alpha=.55, zorder=3))
    for k in range(1, 7):
        x, y, th = g[f'x{k}'], g[f'y{k}'], g[f'th{k}']
        col = C_TEAM if k <= 3 else C_OPP
        ax.add_patch(Circle((x, y), 0.22, fc='#cfcfcf', ec=col, lw=1.4, alpha=.6, zorder=3))
        ax.plot([x, x + .42 * np.cos(th)], [y, y + .42 * np.sin(th)], c=col, lw=1.8, alpha=.7, zorder=3)
        ax.text(x, y - .45, str(k), color='#eee', fontsize=7, ha='center', alpha=.8, zorder=3)

def wedge(ax, g, k, R=5.5):
    h = np.degrees(g[f'th{k}'] + g[f'hy{k}'])
    ax.add_patch(Wedge((g[f'x{k}'], g[f'y{k}']), R, h - FOV / 2, h + FOV / 2, fc=OBS_COL[k], ec=OBS_COL[k], alpha=.16, lw=.8, zorder=2))

def draw_tracks(ax, tl):
    n = 0
    for name, cl, mx, my, cxx, cxy, cyy in tl:
        C = np.array([[cxx, cxy], [cxy, cyy]]); w, V = np.linalg.eigh(C); w = np.maximum(w, 1e-9)
        ang = np.degrees(np.arctan2(V[1, 1], V[0, 1])); col = C_BALLT if cl == 'ball' else C_ROBT
        ax.add_patch(Ellipse((mx, my), 2 * np.sqrt(CHI2 * w[1]), 2 * np.sqrt(CHI2 * w[0]), angle=ang, fc=col, ec=col, alpha=.16, lw=1.3, zorder=5))
        ax.plot([mx], [my], 'o' if cl == 'ball' else 's', c=col, mec='w', mew=.8, ms=7, zorder=6)
        ax.text(mx + .15, my + .15, name, color='w', fontsize=6.5, zorder=7, bbox=dict(fc=col, ec='none', alpha=.6, pad=.8)); n += 1
    return n

def draw(i):
    t = times[min(max(i, 0), len(times) - 1)]; state['i'] = times.index(t); g = gt[t]
    ball_vis = []
    for k in (1, 2, 3):
        ax = axc[k - 1]; ax.clear(); ax.axis('off')
        p = f'{HERE}/cache/{t:07.2f}_r{k}.jpg'
        if os.path.exists(p): ax.imshow(Image.open(p), interpolation='bicubic', extent=(0, 320, 240, 0))
        ax.set_xlim(0, 320); ax.set_ylim(240, 0)
        ft = ftimes[k]; j = int(np.argmin(np.abs(ft - t))); tf = ft[j]; nd = 0; bl = False
        if abs(tf - t) < 0.2:
            for d in dets.get((k, round(tf, 3)), []):
                u, v, w, h = (float(d[q]) for q in ('u_px', 'v_px', 'w_px', 'h_px')); c = CLS_COL.get(d['class'], 'w')
                ax.add_patch(Rectangle((u - w / 2, v - h / 2), w, h, fill=False, ec=c, lw=1.6)); nd += 1
                ax.text(u - w / 2, v - h / 2 - 2, f"{d['class']} {float(d['conf']):.2f}", color='k', fontsize=7, va='bottom', bbox=dict(fc=c, ec='none', pad=.6, alpha=.85))
                bl |= d['class'] == 'Ball'
        ball_vis.append(bl)
        ax.set_title(f'Robot {k} camera  |  {nd} dets' + ('  | BALL' if bl else ''), color=OBS_COL[k], fontsize=10, pad=3)
        am = axm[k - 1]; tl = trk.get(('base', k, t), [])
        field(am, f'Robot {k} baseline (own KF only)'); ghost(am, g); wedge(am, g, k, 4)
        n = draw_tracks(am, tl); am.text(-FX, FY + .42, f'{n} live tracks', color='w', fontsize=8, va='bottom') if False else None
        am.text(0.01, 0.015, f'{n} live tracks', color='w', fontsize=8, transform=am.transAxes, bbox=dict(fc='k', alpha=.5, ec='none', pad=1))
    c = state['copy']; field(axj, f'JOINT cooperative SRIF  (robot {c} copy; keys 1/2/3)'); ghost(axj, g)
    for k in (1, 2, 3): wedge(axj, g, k, 6)
    for k in (1, 2, 3):
        axj.plot([g[f'x{k}']], [g[f'y{k}']], '^', c=OBS_COL[k], mec='w', ms=9, zorder=8)
    n = draw_tracks(axj, trk.get(('joint', c, t), []))
    axj.text(0.01, 0.01, f'{n} live tracks', color='w', fontsize=10, transform=axj.transAxes, bbox=dict(fc='k', alpha=.5, ec='none', pad=1))
    axj.plot([], [], 'o', c=C_BALLT, label='ball track (+95% ellipse)'); axj.plot([], [], 's', c=C_ROBT, label='robot track')
    axj.plot([], [], 'o', c=C_TEAM, mfc='#cfcfcf', label='GT teammate (ghost)'); axj.plot([], [], 'o', c=C_OPP, mfc='#cfcfcf', label='GT opponent (ghost)')
    axj.legend(loc='upper right', fontsize=7.5, facecolor='#222', labelcolor='w', framealpha=.8)
    sup.set_text(f'real01   t = {t:7.1f} s   ({"PLAYING" if state["play"] else "paused"})   ghost GT: {"on" if state["ghost"] else "off"}')
    fig.canvas.draw_idle()

def step(d): draw(state['i'] + d)
def onkey(e):
    k = e.key
    if k == 'right': step(1)
    elif k == 'left': step(-1)
    elif k == 'pageup': step(10)
    elif k == 'pagedown': step(-10)
    elif k == 'up': step(1)
    elif k == 'down': step(-1)
    elif k == ' ': state['play'] = not state['play']; draw(state['i'])
    elif k in '123': state['copy'] = int(k); draw(state['i'])
    elif k == 'g': state['ghost'] = not state['ghost']; draw(state['i'])
    elif k == 'q': plt.close(fig)
def tick():
    if state['play']:
        if state['i'] < len(times) - 1: step(2)  # times are 0.5 s apart -> advance 1 s
        else: state['play'] = False
def nearest(t): return int(np.argmin([abs(x - t) for x in times]))
for kk in list(plt.rcParams):
    if kk.startswith('keymap.'): plt.rcParams[kk] = []
# 1 s key frames only (images cached on whole seconds)
times = [x for x in times if abs(x - round(x)) < 1e-6]
gt_times = times

if a.png:
    draw(nearest(a.t)); fig.savefig(a.png, dpi=90, facecolor=fig.get_facecolor())
elif a.mp4:
    from matplotlib.animation import FFMpegWriter
    w = FFMpegWriter(fps=4, bitrate=4000)
    with w.saving(fig, a.mp4, 80):
        for i in range(nearest(a.t0), nearest(a.t0 + a.dur) + 1): draw(i); w.grab_frame()
else:
    fig.canvas.mpl_connect('key_press_event', onkey)
    tm = fig.canvas.new_timer(interval=350); tm.add_callback(lambda: (state.update(), None) and None); tm.stop()
    tm = fig.canvas.new_timer(interval=350)
    tm.add_callback(lambda: (step(1) if state['play'] and state['i'] < len(times) - 1 else None)); tm.start()
    draw(nearest(a.t)); plt.show()
