#!/usr/bin/env python3
"""Key-frame viewer for match real01: cameras+YOLO dets, per-robot baseline minimaps, joint SRIF minimap.
Run:  ~/miniconda3/bin/python tools/viz/viewer.py [--t 120] [--png out.png | --mp4 out.mp4 --t0 100 --dur 60]
Keys: Left/Right +-1 s | PageUp/PageDown +-10 s | Space play/pause | 1/2/3 joint copy | g ghost GT | q quit"""
import argparse, csv, os, collections
import numpy as np
ap = argparse.ArgumentParser()
ap.add_argument('--t', type=float, default=100); ap.add_argument('--png'); ap.add_argument('--mp4')
ap.add_argument('--t0', type=float, default=0); ap.add_argument('--dur', type=float, default=60)
ap.add_argument('--copy', type=int, default=1); ap.add_argument('--fs', type=float, default=2.0, help='text size multiplier (default 2.0)'); ap.add_argument('--topdown', action='store_true', help="use the straight-down bird's-eye render instead of the broadcast view")
a = ap.parse_args()
FS = a.fs
import matplotlib
if a.png or a.mp4: matplotlib.use('Agg')
else:
    try: matplotlib.use('TkAgg')
    except Exception: pass
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse, Wedge, Rectangle, Circle, Polygon, RegularPolygon
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

fig = plt.figure(figsize=(19, 14), facecolor='#15181d')
gs = fig.add_gridspec(4, 4, width_ratios=[1, 1, 1, 1.25], height_ratios=[1, 1, 1, 1], left=0.045, right=0.995, top=0.95, bottom=0.035, wspace=0.04, hspace=0.09)
axc = [fig.add_subplot(gs[0, k]) for k in range(3)]
axm = [fig.add_subplot(gs[1, k]) for k in range(3)]
axs = [fig.add_subplot(gs[2, k]) for k in range(3)]
axw = [fig.add_subplot(gs[3, k]) for k in range(3)]
axj = fig.add_subplot(gs[1:, 3])
axt = fig.add_subplot(gs[0, 3])
for x in axc: x.set_anchor('S')
for x in axm + axs + axw: x.set_anchor('C')
axj.set_anchor('N'); axt.set_anchor('S')
sup = fig.text(0.5, 0.975, '', color='w', ha='center', fontsize=14*FS)
for yy, lab in ((0.86, 'YOLO view'), (0.64, 'NUbots'), (0.40, 'Own SRIF'), (0.16, 'Raw meas.')):
    fig.text(0.008, yy, lab, color='w', rotation=90, ha='left', va='center', fontsize=11*FS)
fig.text(0.01, 0.006, 'Left/Right 1 s | PgUp/PgDn 10 s | Space play | 1/2/3 joint copy | g ghost | q quit', color='#9aa', fontsize=7*FS)

def vert(ax): return ax is axj          # joint panel is drawn vertically: x (long axis) up, +y to the left
def tx(ax, x, y): return (-y, x) if vert(ax) else (x, y)
def tth(ax, th): return th + np.pi / 2 if vert(ax) else th

def field(ax, title):
    ax.clear(); ax.set_facecolor('#2e7d32')
    if vert(ax): ax.set_xlim(-FY - .4, FY + .4); ax.set_ylim(-FX - .4, FX + .4)
    else: ax.set_xlim(-FX - .4, FX + .4); ax.set_ylim(-FY - .4, FY + .4)
    ax.set_aspect('equal'); ax.set_xticks([]); ax.set_yticks([])
    corners = [tx(ax, *p) for p in ((-FX, -FY), (FX, -FY), (FX, FY), (-FX, FY))]
    ax.add_patch(Polygon(corners, closed=True, fill=False, ec='w', lw=1.2))
    (x0, y0), (x1, y1) = tx(ax, 0, -FY), tx(ax, 0, FY); ax.plot([x0, x1], [y0, y1], c='w', lw=1)
    ax.add_patch(Circle((0, 0), 0.75, fill=False, ec='w', lw=1))
    for s_ in (-1, 1):   # goals (home defends x=-7)
        (gx0, gy0), (gx1, gy1) = tx(ax, s_ * FX, -1.3), tx(ax, s_ * FX, 1.3)
        ax.plot([gx0, gx1], [gy0, gy1], c='#ffe082' if s_ < 0 else '#ffffff', lw=4, solid_capstyle='butt', alpha=.9, zorder=1)
    ax.set_title(title, color='w', fontsize=10*FS, pad=3)
    for sp in ax.spines.values(): sp.set_color('#555')

def ball_glyph(ax, x, y, alpha=1.0, z=3, r=0.17):
    x, y = tx(ax, x, y)
    ax.add_patch(Circle((x, y), r, fc='w', ec='k', lw=.8, alpha=alpha, zorder=z))
    ang = np.radians(90 + 72 * np.arange(5))
    ax.add_patch(RegularPolygon((x, y), 5, radius=r * .42, orientation=0, fc='k', ec='k', alpha=alpha, zorder=z + .1))
    for a_ in ang:   # seams from the pentagon to the rim
        ax.plot([x + r * .42 * np.cos(a_), x + r * .98 * np.cos(a_)], [y + r * .42 * np.sin(a_), y + r * .98 * np.sin(a_)], c='k', lw=.7, alpha=alpha, zorder=z + .1)

def robot_glyph(ax, x, y, th, col, alpha=.75, z=3, label=None):
    """top-down humanoid: shoulders (wide ellipse across the heading), head circle, nose pointing where it faces"""
    cx, cy = tx(ax, x, y); h = tth(ax, th); c, s_ = np.cos(h), np.sin(h)
    ax.add_patch(Ellipse((cx, cy), 0.30, 0.62, angle=np.degrees(h), fc=col, ec='w', lw=.9, alpha=alpha, zorder=z))
    ax.add_patch(Circle((cx + .02 * c, cy + .02 * s_), 0.115, fc='#f0f0f0', ec=col, lw=1, alpha=min(1, alpha + .15), zorder=z + .1))
    ax.add_patch(Polygon([(cx + .30 * c, cy + .30 * s_), (cx + .13 * c - .07 * s_, cy + .13 * s_ + .07 * c), (cx + .13 * c + .07 * s_, cy + .13 * s_ - .07 * c)],
                         closed=True, fc='w', ec='none', alpha=alpha, zorder=z + .1))
    if label: ax.text(cx, cy - .50, label, color='#eee', fontsize=7*FS, ha='center', alpha=.85, zorder=z + .2)

def ghost(ax, g):
    if not state['ghost']: return
    ball_glyph(ax, g['bx'], g['by'], alpha=.6)
    for k in range(1, 7):
        robot_glyph(ax, g[f'x{k}'], g[f'y{k}'], g[f'th{k}'], C_TEAM if k <= 3 else C_OPP, alpha=.55, label=str(k))

FOV_LEN = 2.5   # m, just a short indicator of the edge directions
def fov_lines(ax, g, k, L=FOV_LEN):
    """two rays at the horizontal FOV edges - no range cut-off, so no cone"""
    h = g[f'th{k}'] + g[f'hy{k}']; x, y = g[f'x{k}'], g[f'y{k}']
    for sgn in (-1, 1):
        a_ = h + sgn * np.radians(FOV / 2)
        p0 = tx(ax, x, y); p1 = tx(ax, x + L * np.cos(a_), y + L * np.sin(a_))
        ax.plot([p0[0], p1[0]], [p0[1], p1[1]], c=OBS_COL[k], lw=1.1, ls='-', alpha=.9, zorder=2, clip_on=True)

# NUbots radii (NUbots_K1): ball = BallRadius 0.07 m (module/support/configuration/SoccerConfig/data/config/FieldDescription.yaml);
# robot = association_distance 0.5 m (module/localisation/RobotLocalisation/data/config/RobotLocalisation.yaml), the only
# radius RobotLocalisation defines (a detection within 0.5 m of a track is associated to it).
R_BALL_NU, R_ROBOT_NU = 0.07, 0.5
def draw_circles(ax, tl):
    n = 0
    for name, cl, mx, my, *_ in tl:
        px, py = tx(ax, mx, my); ball = cl == 'ball'
        ax.add_patch(Circle((px, py), R_BALL_NU if ball else R_ROBOT_NU, fc='#ff8c00' if ball else '#e02020', ec='w', lw=.6, alpha=1 if ball else .72, zorder=6 if ball else 5))
        if ball: ax.text(px + .12, py + .12, 'ball', color='w', fontsize=6.5*FS, zorder=7, clip_on=True, bbox=dict(fc='#ff8c00', ec='none', alpha=.7, pad=.8))
        n += 1
    return n

def draw_tracks(ax, tl):
    n = 0
    for name, cl, mx, my, cxx, cxy, cyy in tl:
        C = np.array([[cxx, cxy], [cxy, cyy]]); w, V = np.linalg.eigh(C); w = np.maximum(w, 1e-9)
        ang = np.degrees(np.arctan2(V[1, 1], V[0, 1])) + (90 if vert(ax) else 0); col = C_BALLT if cl == 'ball' else C_ROBT
        px, py = tx(ax, mx, my)
        ax.add_patch(Ellipse((px, py), 2 * np.sqrt(CHI2 * w[1]), 2 * np.sqrt(CHI2 * w[0]), angle=ang, fill=False, ec=col, lw=1.4, zorder=5))
        ax.plot([px], [py], '+', c=col, mew=2.0, ms=11, zorder=6)
        ax.text(px + .15, py + .15, name, color='w', fontsize=6.5*FS, zorder=7, bbox=dict(fc=col, ec='none', alpha=.6, pad=.8)); n += 1
    return n

C_RAWB, C_RAWR = '#ff8c00', '#00e5ff'
def draw_raw(ax, tl):
    n = 0
    for name, cl, mx, my, cxx, cxy, cyy in tl:
        C = np.array([[cxx, cxy], [cxy, cyy]]); w, V = np.linalg.eigh(C); w = np.maximum(w, 1e-9)
        ang = np.degrees(np.arctan2(V[1, 1], V[0, 1])) + (90 if vert(ax) else 0); col = C_RAWB if cl == 'ball' else C_RAWR
        px, py = tx(ax, mx, my)
        ax.add_patch(Ellipse((px, py), 2 * np.sqrt(CHI2 * w[1]), 2 * np.sqrt(CHI2 * w[0]), angle=ang, fill=False, ec=col, lw=1.0, alpha=.9, zorder=5))
        ax.plot([px], [py], 'x', c=col, mew=1.6, ms=6, zorder=6); n += 1
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
                ax.text(u - w / 2, v - h / 2 - 2, f"{d['class']} {float(d['conf']):.2f}", color='k', fontsize=5.2*FS, va='bottom', bbox=dict(fc=c, ec='none', pad=.6, alpha=.85))
                bl |= d['class'] == 'Ball'
        ball_vis.append(bl)
        ax.set_title(f'R{k}  {nd} dets' + ('  BALL' if bl else ''), color=OBS_COL[k], fontsize=10*FS, pad=3)
        am = axm[k - 1]
        field(am, f'R{k} NUbots'); ghost(am, g); fov_lines(am, g, k)
        n = draw_circles(am, trk.get(('base', k, t), []))
        am.text(0.01, 0.015, f'{n} live tracks', color='w', fontsize=8*FS, transform=am.transAxes, bbox=dict(fc='k', alpha=.5, ec='none', pad=1))
        asx = axs[k - 1]
        field(asx, f'R{k} own SRIF'); ghost(asx, g); fov_lines(asx, g, k)
        n = draw_tracks(asx, trk.get(('solo', k, t), []))
        asx.text(0.01, 0.015, f'{n} live tracks', color='w', fontsize=8*FS, transform=asx.transAxes, bbox=dict(fc='k', alpha=.5, ec='none', pad=1))
        aw = axw[k - 1]
        field(aw, f'R{k} raw'); ghost(aw, g); fov_lines(aw, g, k)
        tl = trk.get(('raw', k, t), []); n = draw_raw(aw, tl)
        nb = sum(1 for q in tl if q[1] == 'ball')
        aw.text(0.01, 0.015, f'{nb} ball + {n - nb} robot estimates' if n else 'nothing in view', color='w', fontsize=8*FS, transform=aw.transAxes, bbox=dict(fc='k', alpha=.5, ec='none', pad=1))
    axt.clear(); axt.axis('off')
    if a.topdown:
        pt = f'{HERE}/cache_top/{t:07.2f}.jpg'
        if os.path.exists(pt): axt.imshow(Image.open(pt), interpolation='bicubic', extent=(-FX - 1.1, FX + 1.1, -FY - 0.8, FY + 0.8))
        axt.set_xlim(-FX - 1.1, FX + 1.1); axt.set_ylim(-FY - 0.8, FY + 0.8); axt.set_aspect('equal')
        axt.set_title("Bird's-eye (sim render)", color='w', fontsize=10*FS, pad=3)
    else:
        pt = f'{HERE}/cache_broadcast/{t:07.2f}.jpg'
        if os.path.exists(pt): axt.imshow(Image.open(pt), interpolation='bicubic')
        axt.set_aspect('equal')
        axt.set_title('Broadcast view (sim render)', color='w', fontsize=10*FS, pad=3)
    c = state['copy']; field(axj, f'JOINT SRIF (R{c} copy)'); ghost(axj, g)
    for k in (1, 2, 3): fov_lines(axj, g, k)
    for k in (1, 2, 3):
        px_, py_ = tx(axj, g[f'x{k}'], g[f'y{k}']); axj.plot([px_], [py_], 'o', mfc='none', mec=OBS_COL[k], mew=2, ms=17, zorder=8)
    n = draw_tracks(axj, trk.get(('joint', c, t), []))
    axj.text(0.01, 0.01, f'{n} live tracks', color='w', fontsize=10*FS, transform=axj.transAxes, bbox=dict(fc='k', alpha=.5, ec='none', pad=1))
    axj.plot([], [], '+', c=C_BALLT, mew=2, ms=10, label='ball track (+, 95% ellipse)'); axj.plot([], [], '+', c=C_ROBT, mew=2, ms=10, label='robot track (+, 95% ellipse)')
    axj.plot([], [], 'o', c=C_TEAM, mfc='#cfcfcf', label='GT teammate (ghost)'); axj.plot([], [], 'o', c=C_OPP, mfc='#cfcfcf', label='GT opponent (ghost)')
    axj.legend(loc='upper center', bbox_to_anchor=(0.5, -0.01), ncol=1, fontsize=7.5*FS, facecolor='#222', labelcolor='w', framealpha=.8, borderaxespad=0)
    sup.set_text(f't = {t:.0f} s  ({"PLAY" if state["play"] else "paused"})  ghost {"on" if state["ghost"] else "off"}')
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
