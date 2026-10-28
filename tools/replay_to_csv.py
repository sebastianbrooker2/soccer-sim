#!/usr/bin/env python3
"""Convert Booster sim replay output (manifest.json + chunks/frames_*.jsonl)
or a ROS2-logger CSV into the eval GT CSV: t,id,team,x,y,theta,z,roll,pitch,headYaw,headPitch.
Extra columns (appended after theta): z = trunk height (m), roll/pitch = trunk tilt (rad, ZYX
Euler from baseQuaternion; +pitch = nose down), headYaw/headPitch = joints AAHead_yaw/Head_pitch (rad).
Ball rows: z = ball z, others 0. ROS2 input has no such data -> zeros.

ids: 0 ball; robot1..6 -> 1..6; robot1-3 team1, robot4-6 team2.
Frame: raw world (x fwd, y left) == /team1 frame. theta = yaw from baseQuaternion [w,x,y,z]
(gameControlYaw used if quaternion missing). Output resampled to uniform --dt grid.

Usage: replay_to_csv.py REPLAY_DIR OUT.csv [--dt 0.05]
       replay_to_csv.py --ros2-csv IN.csv OUT.csv [--dt 0.05]
         (IN columns: t,id,x,y,theta  [team optional])
"""
import argparse, csv, glob, json, math, os
import numpy as np

HEADER_OLD = ["t", "id", "team", "x", "y", "theta"]
HEADER = HEADER_OLD + ["z", "roll", "pitch", "headYaw", "headPitch"]
NCOL = 9  # series columns: t,x,y,theta,z,roll,pitch,headYaw,headPitch
ANGLE_COLS = (3, 5, 6, 7, 8)


def quat_yaw(q):
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def quat_roll_pitch(q):
    """ZYX Euler (body->world, [w,x,y,z]): roll about x, pitch about y (+pitch = nose down)."""
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    return roll, pitch


def team_of(i):
    return 0 if i == 0 else (1 if i <= 3 else 2)


def new_series():
    return {i: tuple([] for _ in range(NCOL)) for i in range(7)}


def frames_to_series(frames):
    """frames: iterable of dicts -> dict id -> tuple of NCOL lists
    (t, x, y, theta, z, roll, pitch, headYaw, headPitch)."""
    ser = new_series()
    for f in frames:
        t = float(f["time"])
        bp = f["ball"]["position"]
        bz = bp[2] if len(bp) > 2 else 0.0
        for k, v in zip(ser[0], (t, bp[0], bp[1], 0.0, bz, 0.0, 0.0, 0.0, 0.0)):
            k.append(v)
        for r in f["robots"]:
            name = r["name"]
            if not name.startswith("robot"):
                continue
            i = int(name[5:])
            if not 1 <= i <= 6:
                continue
            p = r.get("basePosition", r.get("gameControlPosition"))
            q = r.get("baseQuaternion")
            if q is not None:
                th = quat_yaw(q)
                roll, pitch = quat_roll_pitch(q)
            else:
                th = float(r.get("gameControlYaw", 0.0))
                roll = pitch = 0.0
            z = p[2] if len(p) > 2 else 0.0
            j = r.get("joints") or {}
            hy = float(j.get("AAHead_yaw", 0.0))
            hp = float(j.get("Head_pitch", 0.0))
            for k, v in zip(ser[i], (t, p[0], p[1], th, z, roll, pitch, hy, hp)):
                k.append(v)
    return ser


def resample(ser, dt):
    ts = [np.asarray(s[0]) for s in ser.values() if len(s[0]) > 1]
    if not ts:
        raise ValueError("no data")
    t0 = max(t[0] for t in ts)
    t1 = min(t[-1] for t in ts)
    n = int(math.floor((t1 - t0) / dt + 1e-9)) + 1
    grid = t0 + np.arange(n) * dt
    rows = []
    out = {}
    for i, cols in ser.items():
        t = np.asarray(cols[0])
        if len(t) < 2:
            continue
        order = np.argsort(t, kind="stable")
        t = t[order]
        t, u = np.unique(t, return_index=True)
        res = []
        for c in range(1, NCOL):
            a = np.asarray(cols[c], dtype=float)[order][u]
            if c in ANGLE_COLS:
                a = np.unwrap(a)  # avoid +-pi wrap in interpolation
            a = np.interp(grid, t, a)
            if c in ANGLE_COLS:
                a = (a + np.pi) % (2 * np.pi) - np.pi
            res.append(a)
        out[i] = res
    for j in range(n):
        for i in sorted(out):
            rows.append([grid[j] - t0, i, team_of(i)] + [a[j] for a in out[i]])
    return rows


def read_replay(d):
    with open(os.path.join(d, "manifest.json")) as fh:
        json.load(fh)  # validated only; chunks are globbed in name order
    for c in sorted(glob.glob(os.path.join(d, "chunks", "frames_*.jsonl"))):
        with open(c) as fh:
            for line in fh:
                line = line.strip()
                if line:
                    yield json.loads(line)


def read_ros2_csv(path):
    ser = new_series()
    with open(path) as fh:
        for r in csv.DictReader(fh):
            i = int(r["id"])
            vals = (r["t"], r["x"], r["y"], r.get("theta", 0), 0, 0, 0, 0, 0)
            for k, v in zip(ser[i], vals):
                k.append(float(v))
    return ser


def write_csv(rows, out):
    with open(out, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(HEADER)
        for r in rows:
            w.writerow([f"{r[0]:.4f}", r[1], r[2]] + [f"{v:.5f}" for v in r[3:]])


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("src")
    ap.add_argument("out")
    ap.add_argument("--dt", type=float, default=0.05)
    ap.add_argument("--ros2-csv", action="store_true")
    a = ap.parse_args(argv)
    ser = read_ros2_csv(a.src) if a.ros2_csv else frames_to_series(read_replay(a.src))
    write_csv(resample(ser, a.dt), a.out)


if __name__ == "__main__":
    main()
