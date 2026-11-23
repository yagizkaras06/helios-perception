"""Pseudo ground truth for a real recording with a STATIONARY sensor (no human labels).

Independent of the tracker under test:
  1. whole-recording BEV occupancy (0.15 m cells, 1.0-2.1 m above ground band is not required;
     points 0.25..2.1 m above the floor): cells occupied in >= STATIC_FRAC of all frames = static
  2. per frame: foreground points -> connected components (0.3 m linking)
  3. person-sized components (height span >= 0.7 m, top >= 1.0 m above floor, footprint <= 1.3 m)
  4. offline linking across frames (gate grows with the gap), gaps <= 1.5 s bridged later
  5. non-causal smoothing (centred window) -> position, speed, heading; frames inside a bridged
     gap are flagged 'interp' (person not segmented: occluded / merged / missed)
Output CSV: stamp, traj, x, y, observed(0/1), speed, heading, n_points, zmax, clean, raw_x, raw_y
and occ_frac.npy next to it (whole-recording BEV occupancy, used by pseudo_gt_eval.py for the
static-scene false-MOVING rate).
The result is a REFERENCE PROPOSAL (no human labels); it must be checked visually before use.

usage: pseudo_gt.py <recording.npz from load_bag.py> <out/gt.csv>"""
import csv
import math
import os
import sys

import numpy as np

STATIC_FRAC = 0.6
C = 0.15
R = 15.0
N = int(2 * R / C)
d = np.load(sys.argv[1])
st = d['stamps']
lens = d['lens']
xyz = d['xyz'].astype(np.float32)
offs = np.concatenate([[0], np.cumsum(lens)])
# 3D voxel occupancy over the whole recording (a person above a desk is not "desk")
CZ = 0.15
ZMIN = -0.65
NZ = int((1.2 - ZMIN) / CZ) + 1


def vox(p):
    ij = ((p[:, :2] + R) / C).astype(int)
    kz = ((p[:, 2] - ZMIN) / CZ).astype(int)
    return (ij[:, 0] * N + ij[:, 1]) * NZ + kz


occ3 = np.zeros(N * N * NZ, np.int32)
for k in range(len(st)):
    p = xyz[offs[k]:offs[k + 1]]
    r = np.hypot(p[:, 0], p[:, 1])
    m = (r > 0.6) & (r < R - 0.1) & (p[:, 2] > ZMIN) & (p[:, 2] < 1.2)
    v, c = np.unique(vox(p[m]), return_counts=True)
    occ3[v[c >= 2]] += 1
static3 = occ3 >= STATIC_FRAC * len(st)

# BEV occupancy fraction (cell occupied by >= 2 points in a frame) for pseudo_gt_eval.py
occ2 = np.zeros((N, N), np.int32)
for k in range(len(st)):
    p = xyz[offs[k]:offs[k + 1]]
    r = np.hypot(p[:, 0], p[:, 1])
    m = (r > 0.6) & (r < R - 0.1) & (p[:, 2] > -0.65) & (p[:, 2] < 1.2)
    ij = ((p[m, :2] + R) / C).astype(int)
    g = np.zeros((N, N), np.int16)
    np.add.at(g, (ij[:, 0], ij[:, 1]), 1)
    occ2 += g >= 2
np.save(os.path.join(os.path.dirname(os.path.abspath(sys.argv[2])), 'occ_frac.npy'), occ2 / len(st))


def components(cells):
    """8-connected components with 1-cell gap tolerance (radius 2 cells = 0.3 m)."""
    idx = {c: k for k, c in enumerate(cells)}
    parent = list(range(len(cells)))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a
    for k, (i, j) in enumerate(cells):
        for di in range(-2, 3):
            for dj in range(-2, 3):
                o = idx.get((i + di, j + dj))
                if o is not None and o != k:
                    ra, rb = find(k), find(o)
                    if ra != rb:
                        parent[ra] = rb
    return [find(k) for k in range(len(cells))]


allblobs = []
dets = []   # per frame list of (x, y, n, zspan)
for k in range(len(st)):
    p = xyz[offs[k]:offs[k + 1]]
    r = np.hypot(p[:, 0], p[:, 1])
    m = (r > 0.6) & (r < R - 0.1) & (p[:, 2] > -0.65) & (p[:, 2] < 1.2)
    p = p[m]
    fg = ~static3[vox(p)]
    p = p[fg]
    ij = ((p[:, :2] + R) / C).astype(int)
    cells, inv = np.unique(ij, axis=0, return_inverse=True)
    inv = inv.reshape(-1)
    fd = []
    blobs = []
    if len(cells):
        lab = np.array(components([tuple(c) for c in cells]))[inv]
        for lab_id in np.unique(lab):
            q = p[lab == lab_id]
            if len(q) >= 40:
                blobs.append((float(np.median(q[:, 0])), float(np.median(q[:, 1]))))
            if len(q) < 150 or math.hypot(*np.median(q[:, :2], 0)) > 8.0:
                continue
            zmax, zspan = q[:, 2].max(), q[:, 2].max() - q[:, 2].min()
            ext = np.ptp(q[:, :2], axis=0).max()
            rng = math.hypot(*np.median(q[:, :2], 0))
            ztop = min(0.1, rng * math.tan(math.radians(15.0)) - 0.15)   # vertical FOV limit
            if zmax >= ztop and zspan >= 0.45 and ext <= 1.3 and q[:, 2].min() < -0.2:
                # BEV centre = mid of extent in a 2D sense (robust): median of points
                fd.append((float(np.median(q[:, 0])), float(
                    np.median(q[:, 1])), len(q), float(zmax)))
    dets.append(fd)
    allblobs.append(blobs)

# ---- offline linking: greedy nearest within gate, gaps up to MAX_GAP ---
MAX_GAP = 1.5
ISO = 1.5
trajs = []   # each: list of (frame, x, y, n)
active = []
for k, fd in enumerate(dets):
    used = set()
    for tr in sorted(active, key=lambda t: -len(t)):
        fk, x, y = tr[-1][:3]
        gap = st[k] - st[fk]
        gate = 0.5 + 1.5 * gap
        best, bd = None, gate
        for j, (dx, dy, n, _) in enumerate(fd):
            if j in used:
                continue
            dd = math.hypot(dx - x, dy - y)
            if dd < bd:
                best, bd = j, dd
        if best is not None:
            used.add(best)
            tr.append((k, fd[best][0], fd[best][1], fd[best][2], fd[best][3]))
    for j, (dx, dy, n, _) in enumerate(fd):
        if j not in used:
            t = [(k, dx, dy, n, _)]
            trajs.append(t)
            active.append(t)
    active = [t for t in active if st[k] - st[t[-1][0]] <= MAX_GAP]

long_trajs = [t for t in trajs if st[t[-1][0]] - st[t[0][0]] >= 2.0 and len(t) >= 15]
print('candidate frames with >=1 det:', sum(1 for f in dets if f), '/', len(dets),
      ' multi-det frames:', sum(1 for f in dets if len(f) > 1))
print('trajectories >=2 s:', len(long_trajs))
for t in long_trajs:
    dur = st[t[-1][0]] - st[t[0][0]]
    print(f"  frames {t[0][0]}-{t[-1][0]}  ({dur:.1f} s)  observed {len(t)}  "
          f"mean pts {np.mean([a[3] for a in t]):.0f}")

# ---- fill + smooth -----------------------------------------------------
with open(sys.argv[2], 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['stamp', 'traj', 'x', 'y', 'observed', 'speed',
               'heading', 'n_points', 'zmax', 'clean', 'raw_x', 'raw_y'])
    for ti, t in enumerate(long_trajs):
        fr = np.array([a[0] for a in t])
        ts = st[fr]
        X = np.array([a[1] for a in t])
        Y = np.array([a[2] for a in t])
        allf = np.arange(fr[0], fr[-1] + 1)
        ta = st[allf]
        xi = np.interp(ta, ts, X)
        yi = np.interp(ta, ts, Y)
        obs = np.isin(allf, fr)
        # centred smoothing (+-0.25 s) then centred difference over +-0.5 s

        def smooth(v, h):
            out = np.empty_like(v)
            for i in range(len(v)):
                m = np.abs(ta - ta[i]) <= h
                out[i] = v[m].mean()
            return out
        xs, ys = smooth(xi, 0.25), smooth(yi, 0.25)
        npt = {a[0]: a[3] for a in t}
        zmx = {a[0]: a[4] for a in t}
        for i in range(len(allf)):
            a = np.searchsorted(ta, ta[i] - 0.5)
            b = np.searchsorted(ta, ta[i] + 0.5, 'right') - 1
            if ta[b] - ta[a] < 0.6:
                sp, hd = float('nan'), float('nan')
            else:
                dx, dy = xs[b] - xs[a], ys[b] - ys[a]
                sp = math.hypot(dx, dy) / (ta[b] - ta[a])
                hd = math.atan2(dy, dx)
            # clean: no other non-static blob within ISO m of this object in this frame
            others = [b for b in allblobs[allf[i]] if math.hypot(b[0] - xi[i], b[1] - yi[i]) > 0.6]
            clean = int(all(math.hypot(b[0] - xi[i], b[1] - yi[i]) > ISO for b in others))
            w.writerow([f"{ta[i]:.6f}", ti, f"{xs[i]:.3f}", f"{ys[i]:.3f}", int(obs[i]),
                        f"{sp:.3f}", f"{hd:.4f}", npt.get(allf[i], 0),
                        f"{zmx.get(allf[i], float('nan')):.2f}",
                        clean, f"{xi[i]:.3f}", f"{yi[i]:.3f}"])
