"""v0.2 proxy evaluation (kept for comparability). NOTE: the reference motion is derived from the
evaluated tracker's own filtered positions -> circular; use pseudo_gt_eval.py for before/after."""
import csv
import math
import sys
import collections
rows = list(csv.DictReader(open(sys.argv[1])))
by = collections.defaultdict(list)
for r in rows:
    by[r['id']].append(r)
walk = walk_mov = still = still_mov = 0
herr = []
for tid, rs in by.items():
    rs = [r for r in rs if r['state'] != 'TENTATIVE']
    if len(rs) < 15:
        continue
    t = [float(r['stamp']) for r in rs]
    x = [float(r['x']) for r in rs]
    y = [float(r['y']) for r in rs]
    for k, r in enumerate(rs):
        if float(r['l']) > 1.2 or r['bg'] == '1':
            continue
        a = next((j for j in range(k, -1, -1) if t[k] - t[j] >= 0.5), None)
        b = next((j for j in range(k, len(rs)) if t[j] - t[k] >= 0.5), None)
        if a is None or b is None or t[b] - t[a] > 1.3:
            continue
        dx, dy = x[b] - x[a], y[b] - y[a]
        v = math.hypot(dx, dy) / (t[b] - t[a])
        mov = r['motion'] == 'MOVING'
        if v > 0.4:
            walk += 1
            walk_mov += mov
            if r['heading_valid'] == '1':
                d = float(r['heading']) - math.atan2(dy, dx)
                herr.append(abs(math.degrees(math.atan2(math.sin(d), math.cos(d)))))
        elif v < 0.1:
            still += 1
            still_mov += mov
herr.sort()
walk_pct = 100 * walk_mov / max(walk, 1)
still_pct = 100 * still_mov / max(still, 1)
print(f"walking frames (v>0.4): {walk}, labelled MOVING: {walk_mov} ({walk_pct:.0f} %)")
print(f"still frames (v<0.1):  {still}, labelled MOVING: {still_mov} ({still_pct:.1f} %)")
if herr:
    print(f"heading error vs. actual walking direction: median {herr[len(herr) // 2]:.1f} deg, "
          f"p90 {herr[int(0.9 * len(herr))]:.1f} deg ({len(herr)} samples)")
