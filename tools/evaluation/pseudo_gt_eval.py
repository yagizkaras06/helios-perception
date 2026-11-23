"""Tracker evaluation against the pseudo ground truth of a real recording (gt_walk.csv).

Only CLEAN pseudo-GT frames are used (no other non-static blob within 1.5 m), because the offline
reference itself cannot resolve merges of two dynamic objects; those cases are evaluated on the
synthetic scenarios with exact ground truth. A GT trajectory is split into sub-trajectories at
non-clean frames, so a reference linking error can never be counted as a tracker ID switch.

Matching: per frame, GT object <-> nearest non-TENTATIVE track within MATCH_DIST (greedy by
distance; at most one GT object per frame in the clean set near any track).

usage: pseudo_gt_eval.py gt.csv tracks.csv [--json out.json]
"""
import csv
import collections
import json
import math
import sys

MATCH_DIST = 0.8      # [m] person-sized objects, box centre vs. point median
WALK = 0.4            # [m/s] GT speed above -> walking frame
STILL = 0.1           # [m/s] GT speed below -> still frame
ONSET_STILL_S = 0.7   # still this long before an onset
ONSET_DISP = 0.15     # [m] onset = first raw GT position this far from the rest position ...
ONSET_CONFIRM = 0.5   # [m] ... that reaches this displacement within ONSET_WINDOW
ONSET_WINDOW = 1.5    # [s]
ONSET_MAX_LAT = 3.0   # [s] no MOVING within this -> onset missed
STATIC_TIME = 10.0    # [s] static background rule of the system
NEAR = 1.0            # [m] sensor-mount zone (inside the vehicle footprint on a car)
DEBUG = '--debug' in sys.argv


def load(path):
    return list(csv.DictReader(open(path)))


def main():
    gt_rows, tr_rows = load(sys.argv[1]), load(sys.argv[2])
    out_json = sys.argv[sys.argv.index('--json') + 1] if '--json' in sys.argv else None
    tracks = collections.defaultdict(list)
    tracks_all = collections.defaultdict(list)
    global T0
    T0 = float(gt_rows[0]['stamp'])
    for r in tr_rows:
        tracks_all[round(float(r['stamp']), 3)].append(r)
        if r['state'] != 'TENTATIVE':
            tracks[round(float(r['stamp']), 3)].append(r)
    gt_by_frame = collections.defaultdict(list)
    for r in gt_rows:
        gt_by_frame[round(float(r['stamp']), 3)].append(r)

    # ---- matching (all GT frames; clean flag used later) ---------------------------------------
    match = {}            # (stamp, traj) -> track row or None
    near_gt_tracks = set()  # (stamp, id) of tracks within 1.2 m of ANY pseudo-GT object
    for key, gts in gt_by_frame.items():
        cands = []
        for g in gts:
            gx, gy = float(g['raw_x']), float(g['raw_y'])
            for t in tracks.get(key, []):
                d = math.hypot(float(t['x']) - gx, float(t['y']) - gy)
                if d < 1.2:
                    near_gt_tracks.add((key, t['id']))
                if d < MATCH_DIST:
                    cands.append((d, g['traj'], t['id'], t))
        cands.sort(key=lambda c: c[0])
        used_g, used_t = set(), set()
        for d, gtid, tid, t in cands:
            if gtid in used_g or tid in used_t:
                continue
            used_g.add(gtid)
            used_t.add(tid)
            match[(key, gtid)] = t
        for g in gts:
            match.setdefault((key, g['traj']), None)

    # ---- sub-trajectories over clean frames ----------------------------------------------------
    by_traj = collections.defaultdict(list)
    for r in gt_rows:
        by_traj[r['traj']].append(r)
    subs = []
    for rs in by_traj.values():
        cur = []
        for r in rs:
            if r['clean'] == '1':
                cur.append(r)
            elif cur:
                subs.append(cur)
                cur = []
        if cur:
            subs.append(cur)
    subs = [s for s in subs if len(s) >= 10]
    # GT objects still for >= STATIC_TIME are part of the static background by design (v0.2 rule)
    absorbed = set()
    for rs in by_traj.values():
        obs = [(float(r['stamp']), float(r['raw_x']), float(r['raw_y']))
               for r in rs if r['observed'] == '1']
        for r in rs:
            t = float(r['stamp'])
            win = [o for o in obs if t - STATIC_TIME <= o[0] <= t]
            if not win or win[-1][0] - win[0][0] < STATIC_TIME - 0.5:
                continue
            mx = sorted(o[1] for o in win)[len(win) // 2]
            my = sorted(o[2] for o in win)[len(win) // 2]
            dd = sorted(math.hypot(o[1] - mx, o[2] - my) for o in win)
            if dd[int(0.9 * (len(dd) - 1))] <= 0.4:
                absorbed.add((round(t, 3), r['traj']))
    n_absorbed = 0
    near_n = near_hit = 0

    n_gt = n_matched = n_updated = 0
    n_interp = n_interp_cov = 0
    idsw = frag = 0
    idtp = idfn = 0
    assigned_ids = []
    walk = walk_mov = still = still_mov = 0
    herr = []
    onsets = []
    for s in subs:
        prev_id = None
        was_matched = False
        id_count = collections.Counter()
        for r in s:
            key = round(float(r['stamp']), 3)
            t = match.get((key, r['traj']))
            if (key, r['traj']) in absorbed:
                n_absorbed += 1
                continue
            if math.hypot(float(r['raw_x']), float(r['raw_y'])) < NEAR:
                if r['observed'] == '1':
                    near_n += 1
                    near_hit += t is not None
                continue
            if r['observed'] == '0':
                n_interp += 1
                n_interp_cov += t is not None or any(
                    math.hypot(float(q['x']) - float(r['raw_x']),
                               float(q['y']) - float(r['raw_y'])) < 1.2
                    for q in tracks.get(key, []))
                continue
            n_gt += 1
            if t is None and DEBUG:
                rx, ry = float(r['raw_x']), float(r['raw_y'])
                near = sorted(
                    (round(math.hypot(float(q['x']) - rx, float(q['y']) - ry), 2), q['id'],
                     q['state'][:4], q['bg'], q['l'][:4]) for q in tracks_all.get(key, []))[:2]
                print(
                    f"MISS t={
                        float(
                            r['stamp']) - T0:.1f} traj={
                        r['traj']} gt=({
                        float(
                            r['raw_x']):.2f},{
                            float(
                                r['raw_y']):.2f}) v={
                                    r['speed']} n={
                                        r['n_points']}", near)
            if t is None:
                was_matched = False if prev_id is not None else was_matched
                continue
            n_matched += 1
            n_updated += t['updated'] == '1'
            id_count[t['id']] += 1
            if prev_id is not None and t['id'] != prev_id:
                idsw += 1
            if prev_id is not None and not was_matched:
                frag += 1
            prev_id = t['id']
            was_matched = True
            sp = float(r['speed'])
            mov = t['motion'] == 'MOVING'
            if sp > WALK:
                walk += 1
                walk_mov += mov
                if t['heading_valid'] == '1':
                    d = float(t['heading']) - float(r['heading'])
                    herr.append(abs(math.degrees(math.atan2(math.sin(d), math.cos(d)))))
            elif sp < STILL:
                still += 1
                still_mov += mov
        if id_count:
            best_id, best_n = id_count.most_common(1)[0]
            assigned_ids.append((best_id, s[0]['stamp'], s[-1]['stamp']))
            idtp += best_n
        idfn += sum(
            1 for r in s if r['observed'] == '1' and (
                round(
                    float(
                        r['stamp']), 3), r['traj']) not in absorbed and math.hypot(
                float(
                    r['raw_x']), float(
                    r['raw_y'])) >= NEAR) - (
            id_count.most_common(1)[0][1] if id_count else 0)

        # ---- motion onset events in this sub-trajectory (raw observed positions) ---------------
        ts = [float(r['stamp']) for r in s]
        k = 0
        while k < len(s):
            r = s[k]
            sp = float(r['speed'])
            if not (sp < STILL * 1.5):
                k += 1
                continue
            # still segment [k, e)
            e = k
            while e < len(s) and float(s[e]['speed']) < STILL * 1.5:
                e += 1
            if ts[min(e, len(s) - 1)] - ts[k] < ONSET_STILL_S or e >= len(s):
                k = e + 1
                continue
            rest = [(float(q['raw_x']), float(q['raw_y'])) for q in s[k:e] if q['observed'] == '1']
            if len(rest) < 5:
                k = e + 1
                continue
            rx = sorted(p[0] for p in rest)[len(rest) // 2]
            ry = sorted(p[1] for p in rest)[len(rest) // 2]
            onset = None
            for j in range(e - 3 if e - 3 > k else k, len(s)):
                q = s[j]
                if q['observed'] != '1':
                    continue
                dd = math.hypot(float(q['raw_x']) - rx, float(q['raw_y']) - ry)
                if dd > ONSET_DISP:
                    # confirm: reaches ONSET_CONFIRM within the window
                    ok = any(
                        math.hypot(
                            float(
                                s[m]['raw_x']) -
                            rx,
                            float(
                                s[m]['raw_y']) -
                            ry) > ONSET_CONFIRM for m in range(
                            j,
                            len(s)) if ts[m] -
                        ts[j] <= ONSET_WINDOW and s[m]['observed'] == '1')
                    if ok:
                        onset = j
                    break
            if onset is None:
                k = e + 1
                continue
            t0 = ts[onset]
            lat = None
            # tracked & classified MOVING
            for m in range(onset, len(s)):
                if ts[m] - t0 > ONSET_MAX_LAT:
                    break
                tm = match.get((round(ts[m], 3), s[m]['traj']))
                if tm is not None and tm['motion'] == 'MOVING':
                    lat = ts[m] - t0
                    break
            onsets.append({'t': round(t0 - float(gt_rows[0]['stamp']), 2),
                           'latency_ms': None if lat is None else round(lat * 1000)})
            # skip to the end of this movement
            m = onset
            while m < len(s) and float(s[m]['speed']) > STILL * 1.5:
                m += 1
            k = m

    # ---- walking-segment latency: run of GT speed > WALK lasting >= 1 s, preceded by >= 0.5 s
    # below WALK, track not MOVING at its start; latency = first MOVING - segment start ----------
    seg_lat = []
    for s_ in subs:
        ts = [float(r['stamp']) for r in s_]
        sp = [float(r['speed']) for r in s_]
        k = 0
        while k < len(s_):
            if not sp[k] > WALK:
                k += 1
                continue
            e = k
            while e < len(s_) and sp[e] > WALK:
                e += 1
            pre = [j for j in range(k) if ts[k] - ts[j] <= 0.5]
            if ts[e - 1] - ts[k] >= 1.0 and pre and ts[k] - \
                    ts[pre[0]] >= 0.45 and all(sp[j] <= WALK for j in pre):
                m0 = match.get((round(ts[k], 3), s_[k]['traj']))
                if m0 is None or m0['motion'] != 'MOVING':
                    lat = None
                    for m in range(k, e):
                        tm = match.get((round(ts[m], 3), s_[m]['traj']))
                        if tm is not None and tm['motion'] == 'MOVING':
                            lat = ts[m] - ts[k]
                            break
                    seg_lat.append(None if lat is None else round(lat * 1000))
            k = e
    # IDFP: frames of the assigned tracker ids inside the sub-trajectory time span not matched to it
    idfp = 0
    for tid, a, b in assigned_ids:
        a, b = float(a), float(b)
        tot = sum(1 for key, ts_ in tracks.items() if a <= key <= b for t in ts_ if t['id'] == tid)
        idfp += max(0, tot - sum(1 for s in subs for r in s if a <= float(r['stamp']) <= b and (
            match.get((round(float(r['stamp']), 3), r['traj'])) or {}).get('id') == tid))
    idf1 = 2 * idtp / max(2 * idtp + idfp + idfn, 1)

    # false MOVING on the static scene: tracks whose centre lies in cells occupied in >= 60 % of
    # all frames of the recording (whole-bag BEV occupancy, independent of the tracker) and that are
    # not near any dynamic pseudo-GT object
    import numpy as np
    import os
    frac = np.load(os.path.join(os.path.dirname(os.path.abspath(sys.argv[1])), 'occ_frac.npy'))
    C, R = 0.15, 15.0
    bg = bg_mov = 0
    bg_ids = collections.Counter()
    for key, ts_ in tracks.items():
        for t in ts_:
            if (key, t['id']) in near_gt_tracks:
                continue
            i, j = int((float(t['x']) + R) / C), int((float(t['y']) + R) / C)
            if not (1 <= i < frac.shape[0] - 1 and 1 <= j < frac.shape[1] - 1):
                continue
            if frac[i - 1:i + 2, j - 1:j + 2].max() < 0.6:
                continue
            bg += 1
            if t['motion'] == 'MOVING':
                bg_mov += 1
                bg_ids[t['id']] += 1
    if DEBUG:
        print('false MOVING static-scene track ids:', dict(bg_ids))
    herr.sort()
    lats = sorted(o['latency_ms'] for o in onsets if o['latency_ms'] is not None)
    seg_ok = sorted(x for x in seg_lat if x is not None)
    seg_median = seg_ok[len(seg_ok) // 2] if seg_ok else None
    res = {
        'gt_frames_clean': n_gt, 'absorbed_frames': n_absorbed,
        'near_frames': near_n, 'near_recall': near_hit / max(near_n, 1),
        'sub_trajectories': len(subs),
        'recall': n_matched / max(n_gt, 1),
        'recall_updated': n_updated / max(n_gt, 1),
        'interp_frames': n_interp, 'interp_covered': n_interp_cov,
        'id_switches': idsw, 'fragmentations': frag, 'idf1': idf1,
        'walk_frames': walk, 'walk_moving': walk_mov / max(walk, 1),
        'still_frames': still, 'still_false_moving': still_mov / max(still, 1),
        'bg_track_frames': bg, 'bg_false_moving': bg_mov / max(bg, 1),
        'heading_median_deg': herr[len(herr) // 2] if herr else None,
        'heading_p90_deg': herr[int(0.9 * len(herr))] if herr else None,
        'heading_samples': len(herr),
        'walk_segments': seg_lat,
        'walk_segment_latency_median_ms': seg_median,
        'onsets': onsets, 'onset_detected': len(lats), 'onset_total': len(onsets),
        'onset_latency_median_ms': lats[len(lats) // 2] if lats else None,
        'onset_latency_mean_ms': sum(lats) / len(lats) if lats else None,
    }
    pct = 100.0
    print(f"clean observed GT frames {n_gt} (+{n_absorbed} absorbed into static background) "
          f"in {len(subs)} sub-trajectories | recall {pct * res['recall']:.1f} % "
          f"(updated {pct * res['recall_updated']:.1f} %) | "
          f"interp frames covered {n_interp_cov}/{n_interp}")
    near_recall = pct * near_hit / max(near_n, 1)
    print(f"near zone (< {NEAR} m, desk): {near_n} frames, recall {near_recall:.1f} % "
          "(reported separately)")
    print(f"ID switches {idsw} | fragmentations {frag} | IDF1 {idf1:.3f}")
    print(f"walking frames {walk}: MOVING {pct * res['walk_moving']:.1f} % | "
          f"still frames {still}: false MOVING {pct * res['still_false_moving']:.2f} % | "
          f"static-scene track frames {bg}: false MOVING {pct * res['bg_false_moving']:.2f} %")
    if herr:
        print(f"heading error median {res['heading_median_deg']:.1f} deg, "
              f"p90 {res['heading_p90_deg']:.1f} deg ({len(herr)})")
    onset_mean = res['onset_latency_mean_ms']
    print(f"motion onsets {len(onsets)}: detected {len(lats)}, "
          f"latency median {res['onset_latency_median_ms']} ms, "
          f"mean {round(onset_mean) if onset_mean is not None else None} ms  "
          f"{[o['latency_ms'] for o in onsets]}")
    ok = [x for x in seg_lat if x is not None]
    seg_mean = round(sum(ok) / len(ok)) if ok else None
    print(f"walking segments (GT > {WALK} m/s for >= 1 s, not MOVING at start): {len(seg_lat)}, "
          f"reached MOVING {len(ok)}, latency median {res['walk_segment_latency_median_ms']} ms, "
          f"mean {seg_mean} ms  {seg_lat}")
    if out_json:
        json.dump(res, open(out_json, 'w'), indent=1)


if __name__ == '__main__':
    main()
