"""Aggregates METRICS lines (several seeds) per scenario: sums of counts, pooled onset latencies."""
import sys
import collections
import math
rows = collections.defaultdict(list)
for line in open(sys.argv[1]):
    kv = dict(t.split('=', 1) for t in line.split()[1:])
    rows[kv['scenario']].append(kv)


def f(x):
    try:
        return float(x)
    except ValueError:
        return float('nan')


print(
    f"{
        'scenario':12} {
            'runs':>4} {
                'GT':>6} {
                    'FN':>5} {
                        'FP':>5} {
                            'IDSW':>5} {
                                'frag':>5} {
                                    'MOTA':>6} {
                                        'IDF1':>6} {
                                            'recall':>6} {
                                                'prec':>6} {
                                                    'mov->MOV':>8} {
                                                        'falseMOV':>9} {
                                                            'onsets':>11} {
                                                                'onset med/mean ms':>18} {
                                                                    'stop med':>8} {
                                                                        'hdg':>5}")
for sc, rs in rows.items():
    def S(k): return sum(f(r[k]) for r in rs)
    gt, fn, fp, idsw = S('gt'), S('fn'), S('fp'), S('idsw')
    mota = 1 - (fn + fp + idsw) / gt
    tp = S('tp')
    idf1 = sum(f(r['idf1']) * (f(r['gt']) + f(r['tp']) + f(r['fp'])) for r in rs) / (gt + tp + fp)
    mm = S('mov_as_mov') / max(S('mov_as_mov') + S('mov_as_static') + S('mov_as_unknown'), 1)
    fm = S('never_moved_as_moving') / max(S('never_moved_frames'), 1)
    det = S('onset_detected')
    ons = S('onsets')
    # pooled mean from per-run means
    pm = [(f(r['onset_mean_ms']), f(r['onset_detected'])) for r in rs if f(r['onset_detected']) > 0]
    mean = sum(a * b for a, b in pm) / max(sum(b for _, b in pm), 1) if pm else float('nan')
    med = sorted(f(r['onset_median_ms']) for r in rs if f(r['onset_detected']) > 0)
    med = med[len(med) // 2] if med else float('nan')
    stop = sorted(f(r['stop_median_ms']) for r in rs if not math.isnan(f(r['stop_median_ms'])))
    stop = stop[len(stop) // 2] if stop else float('nan')
    hdg = sum(f(r['heading_mae']) for r in rs) / len(rs)
    prec = tp / max(tp + fp, 1)
    onsets = f"{det:>4.0f}/{ons:<3.0f}({S('onset_missed'):.0f}m)"
    print(f"{sc:12} {len(rs):>4} {gt:>6.0f} {fn:>5.0f} {fp:>5.0f} {idsw:>5.0f} "
          f"{S('frag'):>5.0f} {mota:>6.3f} {idf1:>6.3f} {tp / gt:>6.3f} {prec:>6.3f} "
          f"{100 * mm:>7.1f}% {100 * fm:>8.2f}% {onsets} {med:>8.0f}/{mean:<8.0f} "
          f"{stop:>8.0f} {hdg:>5.1f}")
