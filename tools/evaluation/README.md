# Evaluation tools (not installed)

Used for the v0.2 → v0.3 comparison in `doc/v030_report.md`.

| File | Purpose |
|---|---|
| `run_eval.sh` | synthetic scenarios × 5 seeds (CLEAR-MOT, IDF1, onset latency) + optional real recording |
| `aggregate.py` | sums the `METRICS` lines of `helios_benchmark synthetic` per scenario |
| `load_bag.py` | `/rslidar_points` of a rosbag2 → compressed `.npz` |
| `pseudo_gt.py` | pseudo ground truth of a recording with a **stationary** sensor (no human labels) |
| `pseudo_gt_eval.py` | tracker CSV vs. pseudo ground truth (recall, ID switches, IDF1, MOVING rate, false MOVING, heading, onset) |
| `proxy_eval_v02.py` | v0.2 proxy metric (circular: uses the tracker's own positions; kept for comparability) |

```bash
# synthetic only
tools/evaluation/run_eval.sh ~/helios_ws out/v030
# with a real recording
python3 tools/evaluation/load_bag.py ~/bags/walk rec.npz
python3 tools/evaluation/pseudo_gt.py rec.npz gt/gt.csv          # also writes gt/occ_frac.npy
tools/evaluation/run_eval.sh ~/helios_ws out/v030 ~/bags/walk gt/gt.csv
```

The pseudo ground truth is an automatic proposal; only "clean" segments (no other dynamic object
within 1.5 m) are scored, objects still for ≥ 10 s and the < 1 m zone are reported separately.
For release decisions it should be checked / corrected by a person.

`helios_benchmark` prints diagnostic lines (false positives, ID switches, false MOVING, onsets) on
stderr when the environment variable `HELIOS_FP_DEBUG=1` is set.
