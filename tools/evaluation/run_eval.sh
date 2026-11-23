#!/usr/bin/env bash
# Version comparison on synthetic scenarios (exact ground truth) and, optionally, a real recording
# with a pseudo ground truth.
#
#   run_eval.sh <colcon_ws> <out_dir> [<bag_dir> <gt.csv>]
#
#   <colcon_ws>  workspace containing build/helios_obstacle_detection/helios_benchmark
#   <bag_dir>    real recording (stationary sensor), <gt.csv> made by pseudo_gt.py for it:
#                  python3 load_bag.py <bag_dir> rec.npz && python3 pseudo_gt.py rec.npz gt/gt.csv
set -euo pipefail
WS=$1; OUT=$2; BAG=${3:-}; GT=${4:-}
EV=$(cd "$(dirname "$0")" && pwd)
B=$WS/build/helios_obstacle_detection/helios_benchmark
mkdir -p "$OUT"
: > "$OUT/synthetic_metrics.txt"
for sc in occlusion merge_split wall_walk start_stop crossing ego_motion approach; do
  dur=25; [[ $sc == start_stop ]] && dur=30
  [[ $sc == crossing || $sc == ego_motion || $sc == approach ]] && dur=20
  for seed in 1 2 3 4 5; do
    "$B" synthetic --scenario $sc --duration $dur --seed $seed > "$OUT/syn_${sc}_${seed}.md" || true
    grep '^METRICS' "$OUT/syn_${sc}_${seed}.md" >> "$OUT/synthetic_metrics.txt"
  done
done
python3 "$EV/aggregate.py" "$OUT/synthetic_metrics.txt" | tee "$OUT/synthetic_summary.txt"
if [[ -n "$BAG" && -n "$GT" ]]; then
  "$B" bag "$BAG" --tracks-csv "$OUT/tracks.csv" > "$OUT/bag_report.md"
  python3 "$EV/pseudo_gt_eval.py" "$GT" "$OUT/tracks.csv" --json "$OUT/real_eval.json" | tee "$OUT/real_eval.txt"
  python3 "$EV/proxy_eval_v02.py" "$OUT/tracks.csv" | tee "$OUT/proxy_v02.txt"
  grep -A12 '^| stage' "$OUT/bag_report.md" | tee "$OUT/timing.txt"
fi
