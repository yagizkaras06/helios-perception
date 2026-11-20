#!/usr/bin/env bash
# usage: run_eval.sh <workspace_dir> <out_dir>   (workspace with build/helios_obstacle_detection)
set -euo pipefail
WS=$1; OUT=$2; EV=$(cd "$(dirname "$0")" && pwd)
B=$WS/build/helios_obstacle_detection/helios_benchmark
mkdir -p "$OUT"
: > "$OUT/synthetic_metrics.txt"
for sc in occlusion merge_split wall_walk start_stop crossing ego_motion approach; do
  dur=25; [[ $sc == start_stop ]] && dur=30; [[ $sc == crossing || $sc == ego_motion || $sc == approach ]] && dur=20
  for seed in 1 2 3 4 5; do
    $B synthetic --scenario $sc --duration $dur --seed $seed > "$OUT/syn_${sc}_${seed}.md" || true
    grep '^METRICS' "$OUT/syn_${sc}_${seed}.md" >> "$OUT/synthetic_metrics.txt"
  done
done
python3 "$EV/aggregate.py" "$OUT/synthetic_metrics.txt" | tee "$OUT/synthetic_summary.txt"
# real recording
$B bag ~/helios_ws/bags/walk_001258 --tracks-csv "$OUT/walk_tracks.csv" > "$OUT/bag_report.md"
python3 "$EV/pseudo_gt_eval.py" "$EV/gt_walk.csv" "$OUT/walk_tracks.csv" --json "$OUT/real_eval.json" | tee "$OUT/real_eval.txt"
python3 "$EV/proxy_eval_v02.py" "$OUT/walk_tracks.csv" | tee "$OUT/proxy_v02.txt"
grep -A12 '^| stage' "$OUT/bag_report.md" | tee "$OUT/timing.txt"
