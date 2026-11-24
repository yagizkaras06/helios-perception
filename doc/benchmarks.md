# Benchmarks (v0.3.1)

All numbers on this page were measured on 2026-11-24 with the code of this release. Three kinds of
measurement are kept apart because they answer different questions:

| Kind | Input | Answers | Does not answer |
|---|---|---|---|
| Live sensor | RoboSense Helios 32, real time | Does it keep up at 10 Hz? Latency, CPU, RAM | Accuracy (no ground truth) |
| Offline replay | Two real Helios recordings, as fast as possible | Per-stage cost, throughput | Accuracy (no ground truth) |
| Synthetic | Ray-cast Helios-32-like scans with ground truth | Detection / tracking / motion / TTC accuracy | Real-world accuracy (simplified world, read as an upper bound) |

## Test machine

| | |
|---|---|
| CPU | Intel Core i7-13620H (16 logical CPUs), laptop, `powersave` governor |
| RAM | 31 GB |
| OS / ROS | Ubuntu 24.04, ROS 2 Jazzy, Fast DDS |
| Build | `-DCMAKE_BUILD_TYPE=Release`, `threads: 2` (default) |
| Sensor | RoboSense Helios 32 (`rslidar_sdk`), 10 Hz, 57,600 points per scan, Ethernet 100 Mbit/s |
| Scene | indoor lab, sensor stationary on a desk |

CPU figures are per process with 100 % = one core.

## 1. Live sensor

`detection.launch.py record_events:=true` (detector and event recorder in one process), 817 s window
(13.6 min), values from the node's own `/processing_stats` and log.

| Metric | Value |
|---|---|
| Input / output rate | 9.96 / 9.96 Hz (mean over 147 reports) |
| Frames received / processed / dropped by the detector | 8,145 / 8,145 / 0 |
| Input interruptions | 2, both at 21:13:39-44, ~1.7-1.8 s each (~35 scans); reported as `NO_DATA` / `SENSOR_DEGRADED` |
| Pipeline time per frame | 13.7 ms mean, 18.4 ms p95 (worst 10 s window), 31.1 ms max |
| Receive to publish of `/tracks` | 13.9 ms mean, 18.6 ms p95, 31.3 ms max |
| CPU | 17.9 % mean, 23 % max |
| RSS | 139-163 MB (detector + event recorder) |

Stage means (ms): ROI 0.84, voxel 2.04, ground 0.58, ego-motion 7.28, clustering 1.94, boxes 0.42,
tracking + risk 0.58. Ego-motion is the largest stage on live data.

The interruptions came from the input side: the detector processed every frame it received, and
the gaps are visible in the sensor header stamps. On this setup, frames missing at the driver
output were also observed after Ethernet link interruptions, while the network interface still
received the full packet rate. See *Known limitations* in the README.

Absolute sensor-to-host latency cannot be measured on this setup: the Helios clock is not
synchronised (no PTP / GPS). The node detects this and reports only the delivery jitter.

## 2. Offline replay (real recordings)

`helios_benchmark bag <recording> --repeat 3`: same pipeline code, no ROS transport, as fast as
possible.

| Recording | Frames | Mean | p95 | Max | Throughput |
|---|---|---|---|---|---|
| `walk` (90 s, people walking near the sensor) | 2,703 (901 x 3) | 5.86 ms | 6.95 ms | 11.83 ms | 170 frames/s |
| `static_20s` (20 s, static scene) | 603 (201 x 3) | 4.59 ms | 5.05 ms | 6.01 ms | 217 frames/s |

Stage means on `walk` (ms): ROI 0.34, voxel 0.94, ground 0.26, clustering 1.13, ego-motion 2.65,
tracking + risk 0.23.

Live per-frame times are ~2.5x the offline times. At 10 Hz the cores idle between scans
(`powersave` frequency scaling, cold caches), and the offline run keeps them busy.

The recordings are not part of the repository (site data). Any Helios / `rslidar_sdk` recording can
be measured the same way.

## 3. Synthetic accuracy (ground truth)

`helios_benchmark synthetic --scenario all` and `--scenario tracking`, 10 s per scenario (100 scans;
the first 5 are excluded from the metrics). LiDAR-only ego-motion estimate (`--ego estimate`), CZM
ground.

| Scenario | MOTA | Precision / recall | IDF1 | ID sw. | Velocity RMSE | Moving GT shown MOVING | Never-moved shown MOVING | TTC MAE |
|---|---|---|---|---|---|---|---|---|
| `crossing`: still sensor, pedestrians, cars, static boxes | 0.837 | 0.869 / 0.986 | 0.906 | 1 | 0.17 m/s | 264 / 267 | 0 / 367 | 0.58 s |
| `ego_motion`: sensor 3 m/s + turn, 40 static obstacles | 0.823 | 0.909 / 0.919 | 0.813 | 12 | 0.29 m/s | 178 / 199 | 0 / 1,924 | 0.15 s |
| `approach`: object driving at the still sensor, 2 m/s | 0.992 | 0.992 / 1.000 | 0.996 | 0 | 0.02 m/s | 92 / 93 | 0 / 280 | 0.02 s |
| `occlusion`: walkers pass behind occluders (0.5-2.5 s) | 0.966 | 0.967 / 1.000 | 0.983 | 0 | 0.09 m/s | 371 / 383 | 0 / 285 | - |
| `merge_split`: walkers pass / walk side by side | 0.932 | 0.991 / 0.942 | 0.961 | 1 | 0.19 m/s | 378 / 396 | 0 / 95 | - |
| `wall_walk`: walking along a wall at 0.1-0.2 m | 1.000 | 1.000 / 1.000 | 1.000 | 0 | 0.12 m/s | 23 / 30 | 0 / 255 | - |
| `start_stop`: standing, walking 1-3 s, stopping | 0.985 | 0.991 / 0.996 | 0.939 | 1 | 0.23 m/s | 115 / 154 | 0 / 371 | - |

* LiDAR-only ego-velocity error: 0.000 m/s with the sensor still, 0.018 m/s at 3 m/s with a
  0.05 rad/s turn.
* Motion onset latency (median): 400 ms (`wall_walk`), 300 ms (`start_stop`).
* No object that never moved was ever classified MOVING in any scenario.
* In `crossing` the static boxes stay UNKNOWN (not STATIC) during the 10 s run: the static
  background needs 10 s of evidence before it is learned. This lowers the precision of that run,
  not the moving-object results.
* Pessimistic TTC exceeded the true TTC in 3.3 % of the samples (`crossing`, `ego_motion`).

The simulator has a flat ground, no intra-scan motion and an approximate beam layout. Treat the
numbers as an upper bound for real data. Accuracy on real data needs labelled recordings, which
this project does not have yet (see [ROADMAP](../ROADMAP.md)).

## 4. Cost of the demo layer

Same recording replayed at 10 Hz in an isolated ROS domain, 60 s windows after a 15 s warm-up.

| | Detector only | Detector + demo (RViz, demo_visualizer, status panel) |
|---|---|---|
| Detector receive-to-publish | 15.4 ms mean, 18.1 ms p95, 20.4 ms max | 16.2 ms mean, 18.5 ms p95, 20.7 ms max |
| Detector CPU | 17.3 % | 17.8 % |
| Dropped frames | 0 | 0 |
| RViz2 | - | 10.9 % CPU, 212 MB |
| demo_visualizer.py | - | 5.1 % CPU, 74 MB |
| status_panel.py | - | 3.4 % CPU, 161 MB |

The visualisation runs in separate processes and costs the detector < 1 ms.

## Reproduce

```bash
# offline / synthetic (no sensor)
ros2 run helios_obstacle_detection helios_benchmark synthetic --scenario all
ros2 run helios_obstacle_detection helios_benchmark synthetic --scenario tracking
ros2 run helios_obstacle_detection helios_benchmark bag <recording_dir> --repeat 3 --csv frames.csv
# live: rates, latency and CPU from the running node
ros2 run helios_obstacle_detection perf_report.py --duration 60 --out perf.md
```

Older reports in this folder (`benchmark_*.md`, `live_replay_perf.md`, `v030_report.md`,
`v030_eval/`) document earlier versions and are kept for history. Where they differ from this
page, this page is current.
