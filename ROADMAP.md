# Roadmap: v0.4.0

Scope: the LiDAR-only perception stack in this repository. Camera / sensor fusion (e.g. ZED 2) is
planned as a separate project and is out of scope here. Every item has a measurable exit criterion;
"current" values are from [doc/benchmarks.md](doc/benchmarks.md).

| # | Goal | Current (v0.3.1) | Exit criterion for v0.4.0 |
|---|---|---|---|
| 1 | **Accuracy on real data.** Labelled Helios recordings (boxes, track ids) and a CI-runnable evaluation | Real data: consistency checks only; accuracy only on synthetic data | ≥ 3 labelled sequences, ≥ 2,000 frames, indoor + outdoor; MOTA, IDF1, velocity RMSE reported per sequence |
| 2 | **Identity after occlusion.** Fewer new tracks for the same object, no re-association to a different object | `tracks_created_total` +25 in a 19 s live sample of a mostly static indoor scene (~80 / min, tentative tracks included); one wrong re-association seen in the same sample | ≤ 30 / min on the same scene over ≥ 10 min; IDF1 on the synthetic `occlusion` / `merge_split` scenarios not lower than now (0.983 / 0.961) |
| 3 | **Ego-motion cost.** Parallel nearest-neighbour queries in ICP with a fixed reduction order (deterministic) | 7.3 ms mean, ~11 ms p95 live; largest stage | ≤ 4 ms mean / ≤ 6 ms p95 live; pose difference to the sequential result < 1 mm / 0.01 deg on the recordings |
| 4 | **Moving-vehicle validation.** Recordings on the vehicle at walking and driving speed | Live tests with a stationary sensor only | ≥ 30 min driving data; ego state UNKNOWN / UNRELIABLE < 2 % of frames outside degenerate scenes; no static object shown MOVING |
| 5 | **Intra-scan motion compensation (deskew)** from the ego velocity | Not done (scan smears ~1 m at 10 m/s) | Simulator with per-point timing; box centre error at 10 m/s reduced by ≥ 50 % |
| 6 | **Vehicle footprint for TTC.** Rectangle (length, width, sensor offset) instead of a circle around the sensor | `risk.ego_radius` circle | Footprint parameters + tests; TTC MAE on synthetic `approach` ≤ 0.05 s with the rectangle |
| 7 | **Input health.** Distinguish driver-side frame loss from network loss in the diagnostics | Gaps detected from header stamps, cause not separated | Diagnostic reports the packet rate on the interface vs published scans (read-only), documented recovery steps |
| 8 | **Event retention.** Optional oldest-first deletion when the quota is reached | New events are refused when the quota is full | `events.retention: refuse | delete_oldest`, tests for both |
| 9 | **Reproducible environment.** Container image for build, tests and the synthetic demo | Native build only | `docker run ... synthetic` demo; fresh-clone build + tests in CI < 15 min |

Not planned for v0.4.0: object classification (pedestrian / vehicle), vehicle control interfaces,
safety certification.
