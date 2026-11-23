## Bag `<bags>/helios_test_20s`

| item | value |
|---|---|
| frames processed | 603 (201 msgs x 3) |
| ground method / ego mode | czm / estimate |
| worker threads | 2 + caller |
| offline throughput (max speed) | 204.8 frames/s |
| CPU time per frame | 7.2 ms  (=> ~7 % of one core at 10 Hz) |
| peak-ish RSS | 228.9 MB |
| points finite / voxel / non-ground (mean) | 41255 / 3622 / 3038 |
| clusters per frame mean / max | 29.8 / 33 |
| ground RELIABLE / DEGRADED / FAILED | 0 / 603 / 0 |
| accepted ground planes per frame (mean) | 4.9 |
| ego STATIONARY / MOVING / UNRELIABLE / UNKNOWN | 600 / 0 / 0 / 3 |
| measured ego speed mean / p95 / max | 0.000 / 0.000 / 0.000 m/s |
| confirmed tracks per frame mean / max | 30.4 / 34 |
| moving tracks per frame mean / max | 0.00 / 0 |
| confirmed track ids / median lifetime / p90 | 126 / 20.0 s / 20.0 s |
| speed of STATIC tracks mean / p95 (noise floor) | 0.000 / 0.000 m/s |
| frames by max risk NONE/LOW/MEDIUM/HIGH/CRITICAL | 6 / 0 / 0 / 0 / 597 |

Timing (offline, as fast as possible):

| stage | mean ms | p95 ms | max ms |
|---|---|---|---|
| roi_filter | 0.30 | 0.34 | 0.53 |
| voxel_grid | 0.71 | 0.75 | 1.07 |
| ground_segmentation | 0.20 | 0.28 | 0.42 |
| clustering | 3.36 | 3.52 | 5.56 |
| bounding_box | 0.23 | 0.34 | 2.86 |
| ego_motion(concurrent) | 2.10 | 2.76 | 3.94 |
| tracking_risk | 0.09 | 0.11 | 0.20 |
| total | 4.88 | 5.13 | 7.82 |

