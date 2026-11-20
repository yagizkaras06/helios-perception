## Bag `/home/yagiz/helios_ws/bags/helios_test_20s`

| item | value |
|---|---|
| frames processed | 603 (201 msgs x 3) |
| ground method / ego mode | ransac_zones / estimate |
| worker threads | 2 + caller |
| offline throughput (max speed) | 211.1 frames/s |
| CPU time per frame | 6.9 ms  (=> ~7 % of one core at 10 Hz) |
| peak-ish RSS | 229.8 MB |
| points finite / voxel / non-ground (mean) | 41255 / 3622 / 2992 |
| clusters per frame mean / max | 29.9 / 33 |
| ground RELIABLE / DEGRADED / FAILED | 231 / 372 / 0 |
| accepted ground planes per frame (mean) | 1.0 |
| ego STATIONARY / MOVING / UNRELIABLE / UNKNOWN | 600 / 0 / 0 / 3 |
| measured ego speed mean / p95 / max | 0.000 / 0.000 / 0.000 m/s |
| confirmed tracks per frame mean / max | 31.6 / 35 |
| moving tracks per frame mean / max | 0.00 / 0 |
| confirmed track ids / median lifetime / p90 | 162 / 11.7 s / 20.0 s |
| speed of STATIC tracks mean / p95 (noise floor) | 0.000 / 0.000 m/s |
| frames by max risk NONE/LOW/MEDIUM/HIGH/CRITICAL | 6 / 0 / 0 / 0 / 597 |

Timing (offline, as fast as possible):

| stage | mean ms | p95 ms | max ms |
|---|---|---|---|
| roi_filter | 0.29 | 0.33 | 0.38 |
| voxel_grid | 0.69 | 0.74 | 0.89 |
| ground_segmentation | 0.10 | 0.12 | 0.18 |
| clustering | 3.25 | 3.44 | 3.60 |
| bounding_box | 0.30 | 0.40 | 0.49 |
| ego_motion(concurrent) | 2.06 | 2.38 | 4.01 |
| tracking_risk | 0.09 | 0.11 | 0.13 |
| total | 4.73 | 5.00 | 5.29 |

