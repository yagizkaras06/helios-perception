# Helios perception performance (45 s, ns /helios_eval)

| metric | value |
|---|---|
| input FPS (mean / min) | 10.03 / 9.98 |
| output FPS (mean / min) | 10.03 / 9.96 |
| sensor stamp rate | 10.00 Hz |
| CPU, whole node (100 % = 1 core) mean / max | 21.3 / 24.0 % |
| CPU normalised to all cores | 1.33 % |
| RSS | 67.1 MB, 19 threads |
| host latency rx->publish mean / p95 / max | 13.8 / 15.5 / 23.4 ms |
| clock synced | False (offset 311888235.806 s) |
| stamp age mean / p95 | unobservable (sensor clock not synchronised) |
| delivery jitter p95 | 13.93 ms |
| frames rx / processed / superseded / overload | 490 / 490 / 0 / 0 |
| gaps / missed (est.) / time jumps / partial scans | 0 / 0 / 2 / 0 |
| valid return ratio / azimuth coverage | 0.716 / 0.995 |
| voxel leaf (adaptive) | 0.100 m |
| ground | czm: czm bins acc=5/81 tilt=29 rough=2 elev=1 disc=1 sparse=43 | pts own=10.70% ref=27.72% fb=61.58% | elev_thr=0.15,0.15,0.30,0.30 |
| confirmed tracks per frame / moving | 30.3 / 0.00 |
| ego states (frames) | {'STATIONARY': 450, 'UNKNOWN': 2} |
| max risk (frames) | {'CRITICAL': 448, 'NONE': 4} |

| stage | mean ms | p95 ms | max ms |
|---|---|---|---|
| roi_filter | 1.00 | 1.27 | 1.41 |
| voxel_grid | 2.22 | 2.63 | 2.91 |
| ground_segmentation | 0.49 | 0.58 | 0.67 |
| clustering | 9.04 | 10.27 | 12.55 |
| bounding_box | 0.54 | 0.82 | 0.96 |
| ego_motion | 5.75 | 7.31 | 9.99 |
| tracking_risk | 0.20 | 0.27 | 0.30 |
| total | 13.58 | 15.16 | 19.74 |
| queue_wait | 0.11 | 0.14 | 11.75 |
| rx_to_publish | 13.79 | 15.48 | 23.38 |
| visualization | 0.01 | 0.02 | 0.02 |

Status reports: {'GROUND_DEGRADED': 26, 'SENSOR_DEGRADED': 19}
