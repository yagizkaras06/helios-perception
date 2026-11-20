## Synthetic Helios-32 scenarios (ground truth)

### Scenario `crossing` (100 frames, ground ransac_zones, ego estimate)

| metric | value |
|---|---|
| frames | 95 |
| visible GT object-frames | 722 |
| TP / FP / FN / ID switches | 704 / 18 / 18 / 4 |
| MOTA | 0.945 |
| MOTP (centre error) | 0.183 m |
| precision / recall | 0.975 / 0.975 |
| velocity RMSE (vector) | 0.252 m/s (697 samples) |
| speed MAE | 0.072 m/s |
| heading MAE (moving) | 2.2 deg (322 samples) |
| moving GT -> MOVING / STATIC / UNKNOWN | 322 / 0 / 13 |
| static GT -> STATIC / MOVING / UNKNOWN | 348 / 0 / 14 |
| TTC MAE | 0.414 s (110 samples) |
| pessimistic TTC > true TTC | 9.1 % |
| ID switches (gt id: frames) | 2:1 4:1 6:1 8:1 |
| missed (FN) (gt id: frames) | 2:8 3:4 6:2 8:4 |
| ego velocity error mean / p95 | 0.000 / 0.000 m/s |
| ego yaw-rate error mean | 0.0000 rad/s |
| ego states STATIONARY / MOVING / UNRELIABLE / UNKNOWN | 95 / 0 / 0 / 0 |

Timing (synthetic scans, 3 threads):

| stage | mean ms | p95 ms | max ms |
|---|---|---|---|
| roi_filter | 0.35 | 0.51 | 0.66 |
| voxel_grid | 1.39 | 1.61 | 2.29 |
| ground_segmentation | 0.59 | 0.64 | 1.07 |
| clustering | 1.78 | 3.21 | 3.31 |
| bounding_box | 0.14 | 0.26 | 0.36 |
| ego_motion(concurrent) | 0.60 | 0.92 | 1.65 |
| tracking_risk | 0.03 | 0.04 | 0.05 |
| total | 4.29 | 5.74 | 6.08 |

### Scenario `ego_motion` (100 frames, ground ransac_zones, ego estimate)

| metric | value |
|---|---|
| frames | 95 |
| visible GT object-frames | 2392 |
| TP / FP / FN / ID switches | 2156 / 117 / 236 / 27 |
| MOTA | 0.841 |
| MOTP (centre error) | 0.289 m |
| precision / recall | 0.949 / 0.901 |
| velocity RMSE (vector) | 0.434 m/s (2068 samples) |
| speed MAE | 0.099 m/s |
| heading MAE (moving) | 2.1 deg (183 samples) |
| moving GT -> MOVING / STATIC / UNKNOWN | 183 / 14 / 61 |
| static GT -> STATIC / MOVING / UNKNOWN | 1630 / 0 / 180 |
| TTC MAE | 0.280 s (533 samples) |
| pessimistic TTC > true TTC | 5.8 % |
| moving GT shown STATIC (gt id: frames) | 2:5 3:9 |
| ID switches (gt id: frames) | 2:3 3:1 100:1 102:1 103:1 104:1 106:1 107:1 109:1 110:3 112:1 113:1 114:1 116:1 117:1 119:1 121:1 123:2 125:1 126:1 127:1 129:1 |
| missed (FN) (gt id: frames) | 2:21 100:2 101:14 102:2 103:3 105:14 106:2 107:2 109:5 110:12 111:3 112:4 113:2 114:2 115:63 116:6 117:4 120:37 121:2 122:2 123:15 125:2 126:2 127:2 128:3 129:10 |
| ego velocity error mean / p95 | 0.023 / 0.023 m/s |
| ego yaw-rate error mean | 0.0005 rad/s |
| ego states STATIONARY / MOVING / UNRELIABLE / UNKNOWN | 0 / 95 / 0 / 0 |

Timing (synthetic scans, 3 threads):

| stage | mean ms | p95 ms | max ms |
|---|---|---|---|
| roi_filter | 0.36 | 0.39 | 0.41 |
| voxel_grid | 1.61 | 1.71 | 1.76 |
| ground_segmentation | 0.74 | 0.82 | 0.92 |
| clustering | 5.35 | 5.93 | 6.02 |
| bounding_box | 0.33 | 0.46 | 0.63 |
| ego_motion(concurrent) | 4.65 | 5.02 | 7.92 |
| tracking_risk | 0.11 | 0.13 | 0.16 |
| total | 8.58 | 9.26 | 10.77 |

### Scenario `approach` (100 frames, ground ransac_zones, ego estimate)

| metric | value |
|---|---|
| frames | 95 |
| visible GT object-frames | 373 |
| TP / FP / FN / ID switches | 373 / 3 / 0 / 0 |
| MOTA | 0.992 |
| MOTP (centre error) | 0.171 m |
| precision / recall | 0.992 / 1.000 |
| velocity RMSE (vector) | 0.009 m/s (373 samples) |
| speed MAE | 0.002 m/s |
| heading MAE (moving) | 0.2 deg (91 samples) |
| moving GT -> MOVING / STATIC / UNKNOWN | 91 / 0 / 2 |
| static GT -> STATIC / MOVING / UNKNOWN | 274 / 0 / 6 |
| TTC MAE | 0.020 s (82 samples) |
| pessimistic TTC > true TTC | 0.0 % |
| ego velocity error mean / p95 | 0.000 / 0.000 m/s |
| ego yaw-rate error mean | 0.0000 rad/s |
| ego states STATIONARY / MOVING / UNRELIABLE / UNKNOWN | 95 / 0 / 0 / 0 |

Timing (synthetic scans, 3 threads):

| stage | mean ms | p95 ms | max ms |
|---|---|---|---|
| roi_filter | 0.35 | 0.37 | 0.38 |
| voxel_grid | 1.61 | 1.66 | 1.68 |
| ground_segmentation | 0.68 | 0.77 | 0.87 |
| clustering | 4.29 | 4.51 | 4.58 |
| bounding_box | 0.11 | 0.12 | 0.13 |
| ego_motion(concurrent) | 3.31 | 3.88 | 5.50 |
| tracking_risk | 0.02 | 0.02 | 0.03 |
| total | 7.08 | 7.31 | 8.38 |

