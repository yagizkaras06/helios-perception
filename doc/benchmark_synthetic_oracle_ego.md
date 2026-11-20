## Synthetic Helios-32 scenarios (ground truth)

### Scenario `crossing` (100 frames, ground czm, ego oracle)

| metric | value |
|---|---|
| frames | 95 |
| visible GT object-frames | 722 |
| TP / FP / FN / ID switches | 704 / 18 / 18 / 4 |
| MOTA | 0.945 |
| MOTP (centre error) | 0.188 m |
| precision / recall | 0.975 / 0.975 |
| velocity RMSE (vector) | 0.257 m/s (697 samples) |
| speed MAE | 0.074 m/s |
| heading MAE (moving) | 2.2 deg (322 samples) |
| moving GT -> MOVING / STATIC / UNKNOWN | 322 / 0 / 13 |
| static GT -> STATIC / MOVING / UNKNOWN | 348 / 0 / 14 |
| TTC MAE | 0.413 s (110 samples) |
| pessimistic TTC > true TTC | 9.1 % |
| ID switches (gt id: frames) | 2:1 4:1 6:1 8:1 |
| missed (FN) (gt id: frames) | 2:8 3:4 6:2 8:4 |
| ego velocity error mean / p95 | 0.000 / 0.000 m/s |
| ego yaw-rate error mean | 0.0000 rad/s |
| ego states STATIONARY / MOVING / UNRELIABLE / UNKNOWN | 95 / 0 / 0 / 0 |

Timing (synthetic scans, 3 threads):

| stage | mean ms | p95 ms | max ms |
|---|---|---|---|
| roi_filter | 0.33 | 0.47 | 0.62 |
| voxel_grid | 1.39 | 1.61 | 1.94 |
| ground_segmentation | 0.59 | 0.71 | 0.76 |
| clustering | 1.76 | 3.25 | 3.38 |
| bounding_box | 0.13 | 0.19 | 0.31 |
| ego_motion(concurrent) | 0.00 | 0.00 | 0.00 |
| tracking_risk | 0.03 | 0.05 | 0.06 |
| total | 4.23 | 5.81 | 6.23 |

### Scenario `ego_motion` (100 frames, ground czm, ego oracle)

| metric | value |
|---|---|
| frames | 95 |
| visible GT object-frames | 2392 |
| TP / FP / FN / ID switches | 2151 / 116 / 241 / 28 |
| MOTA | 0.839 |
| MOTP (centre error) | 0.287 m |
| precision / recall | 0.949 / 0.899 |
| velocity RMSE (vector) | 0.501 m/s (2066 samples) |
| speed MAE | 0.109 m/s |
| heading MAE (moving) | 2.7 deg (186 samples) |
| moving GT -> MOVING / STATIC / UNKNOWN | 186 / 10 / 57 |
| static GT -> STATIC / MOVING / UNKNOWN | 1622 / 0 / 191 |
| TTC MAE | 0.275 s (533 samples) |
| pessimistic TTC > true TTC | 5.3 % |
| moving GT shown STATIC (gt id: frames) | 2:4 3:6 |
| ID switches (gt id: frames) | 2:3 3:1 100:1 102:1 103:1 104:1 106:1 107:1 109:1 110:3 112:1 113:1 114:1 116:2 117:1 119:1 121:1 123:2 125:1 126:1 127:1 129:1 |
| missed (FN) (gt id: frames) | 2:26 100:2 101:14 102:2 103:4 105:14 106:2 107:2 109:7 110:12 111:3 112:5 113:2 114:2 115:63 116:2 117:4 120:37 121:2 122:2 123:15 125:2 126:2 127:2 128:3 129:10 |
| ego velocity error mean / p95 | 0.000 / 0.000 m/s |
| ego yaw-rate error mean | 0.0000 rad/s |
| ego states STATIONARY / MOVING / UNRELIABLE / UNKNOWN | 0 / 95 / 0 / 0 |

Timing (synthetic scans, 3 threads):

| stage | mean ms | p95 ms | max ms |
|---|---|---|---|
| roi_filter | 0.35 | 0.39 | 0.51 |
| voxel_grid | 1.61 | 1.68 | 2.33 |
| ground_segmentation | 0.63 | 0.73 | 0.95 |
| clustering | 5.32 | 5.83 | 7.91 |
| bounding_box | 0.28 | 0.39 | 0.48 |
| ego_motion(concurrent) | 0.00 | 0.00 | 0.00 |
| tracking_risk | 0.10 | 0.13 | 0.16 |
| total | 8.29 | 8.79 | 12.09 |

### Scenario `approach` (100 frames, ground czm, ego oracle)

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
| roi_filter | 0.35 | 0.38 | 0.56 |
| voxel_grid | 1.62 | 1.68 | 1.71 |
| ground_segmentation | 0.70 | 0.84 | 0.87 |
| clustering | 4.29 | 4.54 | 4.61 |
| bounding_box | 0.11 | 0.12 | 0.14 |
| ego_motion(concurrent) | 0.00 | 0.00 | 0.00 |
| tracking_risk | 0.02 | 0.03 | 0.03 |
| total | 7.08 | 7.40 | 7.54 |

