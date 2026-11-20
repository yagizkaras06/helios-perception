std_msgs/Header header            # stamp = node wall-clock time at report

uint8 STATUS_OK              = 0
uint8 STATUS_NO_DATA         = 1  # no input for longer than input_timeout_s
uint8 STATUS_OVERLOAD        = 2  # processing time above max_processing_time_ms
uint8 STATUS_GROUND_DEGRADED = 3
uint8 STATUS_GROUND_FAILED   = 4
uint8 STATUS_INVALID_INPUT   = 5  # PointCloud2 layout not usable (missing x/y/z etc.)
uint8 STATUS_SENSOR_DEGRADED = 6  # frame gaps, partial scans, low valid ratio (recent)
uint8 STATUS_EGO_UNKNOWN     = 7  # tracking works but velocities are not ground-relative
uint8 status
string status_text

float32 input_fps                 # messages received from the driver
float32 output_fps                # frames fully processed
float32 sensor_rate_hz            # from header stamps (sensor clock)
uint64 frames_received
uint64 frames_processed
uint64 frames_dropped             # superseded by a newer frame before being processed
uint64 overload_frames
float32 last_input_age_s          # time since last input message (steady clock)

# Sensor data loss / quality
uint64 frame_gaps
uint64 missed_frames_estimate
uint64 time_jumps
uint64 partial_scans
uint64 low_valid_scans
float32 valid_return_ratio
float32 azimuth_coverage
string sensor_health_text

# Latency
bool clock_synced                 # sensor clock comparable with host clock (PTP/GPS)
float64 clock_offset_s            # median(receive wall time - header stamp)
float32 delivery_jitter_p95_ms
float32 stamp_age_mean_ms         # only valid when clock_synced
float32 stamp_age_p95_ms
float32 latency_rx_to_publish_mean_ms  # host: message received -> outputs published
float32 latency_rx_to_publish_p95_ms
float32 latency_rx_to_publish_max_ms

# Resources (whole node process)
float32 cpu_percent               # 100 % = one core
float32 cpu_percent_total         # normalised by core count
float32 rss_mb
int32 threads

# Point counts of the most recent processed frame
uint32 points_input
uint32 points_valid               # finite, inside range limits
uint32 points_roi                 # after ROI + self filter
uint32 points_voxel
uint32 points_ground
uint32 points_obstacle
uint32 clusters
float32 voxel_leaf_size           # current (load-adaptive) leaf [m]

uint8 ground_status               # see ObstacleArray.GROUND_*
string ground_method
string ground_detail

# Tracking / ego motion
uint8 ego_state                   # see TrackedObjectArray.EGO_*
string ego_status_text
uint32 tracks_total
uint32 tracks_confirmed
uint32 tracks_moving
uint32 tracks_static
uint32 tracks_occluded            # predicted (not observed in this scan)
uint32 tracks_lost                # kept for re-identification, not published
uint64 tracks_created_total
uint64 tracks_reidentified_total  # LOST tracks recovered with their id
uint64 merge_splits_total         # merged clusters split between tracks
float32 tracker_predict_ms        # last frame: Kalman prediction
float32 tracker_association_ms    # gated Hungarian association
float32 tracker_occlusion_ms      # occlusion test, merge split, re-identification
uint64 tracker_resets
float32 tracker_mean_nis          # filter consistency, ~2 expected
uint8 max_risk_level
uint32 background_points          # non-ground points in the static background map
uint32 static_voxels
bool background_learning          # map younger than static_time_s: still learning

StageTiming[] timings             # rolling window statistics
