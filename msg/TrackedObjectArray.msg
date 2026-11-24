# Output of the tracker for one scan. Header = input PointCloud2 header (stamp + frame_id).
# PERCEPTION ONLY: nothing here is a control command.
std_msgs/Header header

uint8 EGO_UNKNOWN    = 0      # no usable LiDAR ego-motion: velocities are NOT ground-relative
uint8 EGO_STATIONARY = 1
uint8 EGO_MOVING     = 2
uint8 EGO_UNRELIABLE = 3      # short-term prediction after failed registration
uint8 ego_state
bool ego_motion_measured      # registration succeeded in this scan
geometry_msgs/Vector3 ego_velocity   # sensor velocity over ground, sensor axes [m/s]
float32 ego_yaw_rate          # [rad/s]
float32[9] ego_velocity_covariance   # row-major 3x3 (vx, vy, yaw_rate)
float32 ego_registration_inlier_ratio
float32 ego_registration_rmse # [m]
float32 ego_degeneracy        # smallest translational information / correspondence (0..1)
string ego_status_text

uint8 ground_status           # see ObstacleArray.GROUND_*
uint8 max_risk_level          # over confirmed tracks (STATIC ones skipped while ego is STATIONARY,
                              # see risk.summary_ignore_static_when_stationary; per-track levels unchanged)
int64 max_risk_track_id       # -1 if none

TrackedObject[] objects
