# Persistent multi-object track (constant-velocity Kalman filter + Hungarian association).
# Frame: the parent TrackedObjectArray header frame (sensor frame, "rslidar").
#
# Velocities are GROUND-relative, obtained by compensating the sensor's own motion with a
# LiDAR-only scan-matching estimate (no IMU / GPS / wheel odometry). They are trustworthy only
# when velocity_reliable is true (track confirmed and old enough, not a large structure, and the
# parent array reports a usable ego-motion state). No semantic class is implied.

uint8 TRACK_TENTATIVE = 0     # not yet confirmed (may be clutter)
uint8 TRACK_CONFIRMED = 1
uint8 TRACK_COASTING  = 2     # confirmed but not observed in this scan (predicted)

# With the static background map (default): STATIC = part of the background, i.e. stayed still
# for >= static_map.static_time_s (10 s); UNKNOWN = dynamic object that is not moving right now
# (new, stopped < 10 s) or no evidence (ego motion unknown).
uint8 MOTION_UNKNOWN = 0      # not enough evidence / dynamic but not moving now
uint8 MOTION_STATIC  = 1
uint8 MOTION_MOVING  = 2

uint8 RISK_NONE     = 0
uint8 RISK_LOW      = 1
uint8 RISK_MEDIUM   = 2
uint8 RISK_HIGH     = 3
uint8 RISK_CRITICAL = 4

uint32 id                     # persistent track id
uint8 track_state
uint8 motion_state
bool ever_moved               # was MOVING at some point (e.g. pedestrian that stopped)
bool velocity_reliable
bool large_structure          # wall / building-like; box centre and velocity unreliable
bool in_static_background     # last detection belonged to the static background map
float32 age_s
uint32 hits
uint32 consecutive_misses
float32 time_since_update_s
uint32 point_count
int32 detection_id            # index into the ObstacleArray of the same scan, -1 if coasting

# Yaw-oriented 3D box (filtered position, smoothed shape)
geometry_msgs/Point center
geometry_msgs/Vector3 size    # x = length along yaw, y = width, z = height
float32 yaw                   # [rad] about +Z of the sensor frame
float32[4] position_covariance   # row-major 2x2 (x, y) [m^2]

# Ground-relative velocity in sensor axes. STATIC tracks report the static model (0 +- sigma).
geometry_msgs/Vector3 velocity   # z unused (planar model)
float32[4] velocity_covariance   # row-major 2x2 [m^2/s^2]
float32 speed                 # [m/s]
float32 speed_sigma           # [m/s] 1-sigma along the velocity direction
float32 heading               # [rad] atan2(vy, vx) in sensor frame
float32 heading_sigma         # [rad]
bool heading_valid            # true only for MOVING tracks with reliable velocity
# Motion evidence used by the classifier (diagnostics)
float32 window_speed          # [m/s] centre regression over the motion window
float32 window_speed_sigma
float32 surface_speed         # [m/s] point-surface displacement rate, -1 if not available

# Motion relative to the (moving) sensor; used for TTC. For STATIC tracks the object velocity
# is modelled as zero so only the ego-motion uncertainty remains.
geometry_msgs/Vector3 relative_velocity
float32[4] relative_velocity_covariance

# Closest point / distance from the LiDAR origin (not vehicle-body clearance)
geometry_msgs/Point closest_point
float32 closest_distance

# Time-to-collision w.r.t. a circle of radius ego_radius around the LiDAR (see risk params).
bool ttc_valid                # false -> velocity not reliable, risk is distance-based only
float32 clearance             # [m] closest_distance (horizontal) - ego_radius
float32 closing_speed         # [m/s] > 0 when approaching
float32 closing_speed_sigma
float32 ttc                   # [s] nominal, +inf if not closing
float32 ttc_pessimistic       # [s] with closing_speed + k*sigma (conservative bound)
float32 cpa_time              # [s] time of closest approach (constant relative velocity)
float32 cpa_distance          # [m] gap at closest approach (<0 = predicted overlap)
bool collision_course
uint8 risk_level
