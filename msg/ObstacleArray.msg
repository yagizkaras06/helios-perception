# Header is copied from the input PointCloud2 (stamp + frame_id preserved).
std_msgs/Header header

uint8 GROUND_RELIABLE = 0   # every populated zone produced a validated plane
uint8 GROUND_DEGRADED = 1   # some zones extrapolated from neighbouring zones
uint8 GROUND_FAILED   = 2   # no validated plane; configured fallback plane was used
uint8 ground_status

Obstacle[] obstacles
