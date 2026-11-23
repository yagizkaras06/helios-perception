# Operator collision warning (display / information only - NOT connected to vehicle control).
# Derived from the tracks' TTC / CPA with uncertainty; only tracks with a reliable velocity can
# produce a predicted collision. See collision_warning.hpp for the rules.
std_msgs/Header header

uint8 CLEAR   = 0     # nothing is coming at the sensor
uint8 CAUTION = 1     # approaching on a collision course (longer TTC) or a dynamic object close
uint8 WARNING = 2     # predicted path reaches the ego circle, pessimistic TTC short

uint8 level
bool collision_predicted      # predicted path (k-sigma band) reaches the ego circle
uint32 track_id               # threat track (0: none)
float32 distance              # [m] closest point from the LiDAR
float32 clearance             # [m] distance - ego_radius
float32 closing_speed         # [m/s]
float32 closing_speed_sigma
float32 ttc                   # [s] nominal
float32 ttc_pessimistic       # [s] closing speed + k sigma
float32 cpa_time              # [s]
float32 cpa_distance          # [m] gap at closest approach (< 0: overlap)
string text                   # ASCII summary shown in RViz
