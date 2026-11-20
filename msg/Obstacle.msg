# Single-frame geometric obstacle candidate.
# NOTE: `id` is only valid inside ONE frame (sorted by closest_distance, 0 = nearest).
#       It is NOT a persistent tracking id. No semantic class is implied.
# All coordinates are in the frame given by the parent ObstacleArray header (sensor frame).

uint32 id
uint32 point_count

# Axis-aligned bounding box (AABB)
geometry_msgs/Point center      # ((min+max)/2) per axis
geometry_msgs/Vector3 size      # x = L = xmax-xmin, y = W = ymax-ymin, z = H = zmax-zmin
geometry_msgs/Point min
geometry_msgs/Point max

# Distances measured from the sensor origin (0,0,0) — NOT vehicle body clearance.
geometry_msgs/Point closest_point
float32 closest_distance        # [m] 3D Euclidean distance to closest cluster point
float32 center_distance         # [m] 3D Euclidean distance to AABB center

bool large_structure            # footprint exceeds large_structure_size (wall/building-like)

# Yaw-oriented box (L-shape fit). obb_size.x = length along obb_yaw (>= obb_size.y).
geometry_msgs/Point obb_center
geometry_msgs/Vector3 obb_size
bool background                 # cluster of the learned static background (static map)
float32 obb_yaw                 # [rad] about +Z of the sensor frame, in [-pi/2, pi/2)
