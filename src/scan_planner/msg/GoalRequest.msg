# One atomic point-to-point command. Omitted tolerance uses the receiver default.
float64 MIN_XY_TOLERANCE=0.05
float64 MAX_XY_TOLERANCE=1.0
geometry_msgs/PoseStamped target
uint64 goal_id
uint64 attempt_id
uint32 waypoint_index
bool has_xy_tolerance
float64 xy_tolerance
# Optional continuous reference. Its last point must equal target.position.
# Intermediate points shape the route, only target requires a stopped receipt.
geometry_msgs/Point[] reference_points
