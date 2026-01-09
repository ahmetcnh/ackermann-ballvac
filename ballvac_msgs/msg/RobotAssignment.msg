# Assignment from fleet coordinator to a robot
std_msgs/Header header

# Robot this assignment is for
string robot_id

# Assigned ball info (empty if no assignment)
string ball_id
string ball_color
geometry_msgs/PoseStamped goal_pose

# Standoff distance for approach
float32 standoff_distance

# Assignment valid flag
bool has_assignment

# Estimated path cost (meters)
float32 estimated_cost
