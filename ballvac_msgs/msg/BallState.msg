# Ball state in the fleet coordination system
std_msgs/Header header

# Ball identification
string ball_id          # Unique ball ID (e.g., "ball_red_0")
string color            # Ball color: red, green, blue, yellow, orange, purple, cyan

# Position in map frame
geometry_msgs/PoseStamped pose

# State enumeration (uint8 for efficiency)
uint8 UNCLAIMED=0
uint8 CLAIMED=1
uint8 COLLECTED=2
uint8 LOST=3
uint8 state

# Claim information
string claimed_by_robot  # Robot ID that claimed this ball (empty if unclaimed)
builtin_interfaces/Time claim_time
builtin_interfaces/Time last_seen_time

# Priority (lower = higher priority)
uint8 priority
