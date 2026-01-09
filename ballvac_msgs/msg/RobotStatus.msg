# Status report from robot to fleet coordinator
std_msgs/Header header

# Robot identification
string robot_id

# Robot pose in map frame
geometry_msgs/PoseStamped pose

# Current state
uint8 IDLE=0
uint8 NAVIGATING=1
uint8 APPROACHING=2
uint8 COLLECTING=3
uint8 STUCK=4
uint8 state

# Current assignment (if any)
string assigned_ball_id

# Status flags
bool is_operational
bool navigation_ready

# Action being reported
uint8 ACTION_NONE=0
uint8 ACTION_CLAIM=1
uint8 ACTION_COLLECTED=2
uint8 ACTION_LOST=3
uint8 ACTION_HEARTBEAT=4
uint8 action

# For CLAIM/COLLECTED/LOST actions
string action_ball_id
