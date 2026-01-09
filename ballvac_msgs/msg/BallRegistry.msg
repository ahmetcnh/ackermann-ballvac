# Complete registry of all known balls in the arena
std_msgs/Header header

# Array of all tracked balls
ballvac_msgs/BallState[] balls

# Summary stats
uint32 total_balls
uint32 unclaimed_balls
uint32 claimed_balls
uint32 collected_balls
