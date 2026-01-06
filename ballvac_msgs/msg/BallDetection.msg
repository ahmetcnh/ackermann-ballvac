# Message representing a single detected ball
std_msgs/Header header

# Ball identification
string name              # e.g., "ball_red", "ball_green"
string color             # Color name: red, green, blue, yellow, orange, purple

# Detection information
float32 bearing          # Angle offset from image center (radians, + is right, - is left)
float32 apparent_size    # Radius of detected ball in pixels
float32 confidence       # Detection confidence [0.0 - 1.0]

# Pixel coordinates in image
int32 center_x           # X coordinate of ball center in image
int32 center_y           # Y coordinate of ball center in image
