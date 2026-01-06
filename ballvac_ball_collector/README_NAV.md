# Ball Collection Navigation System - Usage Guide

## Overview

This system enables autonomous ball collection using an Ackermann-steering robot in Gazebo with:
- **SLAM Toolbox** for continuous mapping
- **Nav2** for global path planning and obstacle avoidance
- **MPPI Controller** optimized for Ackermann steering
- **Smart behavior node** for ball detection and collection

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                        Ball Collection System                       │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  ┌─────────────┐    ┌────────────┐    ┌─────────────────────────┐   │
│  │   Gazebo    │    │   SLAM     │    │        Nav2 Stack       │   │
│  │  Simulation │───▶│  Toolbox   │───▶│  (Planner + Controller) │   │
│  └─────────────┘    └────────────┘    └─────────────────────────┘   │
│        │                  │                      │                  │
│        │ /scan            │ /map                 │ /cmd_vel_nav     │
│        │ /odom            │ map→odom TF          │                  │
│        ▼                  ▼                      ▼                  │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │              nav_ball_collector_node                         │   │
│  │  ┌─────────┐  ┌───────────┐  ┌───────────┐  ┌───────────┐    │   │
│  │  │  IDLE   │→ │ EXPLORING │→ │NAVIGATING │→ │APPROACHING│    │   │
│  │  └─────────┘  └───────────┘  └───────────┘  └───────────┘    │   │
│  │       ↑                                           ↓          │   │
│  │  ┌──────────┐                              ┌───────────      │   │
│  │  │RECOVERING│ ←──────────────────────────│ | COLLECTING│     │   │
│  │  └──────────┘                              └───────────┘     │   │
│  └──────────────────────────────────────────────────────────────┘   │
│        ▲                                                            │
│        │ /ball_detections                                           │
│  ┌─────────────┐                                                    │
│  │    Ball     │                                                    │
│  │  Perception │◀─── /camera/front_raw                              │
│  └─────────────┘                                                    │
└─────────────────────────────────────────────────────────────────────┘
```

## Key Fixes Applied

### 1. Costmap Configuration
- **Increased inflation radius** (0.20m → 0.55m) for early obstacle detection
- **Added `sensor_frame: "ballvac"`** to obstacle layer observations
- **Enlarged local costmap** (4x4m → 6x6m) for better Ackermann lookahead
- **Enabled footprint-based collision checking**

### 2. MPPI Controller Improvements
- Added **ObstaclesCritic** for explicit collision avoidance
- Enabled **`consider_footprint: true`** in CostCritic
- Reduced velocities for safer navigation
- Added proper **Ackermann constraints** (`min_turning_r: 0.25`)

### 3. Behavior Node Enhancements
- Smart exploration using LiDAR gap-finding
- **Obstacle-aware approach** - detects obstacles between robot and ball
- Improved stuck detection and escape maneuvers
- Nav2-based exploration with reactive fallback

### 4. Recovery Behaviors
- Integrated Nav2 behaviors (spin, backup, wait)
- Multi-phase escape maneuvers for corner situations
- Blocked direction tracking to avoid repeated failures

## Build Instructions

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Navigate to workspace
cd /home/acan/ackermen_ws

# Build all packages
colcon build

# Source the workspace
source install/setup.bash
```

## Running the System

### Option 1: Full Integrated Launch (Recommended)

```bash
# Terminal 1: Launch everything
ros2 launch ballvac_ball_collector ball_collection_full.launch.py

# Optional arguments:
# use_rviz:=false     - Disable RViz
# world_name:=my_world - Custom world name
```

### Option 2: Separate Components

```bash
# Terminal 1: Start simulation
ros2 launch ballvac_bringup ball_arena_spawn.launch.py

# Terminal 2: Start navigation (after 5 seconds)
ros2 launch ballvac_ball_collector nav_ball_collect.launch.py slam:=True

# Terminal 3: Monitor (optional)
ros2 topic echo /ball_detections
```

## Validation Checklist

### ✅ Robot no longer drives through obstacles
- MPPI controller checks footprint against costmap
- Collision monitor provides safety layer
- Costmaps properly inflate obstacles

### ✅ Robot does not freeze in front of obstacles
- Nav2 planner finds paths around obstacles
- Escape maneuver triggers if stuck
- Recovery behaviors available (spin, backup)

### ✅ Robot detours to collect balls while mapping
- Ball detection triggers state transition to NAVIGATING
- Nav2 plans path to ball position
- Switches to reactive APPROACHING when close

### ✅ Ball near obstacle: approaches without collision
- Smart approach detects obstacles between robot and ball
- Steers around intermediate obstacles
- Maintains safe distance using LiDAR

## Configuration Files

| File | Purpose |
|------|---------|
| [nav2_ball_collector_params.yaml](config/nav2_ball_collector_params.yaml) | Nav2 + SLAM parameters |
| [ball_collection_full.launch.py](launch/ball_collection_full.launch.py) | Integrated launch file |

## Key Parameters to Tune

### Speed and Safety
```yaml
# In nav2_ball_collector_params.yaml
controller_server:
  FollowPath:
    vx_max: 0.8        # Reduce for safer operation
    vx_min: -0.5       # Reverse speed
    wz_max: 1.9        # Max angular velocity
```

### Obstacle Avoidance
```yaml
local_costmap:
  local_costmap:
    inflation_layer:
      inflation_radius: 0.55   # Increase for larger safety margin
      cost_scaling_factor: 3.0 # Higher = steeper cost falloff
```

### Ball Collection
```yaml
# Launch parameters for nav_ball_collector_node
'approach_speed': 0.5          # Speed when approaching ball
'obstacle_stop_m': 0.35        # Stop distance from obstacles
'collect_distance_m': 0.35     # Distance to trigger collection
```

## Troubleshooting

### Robot keeps freezing
1. Check TF tree: `ros2 run tf2_tools view_frames`
2. Verify `/scan` is publishing: `ros2 topic hz /scan`
3. Increase `movement_time_allowance` in progress checker

### Nav2 goals rejected
1. Wait for SLAM to build initial map
2. Check lifecycle manager: `ros2 topic echo /lifecycle_manager_navigation/transition_event`
3. Increase `wait_for_service_timeout` in bt_navigator

### Robot oscillates
1. Reduce `steering_gain` parameter
2. Increase `approach_radius_threshold`
3. Lower `max_steer` value

## Architecture Diagram

```
Sensors                    Processing                     Actuators
┌─────────┐               ┌───────────────┐              ┌─────────┐
│  LiDAR  │──/scan───────▶│  SLAM Toolbox │──/map──────▶│   Nav2  │
└─────────┘               └───────────────┘              │ Planner │
                                   │                     └────┬────┘
┌─────────┐               ┌───────────────┐                   │
│ Camera  │──/image──────▶│Ball Perception│                   │
└─────────┘               └───────┬───────┘                   │
                                  │                           │
┌─────────┐               ┌───────▼───────┐              ┌────▼────┐
│  Odom   │──/odom───────▶│  Collector    │◀─────────────│  MPPI   │
└─────────┘               │    Node       │──/cmd_vel───▶│Controller│
                          └───────────────┘              └─────────┘
```

## License

This project follows the same license as the parent repository.
