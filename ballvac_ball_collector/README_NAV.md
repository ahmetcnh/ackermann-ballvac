## Ball Collection Navigation Guide

This document details the navigation-related components, configuration, and best practices for the BallVac collection system. It focuses on costmaps, controller tuning (MPPI for Ackermann vehicles), behavior-node improvements, recovery strategies, and reproducible run procedures.

Overview
--------

The navigation stack integrates SLAM (SLAM Toolbox), Nav2 for planning and lifecycle management, and a tailored MPPI controller adapted to Ackermann steering constraints. The `nav_ball_collector_node` orchestrates behavior transitions (explore → navigate → approach → collect → recover) and coordinates with perception and the launcher node.

System diagram (high level)
---------------------------

Sensors and processing flow:

```
┌────────────────────────────────────────────────────────────────────────────────────────────────────┐
│                                   Ball Collection System (multi-robot)                              │
├────────────────────────────────────────────────────────────────────────────────────────────────────┤
│                                                                                                    │
│  ┌─────────────┐    ┌────────────┐    ┌─────────────────────────┐    ┌──────────────────────────┐    │
│  │   Gazebo    │    │   SLAM     │    │        Nav2 Stack       │    │   MPPI / LL Controller   │    │
│  │  Simulation │───▶│  Toolbox   │───▶│  (Planner + Behavior)   │───▶│  (Ackermann-aware CMD)   │    │
│  └─────────────┘    └────────────┘    └─────────────────────────┘    └────────────┬─────────────┘    │
│        │                  │                      │                             │ /ballvacX/cmd_vel     │
│        │ /scan            │ /map                 │ /nav_goals                  ▼                       │
│        │ /odom            │ map → odom (TF)      │                             ┌────────────────────┐  │
│        ▼                  ▼                      ▼                             │    robot (wheel/   │  │
│  ┌──────────────────────────────────────────────────────────────────────────────┐  │ steering)         │  │
│  │                          nav_ball_collector_node (per-robot)                │  └────────────────────┘  │
│  │  ┌─────────┐  ┌───────────┐  ┌───────────┐  ┌───────────┐   ┌──────────┐                        │
│  │  │  IDLE   │→ │ EXPLORING │→ │NAVIGATING │→ │APPROACHING│→  │COLLECTING│←──────────────────────────┤
│  │  └─────────┘  └───────────┘  └───────────┘  └───────────┘   └──────────┘                           │
│  │       ↑                                           ↓                                                   │
│  │  ┌──────────┐                              ┌───────────                                                  │
│  │  │RECOVERING│ ←──────────────────────────│ | COLLECTING│                                                 │
│  │  └──────────┘                              └───────────┘                                                 │
│  └──────────────────────────────────────────────────────────────────────────────────────────────────────────┘
│        ▲                                                                                                   │
│        │ /ball_detections                                                                                   │
│  ┌─────────────┐       ┌────────────────────────────────────────────┐      /ball_positions                      │
│  │    Ball     │◀──────│  ball_launcher_node (spawns & publishes)   │◀────┐  (ground-truth from launcher)      │
│  │  Perception │  /camera/front_raw  └────────────────────────────────┘     │                                     │
│  └─────────────┘                                                           │  /ball_claimed (claims)               │
│                                                                            └─────────────────────────────────────▶│
└────────────────────────────────────────────────────────────────────────────────────────────────────┘
```

Notes:
- `/ballvacX/cmd_vel` is produced by the MPPI / low-level controller and consumed by each robot's base controller (namespace `ballvac1`, `ballvac2`, ...).
- TF: SLAM publishes `map` → `odom` transform; robot odom provides `odom` → `base_link`. Ensure correct namespacing per robot.

ball_launcher_node (summary):
- Spawns and removes ball entities in the Ignition world.
-- Publishes `/ball_positions` with ground-truth poses for reliable collection and debugging.
-- Optionally accepts collection requests (service/topic) from collectors to delete a spawned entity when collection succeeds.
- Works as the single source-of-truth for simulated ball state; collector nodes use its data to validate perception-based detections before deletion.

Namespaces & TF (multi-robot):
- Each robot runs under a namespace: `ballvacX` (e.g. `ballvac1`). Topics and TF frames are namespaced accordingly (`/ballvac1/odom`, `/ballvac1/scan`, `/ballvac1/cmd_vel`).
- SLAM produces the `map` frame; `map` → `odom` transform should be published for each robot's localization. Confirm per-robot frame remapping if running multiple SLAM instances.


Key improvements and rationale
-----------------------------

1) Costmap and footprint safety
  - Inflation radius increased to 0.55 m to give earlier obstacle margins for the Ackermann vehicle's turning constraints.
  - Local costmap resized to improve look-ahead for steering maneuvers (recommended 6 × 6 m in tight areas).
  - Added `sensor_frame: "ballvac"` to obstacle layer observations and enabled footprint-based checks so the cost critics evaluate true robot geometry.

2) MPPI & controller tuning
  - ObstaclesCritic included to explicitly penalize trajectories that collide with the costmap.
  - Set `consider_footprint: true` in the CostCritic to reject trajectories intersecting the robot footprint.
  - Added Ackermann-specific constraints (minimum turning radius, steering rate limits) and reduced max velocities for improved stability during approach.

3) Behavior and approach logic
  - Gap-finding exploration: LiDAR-based local gap detection steers exploration away from dense clutter.
  - Obstacle-aware approach: when approaching a ball, the collector checks for obstacles on the direct line-of-sight and switches to a reactive steering behavior if needed.
  - Improved stuck detection: timeouts and travel-distance checks trigger multi-phase recovery rather than immediate halt.

4) Recovery strategies
  - Integrated Nav2 recovery behaviors (spin, backup, wait) combined with custom multi-phase escape maneuvers for corner cases.
  - Blocked-direction memory prevents repeated attempts on a known blocked heading.

Build and prerequisites
-----------------------

1. Source ROS 2:

```bash
source /opt/ros/humble/setup.bash
```

2. Workspace and build (example):

```bash
mkdir ./project & cd ./project & mkdir ./src & cd ./src
git clone https://github.com/ahmetcnh/ackermann-ballvac.git
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

Running the navigation stack
----------------------------

Option A — Full integrated launch (recommended for demos):

```bash
ros2 launch ballvac_ball_collector ball_collection_full.launch.py
# optional: use_rviz:=false  world_name:=custom_world
```

Option B — Step-by-step for development:

Terminal 1 — Simulation + spawn:

```bash
ros2 launch ballvac_bringup ball_arena_spawn.launch.py
```

Terminal 2 — Start Nav2 and collector for robot 1:

```bash
ros2 launch ballvac_ball_collector nav_ball_collect.launch.py slam:=True
```

Terminal 3 — (optional) Monitor detections:

```bash
ros2 topic echo /ballvac1/ball_detections
```

Validation checklist
--------------------

- Robot respects obstacles: MPPI + costmap prevents driving through obstacles.
- Robot does not freeze: Nav2 path planner and recovery behaviors replan and unstick.
- Robot approaches balls while mapping: perception triggers Nav2 goal to ball pose.
- Ball near obstacle: approach uses obstacle-aware steering to avoid contact.

Important configuration files
---------------------------

- `config/nav2_ball_collector_params.yaml` — Nav2 servers and costmap settings.
- `config/nav2_multi_robot_params.yaml` — Multi-robot lifecycle and namespace usage.
- `config/mpii_ackermann_params.yaml` — MPPI / controller tuning.
- `launch/nav_ball_collect.launch.py` — Launch example for a robot collector.

Representative parameter snippets
--------------------------------

Controller limits (nav2_ball_collector_params.yaml):

```yaml
controller_server:
  FollowPath:
    vx_max: 0.7
    vx_min: -0.4
    wz_max: 1.5
```

Costmap inflation (local_costmap):

```yaml
local_costmap:
  local_costmap:
    inflation_layer:
      inflation_radius: 0.55
      cost_scaling_factor: 3.0
```

Approach/collection params (launch args / node params):

```yaml
approach_speed: 0.45
obstacle_stop_m: 0.35
collect_distance_m: 0.35
```

Troubleshooting
---------------

Robot is frozen / not moving:
- Verify TF frames and clock: `ros2 run tf2_tools view_frames` and `ros2 topic hz /clock` (if using simulated time).
- Ensure `/scan` and `/odom` topics publish: `ros2 topic hz /scan` and `ros2 topic hz /odom`.
- Check Nav2 lifecycle states and transitions: `ros2 lifecycle get /lifecycle_manager_navigation` and topic `/lifecycle_manager_navigation/transition_event`.

Nav2 goals rejected:
- Ensure SLAM has produced an initial map or start with a static map for testing.
- Inspect `bt_navigator` logs and increase `wait_for_service_timeout` if services start slowly.

Oscillatory or aggressive steering:
- Reduce `steering_gain` in controller parameters.
- Lower `vx_max` and reduce `max_steer` to smooth motion.

Testing & metrics
-----------------

- Use `ros2 topic hz` to measure sensor rates and ensure components run at expected frequencies.
- Add simple integration tests that spawn a single robot and assert `/ballvacX/cmd_vel` is published when a ball is detected.

Where to look next
------------------

- For tuning: `config/nav2_ball_collector_params.yaml` and MPPI parameter file.
- For behavior logic: `src/nav_ball_collector_node.cpp` and `include/ballvac_ball_collector/nav_ball_collector_node.hpp`.
- For perception: `src/ball_perception_node.cpp` and its parameter YAML in `config/`.

License
-------

See repository `LICENSE` for terms.

