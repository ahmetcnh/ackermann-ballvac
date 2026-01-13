# BallVac — Multi-Robot Autonomous Ball Collection System

Overview
--------

BallVac is a research & demo ROS 2 project that simulates autonomous ball collection in an Ignition Gazebo (Fortress) arena using Ackermann-steered robots. The system integrates perception, planning, multi-robot coordination, and simulation entity management to provide a reproducible baseline for experiments in multi-robot task allocation, constrained steering control, and autonomous behavior design.

Repository layout
-----------------

- `ballvac_ball_collector/` — Collector behaviors, perception, and fleet coordination nodes.
- `ballvac_bringup/` — Launch files and system bring-up for simulation and navigation.
- `ballvac_control/` — MPPI controller and Ackermann-aware low-level control.
- `ballvac_description/` — Robot and world models for Ignition Gazebo.
- `ballvac_msgs/` — Custom messages and services used by the fleet.

Key features
------------

- Multi-robot fleet: three independent Ackermann robots, each with its own navigation stack and localization.
- Vision-based ball detection and prioritized target selection.
- Claiming protocol (`/fleet/ball_claimed`) to avoid duplicate collection attempts.
- Ground-truth ball publisher (`/fleet/ball_positions`) enabling accurate removal of simulated entities.
- Ackermann-aware MPPI controller and tuned costmaps to support safe maneuvering in cluttered arenas.

Requirements
------------

- Ubuntu 22.04
- ROS 2 Humble (or compatible)
- Ignition Gazebo Fortress with `ros_gz` bridge
- Common ROS packages: `nav2`, `slam_toolbox`, `robot_localization`, `xacro`, `tf2_ros`

Build and install
-----------------

From the workspace root (e.g., `~/bjk`):

```bash
# Install ROS dependencies declared by packages in src
rosdep install --from-paths src --ignore-src -r -y

# Build core packages used for simulation
colcon build --packages-select ballvac_ball_collector ballvac_description ballvac_bringup
source install/setup.bash
```

Running the system
------------------

Start the integrated simulation (world, fleet, navigation, ball manager):

```bash
ros2 launch ballvac_ball_collector ball_collection_full.launch.py
```

Launch options and notes:

- Pass `use_rviz:=false` to disable RViz.
- Change `world_name:=...` to load a custom world.
- For development, run the simulation first, then start navigation nodes separately to speed iteration.

Modular run (recommended for debugging)
--------------------------------------

1. Start the simulation and spawn robots:

```bash
ros2 launch ballvac_bringup ball_arena_spawn.launch.py
```

2. Start a robot's collector and navigation stack (example):

```bash
ros2 launch ballvac_ball_collector nav_ball_collect.launch.py slam:=True
```

Architecture and components
---------------------------

Nodes and their roles:

- `nav_ball_collector_node` (per robot): Behavior finite-state machine managing exploration, navigation, approach, collection, and recovery. Interfaces with Nav2 for path planning and the MPPI controller for low-level steering.
- `ball_perception_node`: Processes camera images to detect colored balls and publishes detection messages with position estimates.
- `ball_launcher_node`: Spawns ball entities in Ignition and publishes `/fleet/ball_positions` (ground-truth). It also handles deletion requests when collectors succeed.
- `MPPI Controller` (in `ballvac_control`): Constrained model-predictive controller adapted for Ackermann steering.

Topics and messages
-------------------

- `/fleet/ball_positions` (publisher: launcher): array of ground-truth ball poses.
- `/fleet/ball_claimed` (pub/sub): claim announcements to coordinate collectors.
- `/ballvacX/ball_detections` (publisher: perception): detected ball observations for robot X.
- `/ballvacX/cmd_vel` (subscriber: controller): velocity commands for robot X.

Configuration files
-------------------

Main config files live in `ballvac_ball_collector/config` and `ballvac_bringup/config`.

- `nav2_ball_collector_params.yaml` — Nav2 servers, costmap, and controller settings (tuned for Ackermann vehicles).
- `nav2_multi_robot_params.yaml` — Multi-robot lifecycle and namespace settings.
- `mpii_ackermann_params.yaml` — MPPI controller configuration.

Parameter tuning recommendations
-------------------------------

- Costmap inflation: increase (e.g., 0.55 m) to provide a larger safety buffer in tight environments.
- Controller speed limits: reduce `vx_max` and `wz_max` when testing in cluttered arenas.
- Enable footprint collision checking for the cost critic (`consider_footprint: true`).

Development and contribution
----------------------------

Contributions are welcome. Suggested workflow:

1. Fork the repository and create a feature branch.
2. Add or update code and tests.
3. Run `colcon build` and verify no compile/runtime errors.
4. Open a pull request describing the change and tests.

Testing
-------

- Unit tests: add under each package using `ament` test frameworks.
- Integration tests: add launch tests that bring up a lightweight simulation and assert expected topics/events.

Troubleshooting
---------------

Robot does not move or freezes:
- Check TF frames: `ros2 run tf2_tools view_frames`.
- Monitor `/scan` and `/odom`: `ros2 topic hz /scan`.
- Check Nav2 lifecycle and transitions.

Nav2 goal rejected or oscillating behavior:
- Ensure an initial map is available for SLAM-based runs.
- Lower controller gains or steering_gain parameters.

Ball detection problems:
- Confirm camera topics exist and image encoding is valid.
- Tune color thresholds in perception config for your lighting conditions.

FAQ
---

Q: Can I run more than three robots?
A: The system supports extension to N robots but requires resources and additional namespaces/configuration. See `nav2_multi_robot_params.yaml` for guidance.

Q: Can I use ROS 2 Rolling or a different Ignition release?
A: The code targets Humble + Ignition Fortress. Porting to other releases is possible but may require API adjustments.

License
-------

See the `LICENSE` file in the repository root for license terms. This project is provided primarily for educational and research use.

Contact
-------

Open an issue in the repository for bugs, feature requests, or questions.


