cmake_minimum_required(VERSION 3.8)
project(ballvac_ball_collector)

# Set C++ standard to C++17 for better features
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ballvac_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# ==============================================================================
# Ball Perception Node - Detects colored balls using camera + OpenCV
# ==============================================================================
add_executable(ball_perception_node
  src/ball_perception_node.cpp
)
ament_target_dependencies(ball_perception_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  ballvac_msgs
  cv_bridge
  image_transport
)
target_link_libraries(ball_perception_node
  ${OpenCV_LIBRARIES}
)

# ==============================================================================
# Ball Collector Node - FSM for autonomous ball collection
# ==============================================================================
add_executable(ball_collector_node
  src/ball_collector_node.cpp
)
ament_target_dependencies(ball_collector_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  ballvac_msgs
  ros_gz_interfaces
)

# ==============================================================================
# Ball Launcher Node - Launches balls into the arena
# ==============================================================================
add_executable(ball_launcher_node
  src/ball_launcher_node.cpp
)
ament_target_dependencies(ball_launcher_node
  rclcpp
  rclcpp_components
  std_msgs
  std_srvs
  ros_gz_interfaces
)

# ==============================================================================
# Exploration Collector Node - Explores room then collects balls
# ==============================================================================
add_executable(exploration_collector_node
  src/exploration_collector_node.cpp
)
ament_target_dependencies(exploration_collector_node
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  std_srvs
  ballvac_msgs
  ros_gz_interfaces
  tf2
  tf2_geometry_msgs
)

# ==============================================================================
# Odom TF Publisher - Publishes odom->base_link TF for SLAM
# ==============================================================================
add_executable(odom_tf_publisher
  src/odom_tf_publisher.cpp
)
ament_target_dependencies(odom_tf_publisher
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)

# ==============================================================================
# Nav Ball Collector Node - Uses Nav2 for path-planned ball collection
# ==============================================================================
add_executable(nav_ball_collector_node
  src/nav_ball_collector_node.cpp
)
ament_target_dependencies(nav_ball_collector_node
  rclcpp
  rclcpp_action
  sensor_msgs
  geometry_msgs
  nav_msgs
  nav2_msgs
  std_msgs
  ballvac_msgs
  ros_gz_interfaces
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Install targets
install(
  TARGETS
    ball_perception_node
    ball_collector_node
    ball_launcher_node
    exploration_collector_node
    odom_tf_publisher
    nav_ball_collector_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install Python test scripts
install(
  PROGRAMS scripts/test_steering.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directory
install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
