cmake_minimum_required(VERSION 3.5)

project(ballvac_bringup)

find_package(ament_cmake REQUIRED)
find_package(ballvac_description REQUIRED)

install(
  DIRECTORY
    launch
    maps
    rviz
    config
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python scripts
install(
  PROGRAMS
    scripts/odom_to_tf_node.py
  DESTINATION lib/${PROJECT_NAME}
)
  
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
