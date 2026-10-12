################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(crs_multibot)


################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)



# install(TARGETS
#   talker
################################################################################
# Install
################################################################################
install(DIRECTORY launch models rviz urdf worlds params scripts
  DESTINATION share/${PROJECT_NAME}/
)


install(DIRECTORY config/
   DESTINATION share/${PROJECT_NAME}/config
)
install(PROGRAMS
  scripts/goal_listener_and_sender.py
  scripts/spawn.py
  scripts/fleet_mission_controller.py
  scripts/formation_geometry.py
  scripts/audit_fleet.py
  scripts/validate_fleet_navigation.py
  scripts/validate_formation_modes.py
  scripts/generate_formation_scenario.py
  scripts/validate_fleet_mission.py
  scripts/fleet_rviz_relay.py
  scripts/fleet_live_status.py
  scripts/fleet_status_text.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/multi_robot.dsv.in")

ament_package()
