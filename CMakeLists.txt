cmake_minimum_required(VERSION 3.8)
project(custom_dwa_planner)

# --- Required Packages ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pluginlib REQUIRED)  
find_package(nav2_core REQUIRED)  #Required if you're implementing nav2_core::Controller

# --- Include Directories ---
include_directories(include)

# --- Executables ---
add_executable(dwa_planner_node src/dwa_planner_node.cpp)
ament_target_dependencies(dwa_planner_node
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  visualization_msgs
  angles
)

# --- Shared Library for Plugin ---
add_library(custom_dwa_planner SHARED
  src/dwa_planner_node.cpp
)
ament_target_dependencies(custom_dwa_planner
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  visualization_msgs
  nav2_core
  angles
)
install(TARGETS
  custom_dwa_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# --- Install Targets ---
install(TARGETS
  dwa_planner_node
  custom_dwa_planner
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

#

ament_package()
