cmake_minimum_required(VERSION 3.5)
project(offboard_single_position)

# Default to C++14
set(CMAKE_CXX_STANDARD 14)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)

include_directories(include)

add_executable(offboard_single_position src/offboard_single_position.cpp)

ament_target_dependencies(offboard_single_position
  rclcpp
  geometry_msgs
  mavros_msgs
  nav_msgs
  tf2_geometry_msgs
  action_msgs
  tf2
)

install(TARGETS
  offboard_single_position
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
