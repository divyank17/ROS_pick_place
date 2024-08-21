cmake_minimum_required(VERSION 3.5)
project(pick_and_place)

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(pick_and_place_node src/pick_and_place.cpp)

# Link libraries
ament_target_dependencies(pick_and_place_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
)

install(TARGETS
  pick_and_place_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
