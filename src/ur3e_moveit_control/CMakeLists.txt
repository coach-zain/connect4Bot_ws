cmake_minimum_required(VERSION 3.8)
project(ur3e_moveit_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ── Pick and place executable ─────────────────────────
add_executable(pick_and_place src/pick_and_place.cpp)
ament_target_dependencies(
  pick_and_place
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  std_msgs
)

# ── Pose printer executable ───────────────────────────
add_executable(print_current_pose src/print_current_pose.cpp)
ament_target_dependencies(
  print_current_pose
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
)

# ── Install executables ───────────────────────────────
install(
  TARGETS
    pick_and_place
    print_current_pose
  DESTINATION lib/${PROJECT_NAME}
)

# Optional launch folder
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()