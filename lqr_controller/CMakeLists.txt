cmake_minimum_required(VERSION 3.8)
project(lqr_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# ──────────────── C++ 노드 ────────────────

# C++ node for 1st order LQR Z controller
add_executable(lqr_z1_controller src/lqr_z1_controller_node.cpp)
ament_target_dependencies(lqr_z1_controller rclcpp nav_msgs px4_msgs std_msgs geometry_msgs visualization_msgs)

# C++ node for 2nd order LQR Z controller
add_executable(lqr_z2_controller src/lqr_z2_controller_node.cpp)
ament_target_dependencies(lqr_z2_controller rclcpp nav_msgs px4_msgs std_msgs geometry_msgs visualization_msgs)

# C++ node for 2nd order LQR X controller
add_executable(lqr_x2_controller src/lqr_x2_controller_node.cpp)
ament_target_dependencies(lqr_x2_controller rclcpp nav_msgs px4_msgs std_msgs geometry_msgs visualization_msgs)

install(TARGETS
  lqr_z1_controller
  lqr_z2_controller
  lqr_x2_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
