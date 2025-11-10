cmake_minimum_required(VERSION 3.8)
project(lqr_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(lqr_z_controller src/lqr_z_controller_node.cpp)
ament_target_dependencies(lqr_z_controller rclcpp nav_msgs px4_msgs std_msgs geometry_msgs visualization_msgs)

install(TARGETS
  lqr_z_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
