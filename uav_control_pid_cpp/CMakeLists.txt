cmake_minimum_required(VERSION 3.8)
project(uav_control_pid_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(px4_msgs REQUIRED)

add_executable(offboard_pid_goto_cpp src/offboard_pid_goto.cpp)
ament_target_dependencies(offboard_pid_goto_cpp
  rclcpp nav_msgs geometry_msgs visualization_msgs tf2_ros tf2 px4_msgs)

install(TARGETS
  offboard_pid_goto_cpp
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

