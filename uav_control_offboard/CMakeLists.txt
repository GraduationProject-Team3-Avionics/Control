cmake_minimum_required(VERSION 3.8)
project(uav_control_offboard)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# No targets yet — this package currently ships launch/config/README only.

add_executable(waypoint_manager src/waypoint_manager.cpp)
ament_target_dependencies(waypoint_manager rclcpp geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs)

install(TARGETS
  waypoint_manager
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
