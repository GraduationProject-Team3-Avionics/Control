cmake_minimum_required(VERSION 3.8)
project(system_identification)

find_package(ament_cmake REQUIRED)

# Install Python executables
install(
  PROGRAMS
    scripts/vz_step_commander.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
