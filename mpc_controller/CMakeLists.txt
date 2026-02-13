# ──────────────────────────────────────────────
# ── 새 노드를 추가할 때 건드려야 할 곳마다 표시 ──
cmake_minimum_required(VERSION 3.18)
project(mpc_controller)

# ---- C++ 표준 ----
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# ---- 패키지 의존성 ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)                # 파이썬 노드 있을 때도 선언
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)            # PX4 메시지

# 필요한 패키지 추가 시 여기에 find_package() 추가

# ============================================================
# 1. C++ 노드를 추가하고 싶다면
#    ① add_executable / ② ament_target_dependencies / ③ install(TARGETS)에 등록
# ============================================================

# ★새로운 C++ 노드 예시★
# add_executable(mpc_node src/mpc_node.cpp)
# ament_target_dependencies(mpc_node
#   rclcpp std_msgs geometry_msgs nav_msgs px4_msgs)

# -------------------- Python 노드 --------------------
install(PROGRAMS
  scripts/trajectory_resampler_node.py       # 경로 리샘플링 노드
  scripts/mpc_position_controller_node.py    # MPC 위치 제어 노드
  DESTINATION lib/${PROJECT_NAME})

# -------------------- C++ 바이너리 설치 ----------------
install(TARGETS
  # mpc_node                            # 위 add_executable() 했으면 여기도 추가
  DESTINATION lib/${PROJECT_NAME})

# -------------------- 기타 리소스(launch, config 등) -----
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME})

ament_package()
