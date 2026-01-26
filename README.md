# Flight Control for Quadcopter

- I am planning to make PID Controller.
- Then I am planning to make MPC Controller.

# Issues

### **lqr_controller** 패키지 빌드 오류 문제 해결방법 (260126)
- `lqr_controller` 패키지는 Interface 패키지의 `px4_msgs`에 의존함.
```bash
CMakeLists.txt의 find_package(px4_msgs REQUIRED)
```

- colcon으로 빌드할 때, CMake는 의존 패키지를 찾아야 하는데, 그 위치를 환경 변수(CMAKE_PREFIX_PATH)로 알려줘야 함.

- 따라서 `px4_msgs` 먼저 빌드하고 아래의 명령어로 워크스페이스 환경설정해주고 `lqr_controller` 빌드해줘야 함.
```bash
source install/setup.bash
```

### 깃클론 문제 (260126)
- `How to Clone.md` 확인 바람.