# uav_control_pid (ROS 2 × PX4 Offboard)

ROS 2 Humble에서 PX4 Offboard 제어를 실험·학습하기 위한 상위제어 패키지입니다. 이 패키지는 `/uav/odom`(ENU/FLU)을 단일 상태 입력으로 사용하고, PX4로 속도/위치 세트포인트를 지속 스트리밍합니다.

## 핵심 기능
- Offboard 호버 데모(`offboard_hover`): 현재 위치에서 Z만 `climb_m` 만큼 상승 후 호버(위치 모드)
- PID GOTO 데모(`offboard_pid_goto`): ENU에서 PID로 vx,vy,vz,yawspeed 생성 → NED 변환 후 PX4로 전송(속도 모드)
- 시각화 토픽: 목표 Pose(`/uav_control/goal_pose`), RViz Marker(`/uav_control/goal_marker`)
- 프레임 일치 옵션: odom↔map yaw 보정 후 ENU→NED 매핑

## 요구 사항
- ROS 2 Humble, PX4 uXRCE-DDS, PlotJuggler/RViz2(옵션)
- 상태/TF 파이프라인 실행: `gazebo_env_setup` 패키지의 런치 사용

## 파일 맵(핵심)
- 노드
  - `uav_control_pid/offboard_hover.py`
  - `uav_control_pid/offboard_pid_goto.py`
- 런치
  - `launch/offboard_hover.launch.py`
  - `launch/offboard_pid_goto.launch.py`
- 설정(YAML)
  - `config/offboard_hover.yaml`
  - `config/offboard_pid_goto.yaml`
- 패키지 메타: `package.xml`, `setup.py`, `setup.cfg`, `resource/uav_control_pid`

## 실행 순서(시뮬 기준)
1) 빌드/소스
```
colcon build --packages-select gazebo_env_setup uav_control_pid px4_msgs
source install/setup.bash
```
2) 브리지/TF/상태
```
ros2 launch gazebo_env_setup topic_bridge.launch.py
ros2 launch gazebo_env_setup pose_tf_broadcaster.launch.py
```
3) uXRCE Agent + PX4
```
MicroXRCEAgent udp4 -p 8888
# PX4 SITL/실기 실행 및 Agent 연결 확인
```
4) 제어 노드
```
# 호버(위치 모드)
ros2 launch uav_control_pid offboard_hover.launch.py

# PID GOTO(속도 모드)
ros2 launch uav_control_pid offboard_pid_goto.launch.py \
  goal_x:=0.0 goal_y:=0.0 goal_z:=2.0 goal_yaw:=0.0
```

## 좌표계·Offboard 인터페이스
- 상태 입력: `/uav/odom`(ENU/FLU)
- Offboard 명령: PX4 NED/FRD
  - ENU→NED: `x_ned=y_enu`, `y_ned=x_enu`, `z_ned=-z_enu`
  - yaw: `yaw_ned = pi/2 - yaw_enu`, yaw rate: `yawspeed_ned = - yawspeed_enu`
- 스트리밍: `OffboardControlMode`(velocity 또는 position), `TrajectorySetpoint`, `VehicleCommand`(ARM/Offboard)
- 안전: 10 Hz 이상(권장 50 Hz) 연속 퍼블리시 없으면 Offboard Lost 발생

## 노드별 설명
### offboard_hover
- 기능: 첫 `/uav/odom`을 기준으로 XY 유지, Z=`climb_m` 상승, yaw 유지(위치 세트포인트)
- 주요 파라미터(`config/offboard_hover.yaml`)
  - `rate_hz`(50.0), `climb_m`(2.0), `arm_on_start`(true), `warmup_sec`(0.5)
- 퍼블리시
  - `/fmu/in/offboard_control_mode`(position 사용), `/fmu/in/trajectory_setpoint`(position), `/fmu/in/vehicle_command`

### offboard_pid_goto
- 기능: ENU(odom)에서 목표–현재 오차 → 축별 PID로 속도/각속도 생성 → (옵션) odom→map yaw 보정 → ENU→NED 변환 → PX4에 속도+yawspeed 전송
- 주요 파라미터(`config/offboard_pid_goto.yaml`)
  - 목표: `goal_x,y,z,yaw`
  - PID: `kp_xy,kd_xy,ki_xy`, `kp_z,kd_z,ki_z`, `kp_yaw,kd_yaw,ki_yaw`(기본 I,D=0)
  - 제한: `max_speed_xy,max_speed_z,max_yawspeed`
  - 기타: `apply_odom_to_map_yaw_correction`(true)
- 시각화
  - `/uav_control/goal_pose`(PoseStamped, frame=odom), `/uav_control/goal_marker`(Marker/ARROW)
- 퍼블리시
  - `/fmu/in/offboard_control_mode`(velocity 사용), `/fmu/in/trajectory_setpoint`(velocity, yawspeed), `/fmu/in/vehicle_command`

## 튜닝 가이드(요약)
- 순서: Hover 안정화 → Z → XY → Yaw
- 초기 추천(시뮬): `kp_xy=0.6~1.0`, `kp_z=0.8~1.2`, `kp_yaw=0.8~1.2` (I,D는 0부터)
- 포화: `max_speed_xy=0.6~1.0`, `max_speed_z=0.5~0.8`, `max_yawspeed=0.6~1.0`
- 필요 시 I,D 추가, 적분 클램프/미분 저역통과(향후 옵션)

## 트러블슈팅
- 로터가 돌다 멈춤(Offboard 이탈)
  - `ros2 topic hz /fmu/in/trajectory_setpoint`와 `/uav/odom`, `/clock` 주기 확인(≥10 Hz)
  - uXRCE Agent 로그/연결 점검, 스트림 예열 후 ARM→Offboard 시퀀스 준수
- XY가 원을 그리듯 이상
  - `apply_odom_to_map_yaw_correction:=true`로 odom↔map yaw 보정 적용
- `/uav/odom` 없음
  - `pose_tf_broadcaster.launch.py` 병행 실행, TF 조회 확인

## 향후 계획
- 마지막 유효 세트포인트 홀드 및 진단 토픽(`/uav_control/diag/*`)
- PID 확장: 적분 클램프, 미분 저역통과
- MPC 데모 추가(OSQP, 상태=[p,v], 입력=a, 제약 a_xy/z/틸트)

## 참고 경로
- 상태 브리지: `src/Interface/gazebo_env_setup/src/state_bridge.cpp`
- TF 브로드캐스터: `src/Interface/gazebo_env_setup/src/pose_tf_broadcaster.cpp`

---
