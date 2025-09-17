# How To Play — Waypoint (Debug Near Origin)

사전 실행(둘 중 하나)
- 런치로 실행:
```
ros2 launch uav_control_offboard offboard_waypoint_demo.launch.py
```
- 또는 노드만 실행:
```
ros2 run uav_control_offboard waypoint_manager
```

확인
- 변환된 목표 확인: `ros2 topic echo /uav/goal_odom --once` (frame_id=odom)
- RViz: Fixed Frame=map, Pose 디스플레이에 `/uav/goal_odom` 추가

---

## A. 디버깅용 “원점 부근” 명령 (map 프레임)

- (0, 0, 1.5), yaw=0
```
ros2 topic pub /uav/goal_map geometry_msgs/PoseStamped "{header: {frame_id: map}, pose: {position: {x: -115.0, y: 40.0, z: 1.5}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

- (2, 0, 1.5), yaw=0
```
ros2 topic pub /uav/goal_map geometry_msgs/PoseStamped "{header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0, z: 1.5}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

- (0, 2, 1.5), yaw=+90° (π/2)
```
ros2 topic pub /uav/goal_map geometry_msgs/PoseStamped "{header: {frame_id: map}, pose: {position: {x: 0.0, y: 2.0, z: 1.5}, orientation: {x: 0.0, y: 0.0, z: 0.7071068, w: 0.7071068}}}"
```

- (2, 2, 1.5), yaw=−90° (−π/2)
```
ros2 topic pub /uav/goal_map geometry_msgs/PoseStamped "{header: {frame_id: map}, pose: {position: {x: 2.0, y: 2.0, z: 1.5}, orientation: {x: 0.0, y: 0.0, z: -0.7071068, w: 0.7071068}}}"
```

## B. odom 프레임으로 직접 발행(패스스루)

- (2, 1, 1.5), yaw=0 (frame_id=odom)
```
ros2 topic pub /uav/goal_map geometry_msgs/PoseStamped "{header: {frame_id: odom}, pose: {position: {x: 2.0, y: 1.0, z: 1.5}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

## C. 주기 퍼블리시(5 Hz)
```
ros2 topic pub -r 5 /uav/goal_map geometry_msgs/PoseStamped "{header: {frame_id: map}, pose: {position: {x: 0.0, y: 0.0, z: 1.5}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

메모
- yaw만 바꿀 때: orientation은 x=y=0, z=sin(yaw/2), w=cos(yaw/2)
- waypoint_manager가 어떤 프레임이든 odom으로 변환해 `/uav/goal_odom`으로 발행합니다.

