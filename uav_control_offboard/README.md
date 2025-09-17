# uav_control_offboard (Scaffold)

Purpose
- High-level UAV controller focused on PID-first waypoint following with PX4 Offboard.
- No code yet — documents interfaces, frames, and parameters (PID velocity mode).

Frames & Coordinates (project contract)
- TF: `map → odom` (yaw-only, fixed at start), `odom → base_link` (dynamic), `base_link → rotor_i` (static)
- ENU/FLU inside ROS. Convert to NED/FRD only at PX4 boundary.
- Controller subscribes to `/uav/odom` (`nav_msgs/Odometry`, ENU, `odom`→`base_link`).

I/O Contracts
- Input (state): `/uav/odom`
- Input (goal): waypoint/trajectory in ENU `map` → converted to `odom` at runtime
- Output (PX4 Offboard): `/fmu/in/offboard_control_mode`, `/fmu/in/trajectory_setpoint`, `/fmu/in/vehicle_command`

Output Mode (Phase 1: PID)
- Velocity setpoint only: vx, vy, vz (ENU odom), yaw_rate
  - Convert to NED/FRD in offboard_adapter before publishing

ENU ↔ NED Quick Map
- World: [x_n, y_n, z_n] = [y_e, x_e, −z_e]
- FLU ↔ FRD: [x, y, z]_FRD = [x, −y, −z]_FLU
- yaw_NED = π/2 − yaw_ENU, yawspeed_NED = − yawspeed_ENU

Launch & Config (placeholders)
- Launch: `launch/offboard_waypoint_demo.launch.py`
- Params: `config/pid_defaults.yaml`

Roadmap
- waypoint_manager (map→odom)
- follower_pid (outer loop PID → velocity setpoint)
- offboard_adapter (ENU→NED/FRD conversion + PX4 I/F)
