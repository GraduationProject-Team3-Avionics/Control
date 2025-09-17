// offboard_adapter.cpp (skeleton — PID-first)
// Role: Convert ENU(odom) velocity setpoints to PX4 Offboard messages (NED/FRD),
//       send heartbeat, and manage arm/mode commands.
// Scope: Velocity mode only (vx, vy, vz in ENU odom; yaw_rate about Up).
// No implementation yet. Intended interfaces:
//   Sub (internal ENU bus):
//     - geometry_msgs::msg::TwistStamped  /uav/ref/twist   (frame_id=odom)
//       * linear: vx, vy, vz in ENU(odom)
//       * angular.z: yaw_rate in ENU(odom) (rad/s)
//   Pub (PX4):
//     - px4_msgs::msg::OffboardControlMode  /fmu/in/offboard_control_mode
//     - px4_msgs::msg::TrajectorySetpoint   /fmu/in/trajectory_setpoint
//     - px4_msgs::msg::VehicleCommand       /fmu/in/vehicle_command
//   Params:
//     - rate: 20-50 Hz
//     - auto_arm: bool, auto_offboard: bool
//     - frame_world (= "odom")
//   Notes:
//     - World transform ENU→NED for velocities
//       [vx_n, vy_n, vz_n] = [vy_e, vx_e, −vz_e]
//     - Yaw-rate: yawspeed_NED = − yawspeed_ENU
