// follower_pid.cpp (skeleton)
// Role: Outer-loop PID controller that converts ENU odom errors into
//       velocity (and yaw_rate) setpoints.
// No implementation yet. Intended interfaces:
//   Sub:
//     - nav_msgs::msg::Odometry           /uav/odom
//     - geometry_msgs::msg::PoseStamped   /uav/goal_odom
//   Pub (internal ENU bus):
//     - geometry_msgs::msg::TwistStamped  /uav/ref/twist      (frame_id=odom)
//     - std_msgs::msg::Float32            /uav/ref/yaw_rate   (optional)
//   Params:
//     - pid.{x,y,z,yaw} = [P,I,D]
//     - limits.{v_xy,v_z,yaw_rate}
//     - anti_windup (bool), dt (Hz), frame_world (= "odom")

