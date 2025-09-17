// waypoint_manager.cpp (skeleton)
// Role: Manage goal(s) defined in ENU map frame, transform to odom at runtime,
//       and publish a local reference for controllers.
// No implementation yet. Intended interfaces:
//   Sub:
//     - geometry_msgs::msg::PoseStamped  /uav/goal_map        (frame_id=map)
//     - nav_msgs::msg::Path              /uav/path_map        (optional, frame_id=map)
//   Pub:
//     - geometry_msgs::msg::PoseStamped  /uav/goal_odom       (frame_id=odom)
//     - nav_msgs::msg::Path              /uav/path_odom       (optional)
//   Params:
//     - lookahead (m)
//     - use_path (bool)
//     - frame_world (= "odom")

