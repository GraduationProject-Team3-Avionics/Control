// waypoint_manager.cpp
// Role: Manage goal(s) defined in ENU map frame, transform to odom at runtime,
//       and publish a local reference for controllers.

#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class WaypointManager : public rclcpp::Node {
public:
  WaypointManager()
  : Node("waypoint_manager")
  {
    frame_world_ = this->declare_parameter<std::string>("frame_world", "odom");
    rate_hz_ = this->declare_parameter<double>("rate", 10.0);
    track_path_ = this->declare_parameter<bool>("track_path", false);
    lookahead_m_ = this->declare_parameter<double>("lookahead", 1.0);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    sub_goal_map_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/uav/goal_map", 10,
      std::bind(&WaypointManager::onGoalMap, this, std::placeholders::_1));

    sub_path_map_ = this->create_subscription<nav_msgs::msg::Path>(
      "/uav/path_map", 10,
      std::bind(&WaypointManager::onPathMap, this, std::placeholders::_1));

    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/uav/odom", 20,
      std::bind(&WaypointManager::onOdom, this, std::placeholders::_1));

    pub_goal_odom_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/uav/goal_odom", 10);
    pub_path_odom_ = this->create_publisher<nav_msgs::msg::Path>(
      "/uav/path_odom", 10);

    auto period = std::chrono::duration<double>(1.0 / std::max(1.0, rate_hz_));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&WaypointManager::onTick, this));

    RCLCPP_INFO(this->get_logger(), "waypoint_manager started: frame_world=%s, rate=%.1f, track_path=%s, lookahead=%.2f m",
                frame_world_.c_str(), rate_hz_, track_path_ ? "true" : "false", lookahead_m_);
  }

private:
  void onGoalMap(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    last_goal_map_ = *msg;
    // Allow odom-framed input as well (pass-through)
    publishGoalOdomUnlocked();
  }

  void onPathMap(const nav_msgs::msg::Path::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    last_path_map_ = *msg;
    // Transform and publish immediately for visualization
    computePathOdomUnlocked();
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    last_odom_ = *msg;
  }

  void onTick()
  {
    std::lock_guard<std::mutex> lk(mutex_);
    if (track_path_ && !last_path_map_.poses.empty() && haveOdom()) {
      computePathOdomUnlocked();
      geometry_msgs::msg::PoseStamped target;
      if (selectLookaheadTargetUnlocked(path_odom_, target)) {
        pub_goal_odom_->publish(target);
        return;
      }
    }
    // Fallback: transform last goal
    publishGoalOdomUnlocked();
  }

  bool haveOdom() const {
    return last_odom_.header.frame_id.size() > 0;
  }

  // Transform latest goal to odom and publish
  void publishGoalOdomUnlocked()
  {
    if (last_goal_map_.header.frame_id.empty()) return;
    geometry_msgs::msg::PoseStamped odom_goal;
    if (!toOdom(last_goal_map_, odom_goal)) return;
    pub_goal_odom_->publish(odom_goal);
  }

  // Transform path to odom and publish/store
  void computePathOdomUnlocked()
  {
    if (last_path_map_.header.frame_id.empty()) return;
    path_odom_.header.stamp = this->now();
    path_odom_.header.frame_id = frame_world_;
    path_odom_.poses.clear();
    path_odom_.poses.reserve(last_path_map_.poses.size());
    for (const auto &p : last_path_map_.poses) {
      geometry_msgs::msg::PoseStamped out;
      if (toOdom(p, out)) {
        path_odom_.poses.push_back(out);
      }
    }
    if (!path_odom_.poses.empty()) {
      pub_path_odom_->publish(path_odom_);
    }
  }

  bool toOdom(const geometry_msgs::msg::PoseStamped &in,
              geometry_msgs::msg::PoseStamped &out)
  {
    try {
      if (in.header.frame_id == frame_world_) {
        out = in;
        return true;
      }
      auto tf = tf_buffer_->lookupTransform(frame_world_, in.header.frame_id, tf2::TimePointZero);
      tf2::doTransform(in, out, tf);
      out.header.stamp = this->now();
      out.header.frame_id = frame_world_;
      return true;
    } catch (const std::exception &e) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "toOdom failed from '%s' to '%s': %s",
                           in.header.frame_id.c_str(), frame_world_.c_str(), e.what());
      return false;
    }
  }

  bool selectLookaheadTargetUnlocked(const nav_msgs::msg::Path &path,
                                     geometry_msgs::msg::PoseStamped &target)
  {
    if (path.poses.empty() || !haveOdom()) return false;
    const auto &p = last_odom_.pose.pose.position;
    const double cx = p.x, cy = p.y, cz = p.z;
    // Find first pose at least lookahead_m_ away
    for (const auto &ps : path.poses) {
      const auto &q = ps.pose.position;
      const double dx = q.x - cx;
      const double dy = q.y - cy;
      const double dz = q.z - cz;
      const double d = std::sqrt(dx*dx + dy*dy + dz*dz);
      if (d >= lookahead_m_) {
        target = ps;
        target.header.stamp = this->now();
        return true;
      }
    }
    // Otherwise use last
    target = path.poses.back();
    target.header.stamp = this->now();
    return true;
  }

private:
  // Params
  std::string frame_world_ {"odom"};
  double rate_hz_ {10.0};
  bool track_path_ {false};
  double lookahead_m_ {1.0};

  // TF
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  // I/O
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_map_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_map_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal_odom_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_odom_;
  rclcpp::TimerBase::SharedPtr timer_;

  // State
  std::mutex mutex_;
  geometry_msgs::msg::PoseStamped last_goal_map_;
  nav_msgs::msg::Path last_path_map_;
  nav_msgs::msg::Path path_odom_;
  nav_msgs::msg::Odometry last_odom_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointManager>());
  rclcpp::shutdown();
  return 0;
}

