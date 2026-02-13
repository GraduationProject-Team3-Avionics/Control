// C++ node equivalent of Python OffboardPidGoto
#include <cmath>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

namespace {

double wrap_pi(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

// ENU -> NED velocity mapping
inline void enu_vel_to_ned(double vx_e, double vy_e, double vz_e,
                           double &vx_n, double &vy_n, double &vz_n) {
  vx_n = vy_e;
  vy_n = vx_e;
  vz_n = -vz_e;
}

// ENU yawrate -> NED yawrate mapping
inline double enu_yawspeed_to_ned(double ys_e) { return -ys_e; }

}  // namespace

class OffboardPidGotoCpp : public rclcpp::Node {
 public:
  OffboardPidGotoCpp()
      : rclcpp::Node("offboard_pid_goto"),
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_),
        have_state_(false),
        armed_(false),
        offboard_(false),
        last_cmd_time_sec_(0.0),
        prev_time_set_(false),
        yaw_map_to_odom_(0.0),
        have_yaw_corr_(false) {
    // Parameters
    rate_hz_ = this->declare_parameter<double>("rate_hz", 50.0);
    arm_on_start_ = this->declare_parameter<bool>("arm_on_start", true);
    warmup_sec_ = this->declare_parameter<double>("warmup_sec", 0.5);
    apply_yaw_corr_ =
        this->declare_parameter<bool>("apply_odom_to_map_yaw_correction", true);

    goal_x_ = this->declare_parameter<double>("goal_x", 0.0);
    goal_y_ = this->declare_parameter<double>("goal_y", 0.0);
    goal_z_ = this->declare_parameter<double>("goal_z", 2.0);
    goal_yaw_ = this->declare_parameter<double>("goal_yaw", 0.0);

    kp_xy_ = this->declare_parameter<double>("kp_xy", 0.8);
    ki_xy_ = this->declare_parameter<double>("ki_xy", 0.0);
    kd_xy_ = this->declare_parameter<double>("kd_xy", 0.0);
    kp_z_ = this->declare_parameter<double>("kp_z", 0.8);
    ki_z_ = this->declare_parameter<double>("ki_z", 0.0);
    kd_z_ = this->declare_parameter<double>("kd_z", 0.0);
    kp_yaw_ = this->declare_parameter<double>("kp_yaw", 1.0);
    ki_yaw_ = this->declare_parameter<double>("ki_yaw", 0.0);
    kd_yaw_ = this->declare_parameter<double>("kd_yaw", 0.0);

    max_speed_xy_ = this->declare_parameter<double>("max_speed_xy", 1.0);
    max_speed_z_ = this->declare_parameter<double>("max_speed_z", 0.8);
    max_yawspeed_ = this->declare_parameter<double>("max_yawspeed", 0.8);

    // QoS for odom (best effort, keep last 5)
    auto qos = rclcpp::QoS(rclcpp::KeepLast(5));
    qos.best_effort();
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/uav/odom", qos,
        std::bind(&OffboardPidGotoCpp::cbOdom, this, std::placeholders::_1));

    // Publishers
    pub_offboard_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode", 10);
    pub_traj_ =
        this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);
    pub_cmd_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command", 10);

    pub_goal_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/uav_control/goal_pose", 10);
    pub_goal_marker_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/uav_control/goal_marker", 10);

    start_time_ = this->get_clock()->now();

    // Timer
    const double period = 1.0 / std::max(1.0, rate_hz_);
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(period),
        std::bind(&OffboardPidGotoCpp::tick, this));

    RCLCPP_INFO(get_logger(),
                "Offboard PID goto started @ %.1f Hz, goal=(%.1f,%.1f,%.1f, yaw=%.2f)",
                rate_hz_, goal_x_, goal_y_, goal_z_, goal_yaw_);
  }

 private:
  inline uint64_t now_us() {
    return static_cast<uint64_t>(this->get_clock()->now().nanoseconds() / 1000);
  }

  void cbOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    x_ = msg->pose.pose.position.x;
    y_ = msg->pose.pose.position.y;
    z_ = msg->pose.pose.position.z;
    const double qx = msg->pose.pose.orientation.x;
    const double qy = msg->pose.pose.orientation.y;
    const double qz = msg->pose.pose.orientation.z;
    const double qw = msg->pose.pose.orientation.w;
    // ENU yaw
    const double siny_cosp = 2.0 * (qw * qz + qx * qy);
    const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    yaw_ = std::atan2(siny_cosp, cosy_cosp);
    have_state_ = true;
  }

  void publishOffboardMode() {
    px4_msgs::msg::OffboardControlMode m;
    m.timestamp = now_us();
    m.position = false;
    m.velocity = true;
    m.acceleration = false;
    m.attitude = false;
    m.body_rate = false;
    m.thrust_and_torque = false;
    m.direct_actuator = false;
    pub_offboard_->publish(m);
  }

  void publishVelocitySetpoint(double vx_e, double vy_e, double vz_e,
                               double yawspeed_e) {
    double vx_n, vy_n, vz_n;
    enu_vel_to_ned(vx_e, vy_e, vz_e, vx_n, vy_n, vz_n);
    const double ys_n = enu_yawspeed_to_ned(yawspeed_e);

    px4_msgs::msg::TrajectorySetpoint t;
    t.timestamp = now_us();
    t.position = {NAN, NAN, NAN};
    t.velocity = {static_cast<float>(vx_n), static_cast<float>(vy_n),
                  static_cast<float>(vz_n)};
    t.acceleration = {NAN, NAN, NAN};
    t.jerk = {NAN, NAN, NAN};
    t.yaw = NAN;
    t.yawspeed = static_cast<float>(ys_n);
    pub_traj_->publish(t);
  }

  void sendVehicleCommand(uint16_t command, float param1 = 0.0f,
                          float param2 = 0.0f) {
    px4_msgs::msg::VehicleCommand msg;
    msg.timestamp = now_us();
    msg.param1 = param1;
    msg.param2 = param2;
    msg.param3 = 0.0f;
    msg.param4 = 0.0f;
    msg.param5 = 0.0f;
    msg.param6 = 0.0f;
    msg.param7 = 0.0f;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    pub_cmd_->publish(msg);
  }

  void tryArmAndOffboard() {
    const double elapsed = (this->get_clock()->now() - start_time_).seconds();
    if (elapsed < warmup_sec_) return;

    // Retry commands every 1 second until both succeed,
    // because PX4 may silently reject the first attempts.
    const double now_sec = this->get_clock()->now().seconds();
    const bool should_retry = (now_sec - last_cmd_time_sec_) >= 1.0;

    if (!offboard_ || should_retry) {
      // OFFBOARD mode must be requested BEFORE arming.
      // VEHICLE_CMD_DO_SET_MODE: custom(1), OFFBOARD(6)
      sendVehicleCommand(176 /*VEHICLE_CMD_DO_SET_MODE*/, 1.0f, 6.0f);
      if (!offboard_) {
        RCLCPP_INFO(get_logger(), "Requested OFFBOARD mode");
        offboard_ = true;
      }
    }
    if (arm_on_start_ && (!armed_ || should_retry)) {
      // VEHICLE_CMD_COMPONENT_ARM_DISARM with ARMING_ACTION_ARM
      sendVehicleCommand(
          400 /*VEHICLE_CMD_COMPONENT_ARM_DISARM*/, 1.0f /*ARM*/);
      if (!armed_) {
        RCLCPP_INFO(get_logger(), "Sent ARM command");
        armed_ = true;
      }
      last_cmd_time_sec_ = now_sec;
    }
  }

  bool ensureYawCorrection() {
    if (have_yaw_corr_) return true;
    try {
      const geometry_msgs::msg::TransformStamped ts =
          tf_buffer_.lookupTransform("map", "odom", rclcpp::Time(0));
      const double qx = ts.transform.rotation.x;
      const double qy = ts.transform.rotation.y;
      const double qz = ts.transform.rotation.z;
      const double qw = ts.transform.rotation.w;
      const double siny_cosp = 2.0 * (qw * qz + qx * qy);
      const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
      yaw_map_to_odom_ = std::atan2(siny_cosp, cosy_cosp);
      have_yaw_corr_ = true;
      RCLCPP_INFO(get_logger(), "Using map->odom yaw: %.3f rad",
                  yaw_map_to_odom_);
    } catch (const std::exception &e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Waiting for TF map->odom yaw: %s", e.what());
      return false;
    }
    return true;
  }

  void publishGoalVisualization() {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp = this->get_clock()->now();
    ps.header.frame_id = "odom";
    ps.pose.position.x = goal_x_;
    ps.pose.position.y = goal_y_;
    ps.pose.position.z = goal_z_;
    const double half = 0.5 * goal_yaw_;
    ps.pose.orientation.x = 0.0;
    ps.pose.orientation.y = 0.0;
    ps.pose.orientation.z = std::sin(half);
    ps.pose.orientation.w = std::cos(half);
    pub_goal_pose_->publish(ps);

    visualization_msgs::msg::Marker m;
    m.header = ps.header;
    m.ns = "uav_control_goal";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::ARROW;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose = ps.pose;
    m.scale.x = 0.6;  // shaft length
    m.scale.y = 0.06; // shaft diameter
    m.scale.z = 0.06; // head diameter
    m.color.r = 0.1f;
    m.color.g = 0.9f;
    m.color.b = 0.1f;
    m.color.a = 1.0f;
    pub_goal_marker_->publish(m);
  }

  void tick() {
    // Always stream control mode
    publishOffboardMode();

    if (have_state_) {
      double ex, ey, ez;
      double yaw;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        ex = goal_x_ - x_;
        ey = goal_y_ - y_;
        ez = goal_z_ - z_;
        yaw = yaw_;
      }

      // dt for PID
      double dt = 0.0;
      const rclcpp::Time now = this->get_clock()->now();
      if (prev_time_set_) {
        dt = (now - prev_time_).seconds();
      }
      prev_time_ = now;
      prev_time_set_ = true;

      // Derivatives
      double dex = 0.0, dey = 0.0, dez = 0.0, de_yaw = 0.0;
      if (dt > 1e-6) {
        dex = (ex - prev_ex_) / dt;
        dey = (ey - prev_ey_) / dt;
        dez = (ez - prev_ez_) / dt;
      }

      // Integrals
      if (dt > 1e-6) {
        sum_ex_ += ex * dt;
        sum_ey_ += ey * dt;
        sum_ez_ += ez * dt;
      }

      // PID to velocity (ENU)
      double vx_e = kp_xy_ * ex + ki_xy_ * sum_ex_ + kd_xy_ * dex;
      double vy_e = kp_xy_ * ey + ki_xy_ * sum_ey_ + kd_xy_ * dey;
      double vz_e = kp_z_ * ez + ki_z_ * sum_ez_ + kd_z_ * dez;

      // Clamp XY magnitude
      const double vxy = std::hypot(vx_e, vy_e);
      if (vxy > max_speed_xy_) {
        const double s = max_speed_xy_ / std::max(1e-6, vxy);
        vx_e *= s;
        vy_e *= s;
      }
      // Clamp Z
      if (vz_e > max_speed_z_) vz_e = max_speed_z_;
      if (vz_e < -max_speed_z_) vz_e = -max_speed_z_;

      // Yaw PID
      const double e_yaw = wrap_pi(goal_yaw_ - yaw);
      if (dt > 1e-6) {
        de_yaw = (e_yaw - prev_eyaw_) / dt;
        sum_eyaw_ += e_yaw * dt;
      }
      double yawspeed_e = kp_yaw_ * e_yaw + ki_yaw_ * sum_eyaw_ + kd_yaw_ * de_yaw;
      // Clamp yaw rate
      if (yawspeed_e > max_yawspeed_) yawspeed_e = max_yawspeed_;
      if (yawspeed_e < -max_yawspeed_) yawspeed_e = -max_yawspeed_;

      // Optional: rotate odom ENU velocities into map ENU
      double vx_map = vx_e, vy_map = vy_e;
      if (apply_yaw_corr_ && ensureYawCorrection()) {
        const double c = std::cos(yaw_map_to_odom_);
        const double s = std::sin(yaw_map_to_odom_);
        vx_map = c * vx_e - s * vy_e;
        vy_map = s * vx_e + c * vy_e;
      }

      publishVelocitySetpoint(vx_map, vy_map, vz_e, yawspeed_e);
      publishGoalVisualization();

      // Save prev errors
      prev_ex_ = ex;
      prev_ey_ = ey;
      prev_ez_ = ez;
      prev_eyaw_ = e_yaw;
    }

    // Mode switching and arm
    tryArmAndOffboard();
  }

  // Members
  // Params
  double rate_hz_{};
  bool arm_on_start_{};
  double warmup_sec_{};
  bool apply_yaw_corr_{};
  double goal_x_{};
  double goal_y_{};
  double goal_z_{};
  double goal_yaw_{};
  double kp_xy_{};
  double ki_xy_{};
  double kd_xy_{};
  double kp_z_{};
  double ki_z_{};
  double kd_z_{};
  double kp_yaw_{};
  double ki_yaw_{};
  double kd_yaw_{};
  double max_speed_xy_{};
  double max_speed_z_{};
  double max_yawspeed_{};

  // TF (declare before others to match init order)
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  double yaw_map_to_odom_;
  bool have_yaw_corr_;

  // State
  std::mutex mutex_;
  bool have_state_;
  double x_{0.0}, y_{0.0}, z_{0.0};
  double yaw_{0.0};

  bool armed_;
  bool offboard_;
  double last_cmd_time_sec_;
  rclcpp::Time start_time_;

  // PID accumulators
  bool prev_time_set_;
  rclcpp::Time prev_time_;
  double prev_ex_{0.0}, prev_ey_{0.0}, prev_ez_{0.0}, prev_eyaw_{0.0};
  double sum_ex_{0.0}, sum_ey_{0.0}, sum_ez_{0.0}, sum_eyaw_{0.0};

  // ROS handles
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr pub_offboard_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr pub_traj_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr pub_cmd_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal_pose_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_goal_marker_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardPidGotoCpp>());
  rclcpp::shutdown();
  return 0;
}
