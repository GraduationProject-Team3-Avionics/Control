// Z-axis LQR controller using identified ARX(1) model: v[k+1] = a v[k] + b u[k]
// Augmented state: x = [z; v] with z[k+1] = z[k] + Ts v[k]
// Control law: u = -K (x - x_ref), x_ref = [z_goal; 0]

#include <cmath>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace {

inline double clamp(double x, double lo, double hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

}  // namespace

class LqrZController : public rclcpp::Node {
 public:
  LqrZController() : rclcpp::Node("lqr_z_controller") {
    // Parameters
    rate_hz_ = this->declare_parameter<double>("rate_hz", 50.0);
    arm_on_start_ = this->declare_parameter<bool>("arm_on_start", true);
    warmup_sec_ = this->declare_parameter<double>("warmup_sec", 0.5);

    goal_z_ = this->declare_parameter<double>("goal_z", 3.0);

    // Identified discrete model (defaults are placeholders; set from identification)
    a_ = this->declare_parameter<double>("a", 0.988336);     // from ARX(1)
    b_ = this->declare_parameter<double>("b", 0.013005);    // from ARX(1)
    Ts_ = this->declare_parameter<double>("Ts", 0.02);   // median dt <=> Sampling Time(T_s)

    // LQR weights
    qz_ = this->declare_parameter<double>("Q_z", 5.0);
    qv_ = this->declare_parameter<double>("Q_v", 1.0);
    r_  = this->declare_parameter<double>("R_u", 0.3);

    max_speed_z_ = this->declare_parameter<double>("max_speed_z", 2.0);

    // Subscriptions
    auto qos = rclcpp::QoS(rclcpp::KeepLast(5));
    qos.best_effort();
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/uav/odom", qos,
        std::bind(&LqrZController::onOdom, this, std::placeholders::_1));

    // Publishers
    pub_offboard_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode", 10);
    pub_traj_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint", 10);
    pub_cmd_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command", 10);
    pub_goal_z_ = this->create_publisher<std_msgs::msg::Float64>(
        "/lqr/goal_z", 10);
    pub_goal_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/lqr/goal_pose", 10);
    pub_goal_marker_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/lqr/goal_marker", 10);

    // Initialize controller gain K from (a,b,Ts,Q,R)
    computeLqrGain();

    // Timer
    start_time_ = this->get_clock()->now();
    const double period = 1.0 / std::max(1.0, rate_hz_);
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(period),
        std::bind(&LqrZController::tick, this));

    RCLCPP_INFO(get_logger(),
                "lqr_z_controller @ %.1f Hz; goal_z=%.2f; a=%.3f b=%.3f Ts=%.3f; K=[%.3f %.3f]",
                rate_hz_, goal_z_, a_, b_, Ts_, Kz_, Kv_);
  }

 private:
  inline uint64_t now_us() {
    return static_cast<uint64_t>(this->get_clock()->now().nanoseconds() / 1000);
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    z_ = msg->pose.pose.position.z;              // ENU z (up positive)
    vz_ = msg->twist.twist.linear.z;             // ENU z velocity (up positive)
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

  void sendVehicleCommand(uint16_t command, float p1 = 0.0f, float p2 = 0.0f) {
    px4_msgs::msg::VehicleCommand msg;
    msg.timestamp = now_us();
    msg.param1 = p1;
    msg.param2 = p2;
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
    const double dt = (this->get_clock()->now() - start_time_).seconds();
    if (dt < warmup_sec_) return;
    if (arm_on_start_ && !armed_) {
      sendVehicleCommand(400 /*ARM_DISARM*/, 1.0f);
      RCLCPP_INFO(get_logger(), "Sent ARM command");
      armed_ = true;
    }
    if (armed_ && !offboard_) {
      sendVehicleCommand(176 /*DO_SET_MODE*/, 1.0f, 6.0f);
      RCLCPP_INFO(get_logger(), "Requested OFFBOARD mode");
      offboard_ = true;
    }
  }

  void publishVelocityZ(double vz_cmd_enu) {
    // Convert ENU z velocity command to NED z component (positive down)
    const float vz_cmd_ned = static_cast<float>(-vz_cmd_enu);
    px4_msgs::msg::TrajectorySetpoint t;
    t.timestamp = now_us();
    t.position = {NAN, NAN, NAN};
    t.velocity = {0.0f, 0.0f, vz_cmd_ned};
    t.acceleration = {NAN, NAN, NAN};
    t.jerk = {NAN, NAN, NAN};
    t.yaw = NAN;
    t.yawspeed = NAN;
    pub_traj_->publish(t);
  }

  void publishGoalSignals() {
    // Float64 topic for PlotJuggler
    std_msgs::msg::Float64 f; f.data = goal_z_;
    pub_goal_z_->publish(f);

    // Pose for RViz
    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp = this->get_clock()->now();
    ps.header.frame_id = "odom";
    ps.pose.position.x = 0.0;
    ps.pose.position.y = 0.0;
    ps.pose.position.z = goal_z_;
    ps.pose.orientation.x = 0.0;
    ps.pose.orientation.y = 0.0;
    ps.pose.orientation.z = 0.0;
    ps.pose.orientation.w = 1.0;
    pub_goal_pose_->publish(ps);

    visualization_msgs::msg::Marker m;
    m.header = ps.header;
    m.ns = "lqr_goal";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose = ps.pose;
    m.scale.x = 0.25; m.scale.y = 0.25; m.scale.z = 0.25;
    m.color.r = 0.1f; m.color.g = 0.4f; m.color.b = 0.9f; m.color.a = 1.0f;
    pub_goal_marker_->publish(m);
  }

  void computeLqrGain() {
    // Discrete system
    const double A11 = 1.0;
    const double A12 = Ts_;
    const double A21 = 0.0;
    const double A22 = a_;
    const double B1 = 0.0;
    const double B2 = b_;

    // Riccati iteration for P (2x2) symmetric
    double P11 = qz_, P12 = 0.0, P22 = qv_;
    const double tol = 1e-10;
    for (int it = 0; it < 500; ++it) {
      // A^T P A components
      const double M11 = P11;
      const double M12 = P11 * A12 + P12 * A22;
      const double M21 = P12;
      const double M22 = P12 * A12 + P22 * A22;
      const double APA11 = M11;                          // = P11
      const double APA12 = M12;                          // = P11*Ts + P12*a
      const double APA22 = A12 * M12 + A22 * M22;        // = Ts^2 P11 + 2 a Ts P12 + a^2 P22

      // S = R + B^T P B = r + b^2 * P22
      const double S = r_ + B2 * B2 * P22;
      // A^T P B vector (2x1): b*[P12, Ts*P12 + a*P22]
      const double AT_PB1 = B2 * P12;
      const double AT_PB2 = B2 * (A12 * P12 + A22 * P22);
      // (A^T P B) * S^{-1} * (B^T P A) term
      const double coef = 1.0 / S;
      const double T11 = coef * (AT_PB1 * AT_PB1);
      const double T12 = coef * (AT_PB1 * AT_PB2);
      const double T22 = coef * (AT_PB2 * AT_PB2);

      const double P11_new = APA11 - T11 + qz_;
      const double P12_new = APA12 - T12 + 0.0;  // Q off-diagonal = 0
      const double P22_new = APA22 - T22 + qv_;

      const double d = std::fabs(P11_new - P11) + std::fabs(P12_new - P12) + std::fabs(P22_new - P22);
      P11 = P11_new; P12 = P12_new; P22 = P22_new;
      if (d < tol) break;
    }

    // K = (R + B^T P B)^{-1} B^T P A
    const double S = r_ + B2 * B2 * P22;
    const double BtPA1 = B2 * P12;                         // corresponds to z-state
    const double BtPA2 = B2 * (A12 * P12 + A22 * P22);     // corresponds to v-state
    Kz_ = BtPA1 / S;
    Kv_ = BtPA2 / S;
  }

  void tick() {
    publishOffboardMode();
    if (have_state_) {
      double z, vz;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        z = z_; vz = vz_;
      }
      const double ez = z - goal_z_;
      const double ev = vz - 0.0;
      // u (ENU z) = -K * [ez; ev]
      double u_enu = -(Kz_ * ez + Kv_ * ev);
      u_enu = clamp(u_enu, -max_speed_z_, max_speed_z_);
      publishVelocityZ(u_enu);
    }
    // Always publish goal signals for plotting/rviz
    publishGoalSignals();
    tryArmAndOffboard();
  }

  // Params
  double rate_hz_{};
  bool arm_on_start_{};
  double warmup_sec_{};
  double goal_z_{};
  double a_{}, b_{}, Ts_{};
  double qz_{}, qv_{}, r_{};
  double max_speed_z_{};

  // Gain
  double Kz_{0.0}, Kv_{0.0};

  // State
  std::mutex mutex_;
  bool have_state_{false};
  double z_{0.0};
  double vz_{0.0};
  bool armed_{false}, offboard_{false};
  rclcpp::Time start_time_;

  // ROS
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr pub_offboard_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr pub_traj_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr pub_cmd_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_goal_z_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal_pose_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_goal_marker_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LqrZController>());
  rclcpp::shutdown();
  return 0;
}
