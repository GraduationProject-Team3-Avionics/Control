// X-axis LQR controller using identified ARX(2) model for v-dynamics
// v[k+1] = a1 v[k] + a2 v[k-1] + b1 u[k] + b2 u[k-1]
// Augmented state: x = [x; v; v_prev; u_prev], with x[k+1] = x[k] + Ts v[k]

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
#include "rcl_interfaces/msg/set_parameters_result.hpp"

namespace {

inline double clamp(double x, double lo, double hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

// ENU -> NED velocity mapping, only X control (ENU x -> NED y)
inline void enu_x_to_ned_xyz(double vx_e, float &vx_n, float &vy_n, float &vz_n) {
  vx_n = 0.0f;             // don't command N (ENU y) here
  vy_n = static_cast<float>(vx_e);  // ENU x maps to NED y
  vz_n = 0.0f;             // don't command Z here
}

}  // namespace

class LqrX2Controller : public rclcpp::Node {
 public:
  LqrX2Controller() : rclcpp::Node("lqr_x2_controller") {
    // Parameters
    rate_hz_ = this->declare_parameter<double>("rate_hz", 50.0);
    arm_on_start_ = this->declare_parameter<bool>("arm_on_start", true);
    warmup_sec_ = this->declare_parameter<double>("warmup_sec", 0.5);

    goal_x_ = this->declare_parameter<double>("goal_x", 3.0);

    // Identified discrete ARX(2) model (set from your identification results)
    a1_ = this->declare_parameter<double>("a1", 1.308175);
    a2_ = this->declare_parameter<double>("a2", -0.311018);
    b1_ = this->declare_parameter<double>("b1", 0.000204);
    b2_ = this->declare_parameter<double>("b2", 0.003063);
    Ts_ = this->declare_parameter<double>("Ts", 0.02);

    // LQR weights
    qx_ = this->declare_parameter<double>("Q_x", 5.0);
    qv_ = this->declare_parameter<double>("Q_v", 1.0);
    qv_prev_ = this->declare_parameter<double>("Q_v_prev", 0.05);
    qu_prev_state_ = this->declare_parameter<double>("Q_u_prev_state", 0.01);
    r_  = this->declare_parameter<double>("R_u", 0.3);

    max_speed_x_ = this->declare_parameter<double>("max_speed_x", 2.0);

    // Subscriptions
    auto qos = rclcpp::QoS(rclcpp::KeepLast(5));
    qos.best_effort();
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/uav/odom", qos,
        std::bind(&LqrX2Controller::onOdom, this, std::placeholders::_1));

    // Publishers
    pub_offboard_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode", 10);
    pub_traj_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint", 10);
    pub_cmd_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command", 10);
    pub_goal_x_ = this->create_publisher<std_msgs::msg::Float64>(
        "/lqr/goal_x", 10);
    pub_goal_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/lqr/goal_pose", 10);
    pub_goal_marker_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/lqr/goal_marker", 10);

    // Initialize controller gain K from (a1,a2,b1,b2,Ts,Q,R)
    computeLqrGain();

    // Allow live tuning of parameters (Q/R, model, limits)
    param_cb_handle_ = this->add_on_set_parameters_callback(
        std::bind(&LqrX2Controller::onSetParameters, this, std::placeholders::_1));

    // Timer
    start_time_ = this->get_clock()->now();
    const double period = 1.0 / std::max(1.0, rate_hz_);
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(period),
        std::bind(&LqrX2Controller::tick, this));

    RCLCPP_INFO(get_logger(),
                "lqr_x2_controller @ %.1f Hz; goal_x=%.2f; ARX(2) a1=%.6f a2=%.6f b1=%.6f b2=%.6f Ts=%.3f; K=[%.3f %.3f %.3f %.3f]",
                rate_hz_, goal_x_, a1_, a2_, b1_, b2_, Ts_, Kx_, Kv_, Kv_prev_, Ku_prev_);
  }

 private:
  inline uint64_t now_us() {
    return static_cast<uint64_t>(this->get_clock()->now().nanoseconds() / 1000);
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    x_ = msg->pose.pose.position.x;              // ENU x (east positive)
    vx_ = msg->twist.twist.linear.x;             // ENU vx (east positive)
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

  void publishVelocityX(double vx_cmd_enu) {
    float vx_n, vy_n, vz_n;
    enu_x_to_ned_xyz(vx_cmd_enu, vx_n, vy_n, vz_n);
    px4_msgs::msg::TrajectorySetpoint t;
    t.timestamp = now_us();
    t.position = {NAN, NAN, NAN};
    t.velocity = {vx_n, vy_n, vz_n};
    t.acceleration = {NAN, NAN, NAN};
    t.jerk = {NAN, NAN, NAN};
    t.yaw = NAN;
    t.yawspeed = NAN;
    pub_traj_->publish(t);
  }

  void publishGoalSignals() {
    // Float64 topic for PlotJuggler
    std_msgs::msg::Float64 f; f.data = goal_x_;
    pub_goal_x_->publish(f);

    // Pose for RViz
    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp = this->get_clock()->now();
    ps.header.frame_id = "odom";
    ps.pose.position.x = goal_x_;
    ps.pose.position.y = 0.0;
    ps.pose.position.z = 0.0;
    ps.pose.orientation.x = 0.0;
    ps.pose.orientation.y = 0.0;
    ps.pose.orientation.z = 0.0;
    ps.pose.orientation.w = 1.0;
    pub_goal_pose_->publish(ps);

    visualization_msgs::msg::Marker m;
    m.header = ps.header;
    m.ns = "lqr_goal_x";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::ARROW;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose = ps.pose;
    m.scale.x = 0.5; m.scale.y = 0.05; m.scale.z = 0.05;
    m.color.r = 0.9f; m.color.g = 0.4f; m.color.b = 0.1f; m.color.a = 1.0f;
    pub_goal_marker_->publish(m);
  }

  void computeLqrGain() {
    // Augmented discrete system for ARX(2): x=[pos, v, v_prev, u_prev]^T
    // x[k+1] = A x[k] + B u[k]
    const double A[16] = {
      1.0,   Ts_,  0.0,  0.0,
      0.0,   a1_,  a2_,  b2_,
      0.0,   1.0,  0.0,  0.0,
      0.0,   0.0,  0.0,  0.0
    };
    const double B[4] = {0.0, b1_, 0.0, 1.0};
    const double Q[16] = {
      qx_,  0.0,      0.0,          0.0,
      0.0,  qv_,      0.0,          0.0,
      0.0,  0.0,      qv_prev_,     0.0,
      0.0,  0.0,      0.0,          qu_prev_state_
    };

    // Iterative solution to discrete-time Riccati equation
    double P[16];
    for (int i = 0; i < 16; ++i) P[i] = Q[i];
    const double tol = 1e-10;
    for (int it = 0; it < 1000; ++it) {
      // Compute A^T P A and A^T P B
      double PA[16] = {0};
      for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
          double s = 0.0; for (int k = 0; k < 4; ++k) s += P[i*4 + k] * A[k*4 + j];
          PA[i*4 + j] = s;
        }
      }
      double AT_PA[16] = {0};
      for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
          double s = 0.0; for (int k = 0; k < 4; ++k) s += A[k*4 + i] * PA[k*4 + j];
          AT_PA[i*4 + j] = s;
        }
      }
      double PB[4] = {0};
      for (int i = 0; i < 4; ++i) { double s = 0.0; for (int k = 0; k < 4; ++k) s += P[i*4 + k] * B[k]; PB[i] = s; }
      double AT_PB[4] = {0};
      for (int i = 0; i < 4; ++i) { double s = 0.0; for (int k = 0; k < 4; ++k) s += A[k*4 + i] * PB[k]; AT_PB[i] = s; }
      const double BtPB = B[0]*PB[0] + B[1]*PB[1] + B[2]*PB[2] + B[3]*PB[3];
      const double S = r_ + BtPB;
      const double invS = 1.0 / S;
      // Y = A^T P B; Pnew = A^T P A - Y invS Y^T + Q
      double YinvsYt[16] = {0};
      for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
          YinvsYt[i*4 + j] = AT_PB[i] * invS * AT_PB[j];
        }
      }
      double Pnew[16];
      double diff = 0.0;
      for (int i = 0; i < 16; ++i) {
        Pnew[i] = AT_PA[i] - YinvsYt[i] + Q[i];
        diff += std::fabs(Pnew[i] - P[i]);
        P[i] = Pnew[i];
      }
      if (diff < tol) break;
    }
    // K = (R + B^T P B)^{-1} B^T P A
    double PB2[4] = {0}; for (int i = 0; i < 4; ++i) { double s=0.0; for (int k=0;k<4;++k) s+= P[i*4+k]*B[k]; PB2[i]=s; }
    const double BtPB2 = B[0]*PB2[0] + B[1]*PB2[1] + B[2]*PB2[2] + B[3]*PB2[3];
    const double S2 = r_ + BtPB2;
    double PA2[16] = {0}; for (int i=0;i<4;++i){ for(int j=0;j<4;++j){ double s=0.0; for(int k=0;k<4;++k) s+= P[i*4+k]*A[k*4+j]; PA2[i*4+j]=s; }}
    double BtPA[4] = {0}; for (int j=0;j<4;++j){ BtPA[j] = B[0]*PA2[0*4+j] + B[1]*PA2[1*4+j] + B[2]*PA2[2*4+j] + B[3]*PA2[3*4+j]; }
    const double invS2 = 1.0 / S2;
    Kx_      = invS2 * BtPA[0];
    Kv_      = invS2 * BtPA[1];
    Kv_prev_ = invS2 * BtPA[2];
    Ku_prev_ = invS2 * BtPA[3];
  }

  rcl_interfaces::msg::SetParametersResult onSetParameters(const std::vector<rclcpp::Parameter>& params) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    bool need_recompute = false;
    for (const auto &p : params) {
      const auto &name = p.get_name();
      if (name == "Q_x") { qx_ = p.as_double(); need_recompute = true; }
      else if (name == "Q_v") { qv_ = p.as_double(); need_recompute = true; }
      else if (name == "Q_v_prev") { qv_prev_ = p.as_double(); need_recompute = true; }
      else if (name == "Q_u_prev_state") { qu_prev_state_ = p.as_double(); need_recompute = true; }
      else if (name == "R_u") { r_ = p.as_double(); need_recompute = true; }
      else if (name == "a1") { a1_ = p.as_double(); need_recompute = true; }
      else if (name == "a2") { a2_ = p.as_double(); need_recompute = true; }
      else if (name == "b1") { b1_ = p.as_double(); need_recompute = true; }
      else if (name == "b2") { b2_ = p.as_double(); need_recompute = true; }
      else if (name == "Ts") { Ts_ = p.as_double(); need_recompute = true; }
      else if (name == "max_speed_x") { max_speed_x_ = p.as_double(); }
      else if (name == "goal_x") { goal_x_ = p.as_double(); }
      else if (name == "rate_hz") { rate_hz_ = p.as_double(); }
    }
    if (need_recompute) {
      computeLqrGain();
      RCLCPP_INFO(get_logger(), "Updated K=[%.3f %.3f %.3f %.3f]", Kx_, Kv_, Kv_prev_, Ku_prev_);
    }
    return result;
  }

  void tick() {
    publishOffboardMode();
    if (have_state_) {
      double x, vx;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        x = x_; vx = vx_;
      }
      const double ex = x - goal_x_;
      const double ev = vx - 0.0;
      const double ev_prev = vx_prev_ - 0.0;
      const double eu_prev = u_prev_enu_ - 0.0;
      // u (ENU x) = -K * [ex; ev; ev_prev; eu_prev]
      double u_enu = -(Kx_ * ex + Kv_ * ev + Kv_prev_ * ev_prev + Ku_prev_ * eu_prev);
      u_enu = clamp(u_enu, -max_speed_x_, max_speed_x_);
      publishVelocityX(u_enu);
      // Update prev states
      vx_prev_ = vx;
      u_prev_enu_ = u_enu;
    }
    // Always publish goal signals for plotting/rviz
    publishGoalSignals();
    tryArmAndOffboard();
  }

  // Params
  double rate_hz_{};
  bool arm_on_start_{};
  double warmup_sec_{};
  double goal_x_{};
  double a1_{}, a2_{}, b1_{}, b2_{};
  double Ts_{};
  double qx_{}, qv_{}, qv_prev_{}, qu_prev_state_{}, r_{};
  double max_speed_x_{};

  // Gain
  double Kx_{0.0}, Kv_{0.0}, Kv_prev_{0.0}, Ku_prev_{0.0};

  // State
  std::mutex mutex_;
  bool have_state_{false};
  double x_{0.0};
  double vx_{0.0};
  double vx_prev_{0.0};
  double u_prev_enu_{0.0};
  bool armed_{false}, offboard_{false};
  rclcpp::Time start_time_;

  // ROS
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr pub_offboard_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr pub_traj_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr pub_cmd_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_goal_x_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal_pose_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_goal_marker_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LqrX2Controller>());
  rclcpp::shutdown();
  return 0;
}

