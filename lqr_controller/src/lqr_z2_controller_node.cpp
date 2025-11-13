// Z-axis LQR controller using identified ARX(2) model for v-dynamics
// v[k+1] = a1 v[k] + a2 v[k-1] + b1 u[k] + b2 u[k-1]
// Augmented state: x = [z; v; v_prev; u_prev], with z[k+1] = z[k] + Ts v[k]

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

class LqrZ2Controller : public rclcpp::Node {
 public:
  LqrZ2Controller() : rclcpp::Node("lqr_z2_controller") {
    // Parameters
    rate_hz_ = this->declare_parameter<double>("rate_hz", 50.0);
    arm_on_start_ = this->declare_parameter<bool>("arm_on_start", true);
    warmup_sec_ = this->declare_parameter<double>("warmup_sec", 0.5);

    goal_z_ = this->declare_parameter<double>("goal_z", 3.0);

    // Identified discrete ARX(2) model (defaults from identification summary)
    a1_ = this->declare_parameter<double>("a1", 1.161027);
    a2_ = this->declare_parameter<double>("a2", -0.174423);
    b1_ = this->declare_parameter<double>("b1", 0.000113);
    b2_ = this->declare_parameter<double>("b2", 0.013628);
    Ts_ = this->declare_parameter<double>("Ts", 0.02);

    // LQR weights (1st attempt)
    // qz_ = this->declare_parameter<double>("Q_z", 5.0);
    // qv_ = this->declare_parameter<double>("Q_v", 1.0);
    // qv_prev_ = this->declare_parameter<double>("Q_v_prev", 0.05);
    // qu_prev_state_ = this->declare_parameter<double>("Q_u_prev_state", 0.01);
    // r_  = this->declare_parameter<double>("R_u", 0.3);

    // LQR weights (tuned for faster response)
    qz_ = this->declare_parameter<double>("Q_z", 20.0);
    qv_ = this->declare_parameter<double>("Q_v", 4.0);
    qv_prev_ = this->declare_parameter<double>("Q_v_prev", 0.10);
    qu_prev_state_ = this->declare_parameter<double>("Q_u_prev_state", 0.01);
    r_  = this->declare_parameter<double>("R_u", 0.15);

    max_speed_z_ = this->declare_parameter<double>("max_speed_z", 2.0);

    // Subscriptions
    auto qos = rclcpp::QoS(rclcpp::KeepLast(5));
    qos.best_effort();
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/uav/odom", qos,
        std::bind(&LqrZ2Controller::onOdom, this, std::placeholders::_1));

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
        "/lqr/goal_pose_z", 10);
    pub_goal_marker_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/lqr/goal_marker_z", 10);

    // Initialize controller gain K from (a1,a2,b1,b2,Ts,Q,R)
    computeLqrGain();

    // Timer
    start_time_ = this->get_clock()->now();
    const double period = 1.0 / std::max(1.0, rate_hz_);
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(period),
        std::bind(&LqrZ2Controller::tick, this));

    RCLCPP_INFO(get_logger(),
                "lqr_z2_controller @ %.1f Hz; goal_z=%.2f; ARX(2) a1=%.6f a2=%.6f b1=%.6f b2=%.6f Ts=%.3f; K=[%.3f %.3f %.3f %.3f]",
                rate_hz_, goal_z_, a1_, a2_, b1_, b2_, Ts_, Kz_, Kv_, Kv_prev_, Ku_prev_);
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
    // Augmented discrete system for ARX(2): x=[z, v, v_prev, u_prev]^T
    // x[k+1] = A x[k] + B u[k]
    const double A[16] = {
      1.0,   Ts_,  0.0,  0.0,
      0.0,   a1_,  a2_,  b2_,
      0.0,   1.0,  0.0,  0.0,
      0.0,   0.0,  0.0,  0.0
    };
    const double B[4] = {0.0, b1_, 0.0, 1.0};
    const double Q[16] = {
      qz_,  0.0,      0.0,          0.0,
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
    double PB[4] = {0}; for (int i = 0; i < 4; ++i) { double s=0.0; for (int k=0;k<4;++k) s+= P[i*4+k]*B[k]; PB[i]=s; }
    const double BtPB = B[0]*PB[0] + B[1]*PB[1] + B[2]*PB[2] + B[3]*PB[3];
    const double S = r_ + BtPB;
    double PA[16] = {0}; for (int i=0;i<4;++i){ for(int j=0;j<4;++j){ double s=0.0; for(int k=0;k<4;++k) s+= P[i*4+k]*A[k*4+j]; PA[i*4+j]=s; }}
    double BtPA[4] = {0}; for (int j=0;j<4;++j){ BtPA[j] = B[0]*PA[0*4+j] + B[1]*PA[1*4+j] + B[2]*PA[2*4+j] + B[3]*PA[3*4+j]; }
    const double invS2 = 1.0 / S;
    Kz_      = invS2 * BtPA[0];
    Kv_      = invS2 * BtPA[1];
    Kv_prev_ = invS2 * BtPA[2];
    Ku_prev_ = invS2 * BtPA[3];
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
      const double ev_prev = vz_prev_ - 0.0;
      const double eu_prev = u_prev_enu_ - 0.0;
      // u (ENU z) = -K * [ez; ev; ev_prev; eu_prev]
      double u_enu = -(Kz_ * ez + Kv_ * ev + Kv_prev_ * ev_prev + Ku_prev_ * eu_prev);
      u_enu = clamp(u_enu, -max_speed_z_, max_speed_z_);
      publishVelocityZ(u_enu);
      // Update prev states
      vz_prev_ = vz;
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
  double goal_z_{};
  double a1_{}, a2_{}, b1_{}, b2_{};
  double Ts_{};
  double qz_{}, qv_{}, qv_prev_{}, qu_prev_state_{}, r_{};
  double max_speed_z_{};

  // Gain
  double Kz_{0.0}, Kv_{0.0}, Kv_prev_{0.0}, Ku_prev_{0.0};

  // State
  std::mutex mutex_;
  bool have_state_{false};
  double z_{0.0};
  double vz_{0.0};
  double vz_prev_{0.0};
  double u_prev_enu_{0.0};
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
  rclcpp::spin(std::make_shared<LqrZ2Controller>());
  rclcpp::shutdown();
  return 0;
}

