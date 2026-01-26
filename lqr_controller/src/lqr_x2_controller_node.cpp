// X-axis LQR controller using identified ARX(2) model for v-dynamics (NED x ≡ ENU y)
// v[k+1] = a1 v[k] + a2 v[k-1] + b1 u[k] + b2 u[k-1]
// Augmented state: x = [p; v; v_prev; u_prev],
//   where p = ENU y position (≈ NED x position),
//         p[k+1] = p[k] + Ts * v[k]
//
// ⚠ 사용 시 주의:
//  - 이 노드는 "수평(North) 방향" 제어용이다.
//  - 기체는 이미 안전한 호버 상태(적당한 고도)에서, z축이 안정된 상태라고 가정한다.
//  - 지면에 붙어 있는 상태에서 바로 실행하는 용도는 아니다.

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

class LqrX2Controller : public rclcpp::Node {
public:
  LqrX2Controller() : rclcpp::Node("lqr_x2_controller") {
    // Parameters
    rate_hz_      = this->declare_parameter<double>("rate_hz", 50.0);
    arm_on_start_ = this->declare_parameter<bool>("arm_on_start", true);  // x축은 기본 false 권장
    warmup_sec_   = this->declare_parameter<double>("warmup_sec", 0.5);

    // "목표 NED x" == "목표 ENU y" 위치 [m]
    goal_x_ = this->declare_parameter<double>("goal_x", 5.0);

    // Identified discrete ARX(2) model (from JSON)
    a1_ = this->declare_parameter<double>("a1", 1.3081754503637384);
    a2_ = this->declare_parameter<double>("a2", -0.31101784173009656);
    b1_ = this->declare_parameter<double>("b1", 0.00020393622774213238);
    b2_ = this->declare_parameter<double>("b2", 0.00306251768932857);
    Ts_ = this->declare_parameter<double>("Ts", 0.020004);  // median dt

    // LQR weights (초기값)
    // qx_            = this->declare_parameter<double>("Q_x", 8.0);   // 위치 오차 가중치
    // qv_            = this->declare_parameter<double>("Q_v", 20.0);    // 속도 가중치
    // qv_prev_       = this->declare_parameter<double>("Q_v_prev", 1.20);
    // qv_prev_       = this->declare_parameter<double>("Q_v_prev", 1.20);
    // qu_prev_state_ = this->declare_parameter<double>("Q_u_prev_state", 0.16);
    // r_             = this->declare_parameter<double>("R_u", 1.0);    // 입력 크기 가중치

    // Overshoot가 큰 경우 완화용 보수적 기본값
    // - Q_v를 높이고(Q_x 대비), R와 u_prev 가중치를 약간 올려 감쇠/완만 제어
    // - max_speed_x를 낮춰 목표점 근방에서 제동 여유 확보
    qx_            = this->declare_parameter<double>("Q_x", 8.0);     // 위치 오차 가중치(↓)
    qv_            = this->declare_parameter<double>("Q_v", 24.0);    // 속도 가중치(↑, 감쇠 강화)
    qv_prev_       = this->declare_parameter<double>("Q_v_prev", 1.20); // 이전 속도 가중치(약간 ↑)
    qu_prev_state_ = this->declare_parameter<double>("Q_u_prev_state", 0.50); // 이전 입력 상태 가중치(↑)
    r_             = this->declare_parameter<double>("R_u", 3.0);     // 입력 크기 가중치(↑)

    max_speed_x_ = this->declare_parameter<double>("max_speed_x", 3.0);  // [m/s] (↓, 과도 속도 억제)

    // Subscriptions (odom: ENU 프레임)
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
        "/lqr/goal_pose_x", 10);
    pub_goal_marker_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/lqr/goal_marker_x", 10);

    // Initialize controller gain K from (a1,a2,b1,b2,Ts,Q,R)
    computeLqrGain();

    // Timer
    start_time_ = this->get_clock()->now();
    const double period = 1.0 / std::max(1.0, rate_hz_);
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(period),
        std::bind(&LqrX2Controller::tick, this));

    RCLCPP_INFO(get_logger(),
                "lqr_x2_controller @ %.1f Hz; goal_x(NED/ENU y)=%.2f; "
                "ARX(2) a1=%.6f a2=%.6f b1=%.6f b2=%.6f Ts=%.3f; "
                "K=[%.3f %.3f %.3f %.3f]",
                rate_hz_, goal_x_, a1_, a2_, b1_, b2_, Ts_,
                Kx_, Kv_, Kv_prev_, Ku_prev_);
  }

private:
  inline uint64_t now_us() {
    return static_cast<uint64_t>(this->get_clock()->now().nanoseconds() / 1000);
  }

  // ENU odom 콜백 (여기서 ENU y를 "제어 x"로 사용)
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    x_  = msg->pose.pose.position.x;          // ENU x (실제 "앞" 축)
    vx_ = msg->twist.twist.linear.x;          // ENU x velocity
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
      RCLCPP_INFO(get_logger(), "[LQR X] Sent ARM command");
      armed_ = true;
    }
    if (armed_ && !offboard_) {
      sendVehicleCommand(176 /*DO_SET_MODE*/, 1.0f, 6.0f);
      RCLCPP_INFO(get_logger(), "[LQR X] Requested OFFBOARD mode");
      offboard_ = true;
    }
  }

  void publishVelocityX(double vx_cmd_enu) {
    // ENU x (odom.x) 속도 ≈ NED x 속도로 사용
    const float vx_cmd_ned = static_cast<float>(vx_cmd_enu);

    px4_msgs::msg::TrajectorySetpoint t;
    t.timestamp = now_us();
    t.position      = {NAN, NAN, NAN};
    t.velocity      = {vx_cmd_ned, 0.0f, 0.0f};  // NED: x=v_north, y=v_east, z=v_down
    t.acceleration  = {NAN, NAN, NAN};
    t.jerk          = {NAN, NAN, NAN};
    t.yaw           = NAN;
    t.yawspeed      = NAN;
    pub_traj_->publish(t);
  }

  void publishGoalSignals() {
    // PlotJuggler용 Float64
    std_msgs::msg::Float64 f; f.data = goal_x_;
    pub_goal_x_->publish(f);

    // RViz용 Pose (odom 프레임에서 y축 방향 목표 위치)
    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp = this->get_clock()->now();
    ps.header.frame_id = "odom";
    ps.pose.position.x = goal_x_;   // ENU x 방향
    ps.pose.position.y = 0.0;
    ps.pose.position.z = 0.0;
    ps.pose.orientation.x = 0.0;
    ps.pose.orientation.y = 0.0;
    ps.pose.orientation.z = 0.0;
    ps.pose.orientation.w = 1.0;
    pub_goal_pose_->publish(ps);

    visualization_msgs::msg::Marker m;
    m.header = ps.header;
    m.ns = "lqr_x_goal";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose = ps.pose;
    m.scale.x = 0.25; m.scale.y = 0.25; m.scale.z = 0.25;
    m.color.r = 0.9f; m.color.g = 0.3f; m.color.b = 0.1f; m.color.a = 1.0f;
    pub_goal_marker_->publish(m);
  }

  void computeLqrGain() {
    // x = [p, v, v_prev, u_prev]^T (p: ENU y pos, v: ENU y vel)
    // p[k+1] = p[k] + Ts * v[k]
    // v[k+1] = a1 v[k] + a2 v[k-1] + b1 u[k] + b2 u[k-1]
    //
    // A, B 구성
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

    // Discrete-time Riccati iteration
    double P[16];
    for (int i = 0; i < 16; ++i) P[i] = Q[i];
    const double tol = 1e-10;
    for (int it = 0; it < 1000; ++it) {
      // PA = P * A
      double PA[16] = {0};
      for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
          double s = 0.0;
          for (int k = 0; k < 4; ++k) s += P[i*4 + k] * A[k*4 + j];
          PA[i*4 + j] = s;
        }
      }
      // AT_PA = A^T * P * A
      double AT_PA[16] = {0};
      for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
          double s = 0.0;
          for (int k = 0; k < 4; ++k) s += A[k*4 + i] * PA[k*4 + j];
          AT_PA[i*4 + j] = s;
        }
      }
      // PB = P * B
      double PB[4] = {0};
      for (int i = 0; i < 4; ++i) {
        double s = 0.0;
        for (int k = 0; k < 4; ++k) s += P[i*4 + k] * B[k];
        PB[i] = s;
      }
      // AT_PB = A^T * P * B
      double AT_PB[4] = {0};
      for (int i = 0; i < 4; ++i) {
        double s = 0.0;
        for (int k = 0; k < 4; ++k) s += A[k*4 + i] * PB[k];
        AT_PB[i] = s;
      }

      const double BtPB = B[0]*PB[0] + B[1]*PB[1] + B[2]*PB[2] + B[3]*PB[3];
      const double S = r_ + BtPB;
      const double invS = 1.0 / S;

      // Y invS Y^T
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
    double PB[4] = {0};
    for (int i = 0; i < 4; ++i) {
      double s = 0.0;
      for (int k = 0; k < 4; ++k) s += P[i*4 + k] * B[k];
      PB[i] = s;
    }
    const double BtPB2 = B[0]*PB[0] + B[1]*PB[1] + B[2]*PB[2] + B[3]*PB[3];
    const double S2 = r_ + BtPB2;

    double PA2[16] = {0};
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        double s = 0.0;
        for (int k = 0; k < 4; ++k) s += P[i*4 + k] * A[k*4 + j];
        PA2[i*4 + j] = s;
      }
    }

    double BtPA[4] = {0};
    for (int j = 0; j < 4; ++j) {
      BtPA[j] = B[0]*PA2[0*4 + j]
              + B[1]*PA2[1*4 + j]
              + B[2]*PA2[2*4 + j]
              + B[3]*PA2[3*4 + j];
    }

    const double invS2 = 1.0 / S2;
    Kx_       = invS2 * BtPA[0];
    Kv_       = invS2 * BtPA[1];
    Kv_prev_  = invS2 * BtPA[2];
    Ku_prev_  = invS2 * BtPA[3];
  }

  void tick() {
    publishOffboardMode();

    // 항상 setpoint 스트림은 유지 (상태 없으면 0 명령)
    double u_enu = 0.0;

    if (have_state_) {
      double x, vx;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        x  = x_;
        vx = vx_;
      }

      const double ex       = x - goal_x_;   // p - p_ref
      const double ev       = vx - 0.0;
      const double ev_prev  = vx_prev_ - 0.0;
      const double eu_prev  = u_prev_enu_ - 0.0;

      // u (ENU y) = -K * [ex; ev; ev_prev; eu_prev]
      u_enu = -(Kx_ * ex + Kv_ * ev + Kv_prev_ * ev_prev + Ku_prev_ * eu_prev);
      u_enu = clamp(u_enu, -max_speed_x_, max_speed_x_);

      // 상태 업데이트용 이전값 갱신
      vx_prev_     = vx;
      u_prev_enu_  = u_enu;
    }

    publishVelocityX(u_enu);
    publishGoalSignals();
    tryArmAndOffboard();
  }

  // Params
  double rate_hz_{};
  bool arm_on_start_{};
  double warmup_sec_{};

  double goal_x_{};                // target NED x ≡ ENU y [m]
  double a1_{}, a2_{}, b1_{}, b2_{};
  double Ts_{};

  double qx_{}, qv_{}, qv_prev_{}, qu_prev_state_{}, r_{};
  double max_speed_x_{};

  // Gains
  double Kx_{0.0}, Kv_{0.0}, Kv_prev_{0.0}, Ku_prev_{0.0};

  // State
  std::mutex mutex_;
  bool have_state_{false};
  double x_{0.0};          // ENU y
  double vx_{0.0};         // ENU y vel
  double vx_prev_{0.0};
  double u_prev_enu_{0.0};
  bool armed_{false}, offboard_{false};
  rclcpp::Time start_time_;

  // ROS
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr pub_offboard_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr  pub_traj_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr      pub_cmd_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr             pub_goal_x_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr    pub_goal_pose_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr    pub_goal_marker_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LqrX2Controller>());
  rclcpp::shutdown();
  return 0;
}
