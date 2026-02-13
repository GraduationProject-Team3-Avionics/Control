#!/usr/bin/env python3
"""
MPC Position Controller Node
MATLAB mpc_controller.m / mpc_path_tracking.m의 ROS 2 Python 포팅

제어 구조:
    - 6-state 선형 모델 (px,py,pz,vx,vy,vz)
    - 3-input (φ_d, θ_d, ΔT)
    - QP solver: OSQP
    - PX4 Level 3 인터페이스 (VehicleAttitudeSetpoint)

구독:
    /fmu/out/vehicle_odometry     (px4_msgs/VehicleOdometry)
    /mpc/reference_trajectory     (nav_msgs/Path)

발행:
    /fmu/in/vehicle_attitude_setpoint  (px4_msgs/VehicleAttitudeSetpoint)
    /fmu/in/offboard_control_mode      (px4_msgs/OffboardControlMode)
    /fmu/in/vehicle_command            (px4_msgs/VehicleCommand)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Path, Odometry
from px4_msgs.msg import (
    VehicleAttitudeSetpoint,
    OffboardControlMode,
    VehicleCommand,
)

import numpy as np
from scipy.linalg import expm
import osqp
from scipy import sparse


class MPCPositionControllerNode(Node):
    def __init__(self):
        super().__init__('mpc_position_controller_node')

        # ═══════════════════════════════════════════
        # Parameters
        # ═══════════════════════════════════════════
        self.declare_parameters(
            namespace='',
            parameters=[
                ('mass', 2.0),
                ('gravity', 9.81),
                ('dt', 0.05),
                ('prediction_horizon', 20),
                ('Q_diag', [50.0, 50.0, 100.0, 5.0, 5.0, 5.0]),
                ('R_diag', [0.5, 0.5, 0.05]),
                ('phi_max_deg', 30.0),
                ('theta_max_deg', 30.0),
                ('v_max', 3.0),
                ('T_max', 39.24),
            ]
        )

        self.m = self.get_parameter('mass').value
        self.g = self.get_parameter('gravity').value
        self.dt = self.get_parameter('dt').value
        self.N = self.get_parameter('prediction_horizon').value

        Q_diag = self.get_parameter('Q_diag').value
        R_diag = self.get_parameter('R_diag').value
        self.Q = np.diag(Q_diag)
        self.R = np.diag(R_diag)

        phi_max = np.deg2rad(self.get_parameter('phi_max_deg').value)
        theta_max = np.deg2rad(self.get_parameter('theta_max_deg').value)
        T_hover = self.m * self.g
        self.T_max = self.get_parameter('T_max').value

        self.u_min = np.array([-phi_max, -theta_max, -T_hover])
        self.u_max = np.array([phi_max, theta_max, T_hover])
        self.v_max = self.get_parameter('v_max').value

        self.nx = 6
        self.nu = 3

        # ═══════════════════════════════════════════
        # 모델 이산화 & QP 사전 계산
        # ═══════════════════════════════════════════
        self.Ad, self.Bd = self._discretize_model()
        self.Phi, self.Gamma = self._build_prediction_matrices()
        self.H, self.Q_bar, self.A_ineq, self.l_ineq, self.u_ineq = \
            self._build_qp_constant_matrices()

        self.solver = None

        # ═══════════════════════════════════════════
        # 상태 변수
        # ═══════════════════════════════════════════
        self.x_current = None
        self.ref_positions = None
        self.traj_idx = 0

        # ARM / Offboard 상태 머신
        self._offboard_setpoint_count = 0
        self._armed = False
        self._offboard = False
        self._last_cmd_time = 0.0

        # ═══════════════════════════════════════════
        # QoS Profiles
        # ═══════════════════════════════════════════
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        # ═══════════════════════════════════════════
        # Subscribers
        # ═══════════════════════════════════════════
        self.odom_sub = self.create_subscription(
            Odometry, '/uav/odom',
            self._odom_callback, px4_qos)

        self.traj_sub = self.create_subscription(
            Path, '/mpc/reference_trajectory',
            self._trajectory_callback, latched_qos)

        # ═══════════════════════════════════════════
        # Publishers
        # ═══════════════════════════════════════════
        self.attitude_pub = self.create_publisher(
            VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint', px4_qos)
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', px4_qos)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', px4_qos)

        # ═══════════════════════════════════════════
        # 제어 타이머 (20 Hz)
        # ═══════════════════════════════════════════
        self.control_timer = self.create_timer(self.dt, self._control_loop)

        self.get_logger().info(
            f'MPC Position Controller | N={self.N}, dt={self.dt}s, '
            f'Q={Q_diag}, R={R_diag}')

    # ══════════════════════════════════════════════════════════
    #  모델 이산화
    # ══════════════════════════════════════════════════════════
    def _discretize_model(self):
        g, m, dt = self.g, self.m, self.dt
        nx, nu = self.nx, self.nu

        Ac = np.zeros((nx, nx))
        Ac[0, 3] = 1.0; Ac[1, 4] = 1.0; Ac[2, 5] = 1.0

        Bc = np.zeros((nx, nu))
        Bc[3, 1] = g; Bc[4, 0] = -g; Bc[5, 2] = 1.0 / m

        M = np.zeros((nx + nu, nx + nu))
        M[:nx, :nx] = Ac; M[:nx, nx:] = Bc
        M_exp = expm(M * dt)

        return M_exp[:nx, :nx], M_exp[:nx, nx:]

    # ══════════════════════════════════════════════════════════
    #  예측 행렬 Φ, Γ
    # ══════════════════════════════════════════════════════════
    def _build_prediction_matrices(self):
        N, nx, nu = self.N, self.nx, self.nu
        Ad, Bd = self.Ad, self.Bd

        Phi = np.zeros((nx * N, nx))
        Gamma = np.zeros((nx * N, nu * N))

        A_power = Ad.copy()
        for i in range(N):
            Phi[i * nx:(i + 1) * nx, :] = A_power
            A_power = A_power @ Ad

        A_powers_B = [Bd.copy()]
        A_pow = Ad.copy()
        for k in range(1, N):
            A_powers_B.append(A_pow @ Bd)
            A_pow = A_pow @ Ad

        for i in range(N):
            for j in range(i + 1):
                Gamma[i * nx:(i + 1) * nx, j * nu:(j + 1) * nu] = A_powers_B[i - j]

        return Phi, Gamma

    # ══════════════════════════════════════════════════════════
    #  QP 고정 행렬 사전 계산
    # ══════════════════════════════════════════════════════════
    def _build_qp_constant_matrices(self):
        N, nx, nu = self.N, self.nx, self.nu

        Q_bar = np.kron(np.eye(N), self.Q)
        R_bar = np.kron(np.eye(N), self.R)

        H = 2.0 * (self.Gamma.T @ Q_bar @ self.Gamma + R_bar)
        H = (H + H.T) / 2.0

        U_min = np.tile(self.u_min, N)
        U_max = np.tile(self.u_max, N)

        C_vel_single = np.zeros((3, nx))
        C_vel_single[0, 3] = 1.0; C_vel_single[1, 4] = 1.0; C_vel_single[2, 5] = 1.0
        C_vel = np.kron(np.eye(N), C_vel_single)

        I_nu_N = np.eye(nu * N)
        C_Gamma = C_vel @ self.Gamma
        A_ineq = np.vstack([I_nu_N, C_Gamma])

        l_ineq = np.concatenate([U_min, -self.v_max * np.ones(3 * N)])
        u_ineq = np.concatenate([U_max,  self.v_max * np.ones(3 * N)])

        self._C_vel_Phi = C_vel @ self.Phi
        self._n_input_constraints = nu * N

        return H, Q_bar, A_ineq, l_ineq, u_ineq

    # ══════════════════════════════════════════════════════════
    #  QP 풀기
    # ══════════════════════════════════════════════════════════
    def _solve_qp(self, x0, X_ref):
        e0 = self.Phi @ x0 - X_ref
        f = 2.0 * self.Gamma.T @ self.Q_bar @ e0

        C_Phi_x0 = self._C_vel_Phi @ x0
        n_ic = self._n_input_constraints
        l = self.l_ineq.copy()
        u = self.u_ineq.copy()
        l[n_ic:] = -self.v_max - C_Phi_x0
        u[n_ic:] =  self.v_max - C_Phi_x0

        H_sparse = sparse.csc_matrix(self.H)
        A_sparse = sparse.csc_matrix(self.A_ineq)

        if self.solver is None:
            self.solver = osqp.OSQP()
            self.solver.setup(
                P=H_sparse, q=f, A=A_sparse, l=l, u=u,
                verbose=False,
                warm_start=True,
                eps_abs=1e-5, eps_rel=1e-5,
                max_iter=200,
                polish=False,       # polish 비활성화 → 로그 제거
            )
        else:
            self.solver.update(q=f, l=l, u=u)

        result = self.solver.solve()

        if result.info.status != 'solved':
            self.get_logger().warn(f'QP failed: {result.info.status}')
            return np.zeros(self.nu)

        return result.x[:self.nu]

    # ══════════════════════════════════════════════════════════
    #  좌표 변환
    # ══════════════════════════════════════════════════════════
    @staticmethod
    def _ned_to_enu_position(pos_ned):
        return np.array([pos_ned[1], pos_ned[0], -pos_ned[2]])

    @staticmethod
    def _ned_to_enu_velocity(vel_ned):
        return np.array([vel_ned[1], vel_ned[0], -vel_ned[2]])

    @staticmethod
    def _euler_to_quaternion(roll, pitch, yaw):
        cr, sr = np.cos(roll / 2.0), np.sin(roll / 2.0)
        cp, sp = np.cos(pitch / 2.0), np.sin(pitch / 2.0)
        cy, sy = np.cos(yaw / 2.0), np.sin(yaw / 2.0)
        return np.array([
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ])

    # ══════════════════════════════════════════════════════════
    #  Callbacks
    # ══════════════════════════════════════════════════════════
    def _odom_callback(self, msg: Odometry):
        """nav_msgs/Odometry (ENU) 파싱 — PID 컨트롤러와 동일한 토픽/타입"""
        pos = msg.pose.pose.position
        vel = msg.twist.twist.linear
        pos_enu = np.array([pos.x, pos.y, pos.z], dtype=np.float64)
        vel_enu = np.array([vel.x, vel.y, vel.z], dtype=np.float64)
        if np.any(np.isnan(pos_enu)) or np.any(np.isnan(vel_enu)):
            return
        self.x_current = np.concatenate([pos_enu, vel_enu])

    def _trajectory_callback(self, msg: Path):
        if len(msg.poses) < 2:
            return
        self.ref_positions = np.array([
            [p.pose.position.x, p.pose.position.y, p.pose.position.z]
            for p in msg.poses
        ])
        self.traj_idx = 0
        self.get_logger().info(f'Reference trajectory 수신: {len(msg.poses)}pts')

    # ══════════════════════════════════════════════════════════
    #  Reference 추출
    # ══════════════════════════════════════════════════════════
    def _get_reference_states(self):
        M = len(self.ref_positions)
        N = self.N
        dt = self.dt

        if self.x_current is not None:
            pos_current = self.x_current[:3]
            search_start = max(0, self.traj_idx - 5)
            search_end = min(M, self.traj_idx + 50)
            distances = np.linalg.norm(
                self.ref_positions[search_start:search_end] - pos_current, axis=1)
            self.traj_idx = search_start + np.argmin(distances)

        X_ref = np.zeros(self.nx * N)
        for k in range(N):
            idx = min(self.traj_idx + k, M - 1)
            p_ref = self.ref_positions[idx]
            v_ref = ((self.ref_positions[min(idx + 1, M - 1)] - self.ref_positions[idx]) / dt
                     if idx < M - 1 else np.zeros(3))
            X_ref[k * self.nx:(k + 1) * self.nx] = np.concatenate([p_ref, v_ref])

        return X_ref

    # ══════════════════════════════════════════════════════════
    #  PX4 Command 헬퍼
    # ══════════════════════════════════════════════════════════
    def _send_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)

    def _arm(self):
        self._send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0)
        self.get_logger().info('ARM 명령 전송')

    def _set_offboard_mode(self):
        self._send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0, param2=6.0)  # PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6
        self.get_logger().info('OFFBOARD 모드 전환 명령 전송')

    # ══════════════════════════════════════════════════════════
    #  제어 루프 (20 Hz)
    # ══════════════════════════════════════════════════════════
    def _control_loop(self):
        # ── 1) OffboardControlMode + Setpoint 항상 발행 (PX4 필수) ──
        self._publish_offboard_mode()

        # MPC 데이터 준비되면 MPC, 아니면 hover setpoint
        if self.x_current is not None and self.ref_positions is not None:
            X_ref = self._get_reference_states()
            u_opt = self._solve_qp(self.x_current, X_ref)
            self._publish_attitude_setpoint(u_opt[0], u_opt[1], u_opt[2])
        else:
            self._publish_hover_setpoint()

        # ── 2) OFFBOARD + ARM 시퀀스 (warmup + 1초 간격 재전송) ──
        self._offboard_setpoint_count += 1

        if self._offboard_setpoint_count < 10:
            # 첫 10틱(0.5초): setpoint만 스트리밍
            return

        now_sec = self.get_clock().now().nanoseconds / 1e9
        should_retry = (now_sec - self._last_cmd_time) >= 1.0

        # OFFBOARD 모드 먼저 요청
        if not self._offboard or should_retry:
            self._set_offboard_mode()
            if not self._offboard:
                self._offboard = True

        # 그 다음 ARM
        if not self._armed or should_retry:
            self._arm()
            if not self._armed:
                self._armed = True
            self._last_cmd_time = now_sec

    # ══════════════════════════════════════════════════════════
    #  PX4 발행
    # ══════════════════════════════════════════════════════════
    def _publish_hover_setpoint(self):
        """데이터 미준비 시 hover setpoint 발행 → PX4 failsafe 방지"""
        msg = VehicleAttitudeSetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.q_d = [1.0, 0.0, 0.0, 0.0]  # identity quaternion (수평)
        T_hover = self.m * self.g
        msg.thrust_body = [0.0, 0.0, float(-T_hover / self.T_max)]
        self.attitude_pub.publish(msg)

    def _publish_attitude_setpoint(self, phi_d_enu, theta_d_enu, delta_T):
        # ENU → NED 변환:
        #   ax_ned = ay_enu → θ_ned = φ_d_enu
        #   ay_ned = ax_enu → φ_ned = θ_d_enu
        roll_ned = theta_d_enu
        pitch_ned = phi_d_enu
        yaw_ned = 0.0

        q = self._euler_to_quaternion(roll_ned, pitch_ned, yaw_ned)

        T_hover = self.m * self.g
        thrust_normalized = np.clip(-(T_hover + delta_T) / self.T_max, -1.0, 0.0)

        msg = VehicleAttitudeSetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.q_d = [float(q[0]), float(q[1]), float(q[2]), float(q[3])]
        msg.thrust_body = [0.0, 0.0, float(thrust_normalized)]
        self.attitude_pub.publish(msg)

    def _publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = True
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        self.offboard_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MPCPositionControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
