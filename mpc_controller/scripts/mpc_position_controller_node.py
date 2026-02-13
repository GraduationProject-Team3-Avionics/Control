#!/usr/bin/env python3
"""
MPC Position Controller Node (Yaw Hold Version)

핵심 변경점 (Yaw Hold):
- odom에서 현재 yaw(ENU)를 추출
- 첫 yaw를 yaw setpoint로 저장하여 계속 유지 (yaw hold)
- yaw setpoint를 NED로 변환해서 VehicleAttitudeSetpoint에 반영

추가 안정화/디버깅:
- OSQP polish 끔(polish=False) → "Polishing not needed" 스팸 제거
- ENU(FLU 가정) → NED(FRD)로 보낼 때 pitch 부호 반영 (FLU→FRD에서 pitch/yaw 부호 반전)
- twist가 이미 world(ENU)로 들어오는 경우를 대비해 파라미터로 회전 변환 on/off 가능

구독:
    /uav/odom                     (nav_msgs/Odometry, ENU)
    /mpc/reference_trajectory     (nav_msgs/Path, ENU)

발행:
    /fmu/in/vehicle_attitude_setpoint_v1  (px4_msgs/VehicleAttitudeSetpoint, NED/FRD)
    /fmu/in/offboard_control_mode         (px4_msgs/OffboardControlMode)
    /fmu/in/vehicle_command               (px4_msgs/VehicleCommand)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Path, Odometry
from px4_msgs.msg import VehicleAttitudeSetpoint, OffboardControlMode, VehicleCommand

import numpy as np
from scipy.linalg import expm
import osqp
from scipy import sparse


def wrap_pi(a: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return (a + np.pi) % (2.0 * np.pi) - np.pi


class MPCPositionControllerNode(Node):
    def __init__(self):
        super().__init__('mpc_position_controller_node')

        # ───────────────────────────────────────────
        # Parameters
        # ───────────────────────────────────────────
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

                # ⚠️ /uav/odom twist가 child_frame(base_link, FLU) 기준인지 여부
                # - True: body -> world 회전 수행 (기존 로직)
                # - False: 이미 world(ENU)로 나온 속도라고 가정하고 회전 안 함
                ('twist_is_body_frame', True),

                # yaw hold 켜기/끄기 (원하면 나중에 yaw 제어 구현 시 False 가능)
                ('enable_yaw_hold', True),

                # yaw hold를 시작하는 조건:
                # - True: odom이 한번 들어오면 즉시 yaw_sp 고정
                # - False: offboard+arm 이후에 yaw_sp 고정 (원하면 변경)
                ('yaw_hold_lock_immediately', True),
            ]
        )

        self.m = float(self.get_parameter('mass').value)
        self.g = float(self.get_parameter('gravity').value)
        self.dt = float(self.get_parameter('dt').value)
        self.N = int(self.get_parameter('prediction_horizon').value)

        Q_diag = self.get_parameter('Q_diag').value
        R_diag = self.get_parameter('R_diag').value
        self.Q = np.diag(Q_diag)
        self.R = np.diag(R_diag)

        phi_max = np.deg2rad(float(self.get_parameter('phi_max_deg').value))
        theta_max = np.deg2rad(float(self.get_parameter('theta_max_deg').value))
        T_hover = self.m * self.g
        self.T_max = float(self.get_parameter('T_max').value)

        # MPC 입력 u = [phi_d, theta_d, delta_T]
        # delta_T는 hover 기준 증분으로 사용 (최대 ±T_hover)
        self.u_min = np.array([-phi_max, -theta_max, -T_hover], dtype=np.float64)
        self.u_max = np.array([ phi_max,  theta_max,  T_hover], dtype=np.float64)

        self.v_max = float(self.get_parameter('v_max').value)

        self.twist_is_body_frame = bool(self.get_parameter('twist_is_body_frame').value)
        self.enable_yaw_hold = bool(self.get_parameter('enable_yaw_hold').value)
        self.yaw_hold_lock_immediately = bool(self.get_parameter('yaw_hold_lock_immediately').value)

        self.nx = 6
        self.nu = 3

        # ───────────────────────────────────────────
        # 모델 이산화 & QP 사전 계산
        # ───────────────────────────────────────────
        self.Ad, self.Bd = self._discretize_model()
        self.Phi, self.Gamma = self._build_prediction_matrices()
        self.H, self.Q_bar, self.A_ineq, self.l_ineq, self.u_ineq = self._build_qp_constant_matrices()

        self.solver = None

        # ───────────────────────────────────────────
        # 상태 변수
        # ───────────────────────────────────────────
        self.x_current = None
        self.ref_positions = None
        self.traj_idx = 0

        # yaw 관련 (ENU/NED)
        self.yaw_enu_current = None
        self.yaw_sp_ned = None

        # ARM / Offboard 상태 머신
        self._offboard_setpoint_count = 0
        self._armed = False
        self._offboard = False
        self._last_cmd_time = 0.0

        # ───────────────────────────────────────────
        # QoS Profiles
        # ───────────────────────────────────────────
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

        # ───────────────────────────────────────────
        # Subscribers
        # ───────────────────────────────────────────
        self.odom_sub = self.create_subscription(Odometry, '/uav/odom', self._odom_callback, px4_qos)
        self.traj_sub = self.create_subscription(Path, '/mpc/reference_trajectory', self._trajectory_callback, latched_qos)

        # ───────────────────────────────────────────
        # Publishers
        # ───────────────────────────────────────────
        self.attitude_pub = self.create_publisher(
            VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint_v1', px4_qos
        )
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', px4_qos
        )
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', px4_qos
        )

        # ───────────────────────────────────────────
        # 제어 타이머 (dt)
        # ───────────────────────────────────────────
        self.control_timer = self.create_timer(self.dt, self._control_loop)

        self.get_logger().info(
            f'MPC Position Controller (Yaw Hold) | N={self.N}, dt={self.dt}s, '
            f'Q={Q_diag}, R={R_diag}, twist_is_body_frame={self.twist_is_body_frame}, '
            f'enable_yaw_hold={self.enable_yaw_hold}'
        )

    # ───────────────────────────────────────────
    #  모델 이산화
    # ───────────────────────────────────────────
    def _discretize_model(self):
        g, m, dt = self.g, self.m, self.dt
        nx, nu = self.nx, self.nu

        # 상태: [px, py, pz, vx, vy, vz]
        Ac = np.zeros((nx, nx), dtype=np.float64)
        Ac[0, 3] = 1.0
        Ac[1, 4] = 1.0
        Ac[2, 5] = 1.0

        # 입력: [phi, theta, delta_T]
        # ENU / yaw=0 근사에서
        # ax(=vx_dot) ≈ g * theta
        # ay(=vy_dot) ≈ -g * phi
        # az(=vz_dot) ≈ delta_T / m
        Bc = np.zeros((nx, nu), dtype=np.float64)
        Bc[3, 1] = g
        Bc[4, 0] = -g
        Bc[5, 2] = 1.0 / m

        # matrix exponential for discretization
        M = np.zeros((nx + nu, nx + nu), dtype=np.float64)
        M[:nx, :nx] = Ac
        M[:nx, nx:] = Bc
        M_exp = expm(M * dt)

        Ad = M_exp[:nx, :nx]
        Bd = M_exp[:nx, nx:]
        return Ad, Bd

    # ───────────────────────────────────────────
    #  예측 행렬 Φ, Γ
    # ───────────────────────────────────────────
    def _build_prediction_matrices(self):
        N, nx, nu = self.N, self.nx, self.nu
        Ad, Bd = self.Ad, self.Bd

        Phi = np.zeros((nx * N, nx), dtype=np.float64)
        Gamma = np.zeros((nx * N, nu * N), dtype=np.float64)

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

    # ───────────────────────────────────────────
    #  QP 고정 행렬 사전 계산
    # ───────────────────────────────────────────
    def _build_qp_constant_matrices(self):
        N, nx, nu = self.N, self.nx, self.nu

        Q_bar = np.kron(np.eye(N), self.Q)
        R_bar = np.kron(np.eye(N), self.R)

        H = 2.0 * (self.Gamma.T @ Q_bar @ self.Gamma + R_bar)
        H = (H + H.T) / 2.0

        U_min = np.tile(self.u_min, N)
        U_max = np.tile(self.u_max, N)

        # 속도 제한: |vx|,|vy|,|vz| <= v_max
        C_vel_single = np.zeros((3, nx), dtype=np.float64)
        C_vel_single[0, 3] = 1.0
        C_vel_single[1, 4] = 1.0
        C_vel_single[2, 5] = 1.0
        C_vel = np.kron(np.eye(N), C_vel_single)

        I_nu_N = np.eye(nu * N)
        C_Gamma = C_vel @ self.Gamma
        A_ineq = np.vstack([I_nu_N, C_Gamma])

        l_ineq = np.concatenate([U_min, -self.v_max * np.ones(3 * N)])
        u_ineq = np.concatenate([U_max,  self.v_max * np.ones(3 * N)])

        self._C_vel_Phi = C_vel @ self.Phi
        self._n_input_constraints = nu * N

        return H, Q_bar, A_ineq, l_ineq, u_ineq

    # ───────────────────────────────────────────
    #  QP 풀기
    # ───────────────────────────────────────────
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
                eps_abs=1e-3, eps_rel=1e-3,
                max_iter=4000,
                polish=False,          # ✅ 스팸 방지
                adaptive_rho=True,
            )
        else:
            self.solver.update(q=f, l=l, u=u)

        result = self.solver.solve()

        if result.info.status not in ('solved', 'solved_inaccurate'):
            self.get_logger().warn(f'QP failed: {result.info.status}', throttle_duration_sec=2.0)
            return np.zeros(self.nu, dtype=np.float64)

        return np.array(result.x[:self.nu], dtype=np.float64)

    # ───────────────────────────────────────────
    #  좌표/회전 유틸
    # ───────────────────────────────────────────
    @staticmethod
    def _quat_rotate(qx, qy, qz, qw, v):
        """Rotate vector v by quaternion (qx, qy, qz, qw). v_world = q ⊗ v ⊗ q*"""
        q = np.array([qx, qy, qz], dtype=np.float64)
        t = 2.0 * np.cross(q, v)
        return v + qw * t + np.cross(q, t)

    @staticmethod
    def _euler_to_quaternion(roll, pitch, yaw):
        """ZYX Euler (roll, pitch, yaw) -> quaternion [w, x, y, z]"""
        cr, sr = np.cos(roll / 2.0), np.sin(roll / 2.0)
        cp, sp = np.cos(pitch / 2.0), np.sin(pitch / 2.0)
        cy, sy = np.cos(yaw / 2.0), np.sin(yaw / 2.0)
        return np.array([
            cr * cp * cy + sr * sp * sy,  # w
            sr * cp * cy - cr * sp * sy,  # x
            cr * sp * cy + sr * cp * sy,  # y
            cr * cp * sy - sr * sp * cy,  # z
        ], dtype=np.float64)

    @staticmethod
    def _quat_to_yaw_enu(qx, qy, qz, qw) -> float:
        """Quaternion -> yaw about +Z in ENU. input: (x,y,z,w)"""
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return float(np.arctan2(siny_cosp, cosy_cosp))

    @staticmethod
    def _yaw_enu_to_ned(yaw_enu: float) -> float:
        """
        ENU yaw(about +Z_enu) -> NED yaw(about +Z_ned=down).
        Common mapping: yaw_ned = pi/2 - yaw_enu
        """
        return wrap_pi(np.pi / 2.0 - yaw_enu)

    # ───────────────────────────────────────────
    #  Callbacks
    # ───────────────────────────────────────────
    def _odom_callback(self, msg: Odometry):
        """nav_msgs/Odometry (ENU) 파싱"""
        pos = msg.pose.pose.position
        pos_enu = np.array([pos.x, pos.y, pos.z], dtype=np.float64)

        o = msg.pose.pose.orientation

        # yaw current (ENU)
        yaw_enu = self._quat_to_yaw_enu(o.x, o.y, o.z, o.w)
        self.yaw_enu_current = yaw_enu

        # yaw hold setpoint lock
        if self.enable_yaw_hold and self.yaw_sp_ned is None and self.yaw_hold_lock_immediately:
            self.yaw_sp_ned = self._yaw_enu_to_ned(yaw_enu)
            self.get_logger().info(
                f'[YawHold] yaw_enu={np.rad2deg(yaw_enu):.1f}deg -> yaw_sp_ned={np.rad2deg(self.yaw_sp_ned):.1f}deg'
            )

        # velocity: either body->world rotate or pass-through
        vel_msg = msg.twist.twist.linear
        v_raw = np.array([vel_msg.x, vel_msg.y, vel_msg.z], dtype=np.float64)

        if self.twist_is_body_frame:
            vel_enu = self._quat_rotate(o.x, o.y, o.z, o.w, v_raw)
        else:
            vel_enu = v_raw

        if np.any(np.isnan(pos_enu)) or np.any(np.isnan(vel_enu)):
            return

        self.x_current = np.concatenate([pos_enu, vel_enu])

        # DEBUG (1Hz)
        self.get_logger().info(
            f'[DBG] odom pos=({pos_enu[0]:.2f},{pos_enu[1]:.2f},{pos_enu[2]:.2f}) '
            f'vel=({vel_enu[0]:.2f},{vel_enu[1]:.2f},{vel_enu[2]:.2f}) '
            f'yaw_enu={np.rad2deg(yaw_enu):.1f}deg',
            throttle_duration_sec=1.0
        )

        # yaw hold를 offboard+arm 이후에 잠그고 싶다면(옵션)
        if self.enable_yaw_hold and self.yaw_sp_ned is None and (not self.yaw_hold_lock_immediately):
            if self._offboard and self._armed:
                self.yaw_sp_ned = self._yaw_enu_to_ned(yaw_enu)
                self.get_logger().info(
                    f'[YawHold] (post-arm) yaw_enu={np.rad2deg(yaw_enu):.1f}deg -> yaw_sp_ned={np.rad2deg(self.yaw_sp_ned):.1f}deg'
                )

    def _trajectory_callback(self, msg: Path):
        if len(msg.poses) < 2:
            return
        self.ref_positions = np.array(
            [[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in msg.poses],
            dtype=np.float64
        )
        self.traj_idx = 0
        self.get_logger().info(f'Reference trajectory 수신: {len(msg.poses)} pts')

    # ───────────────────────────────────────────
    #  Reference 추출
    # ───────────────────────────────────────────
    def _get_reference_states(self):
        M = len(self.ref_positions)
        N = self.N
        dt = self.dt

        if self.x_current is not None:
            pos_current = self.x_current[:3]
            search_start = max(0, self.traj_idx - 5)
            search_end = min(M, self.traj_idx + 50)
            distances = np.linalg.norm(self.ref_positions[search_start:search_end] - pos_current, axis=1)
            self.traj_idx = int(search_start + np.argmin(distances))

        X_ref = np.zeros(self.nx * N, dtype=np.float64)
        for k in range(N):
            idx = min(self.traj_idx + k, M - 1)
            p_ref = self.ref_positions[idx]

            if idx < M - 1:
                v_ref = (self.ref_positions[min(idx + 1, M - 1)] - self.ref_positions[idx]) / dt
            else:
                v_ref = np.zeros(3, dtype=np.float64)

            X_ref[k * self.nx:(k + 1) * self.nx] = np.concatenate([p_ref, v_ref])

        return X_ref

    # ───────────────────────────────────────────
    #  PX4 Command helpers
    # ───────────────────────────────────────────
    def _send_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = int(command)
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)

    def _arm(self):
        self._send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('ARM 명령 전송')

    def _set_offboard_mode(self):
        self._send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info('OFFBOARD 모드 전환 명령 전송')

    # ───────────────────────────────────────────
    #  제어 루프
    # ───────────────────────────────────────────
    def _control_loop(self):
        # 1) OffboardControlMode + Setpoint 항상 발행
        self._publish_offboard_mode()

        if self.x_current is not None and self.ref_positions is not None:
            X_ref = self._get_reference_states()
            u_opt = self._solve_qp(self.x_current, X_ref)
            self._publish_attitude_setpoint(u_opt[0], u_opt[1], u_opt[2], X_ref)
        else:
            self._publish_hover_setpoint()

        # 2) OFFBOARD + ARM 시퀀스
        self._offboard_setpoint_count += 1
        if self._offboard_setpoint_count < 10:
            return

        now_sec = self.get_clock().now().nanoseconds / 1e9
        should_retry = (now_sec - self._last_cmd_time) >= 1.0

        if (not self._offboard) or should_retry:
            self._set_offboard_mode()
            self._offboard = True

        if (not self._armed) or should_retry:
            self._arm()
            self._armed = True
            self._last_cmd_time = now_sec

    # ───────────────────────────────────────────
    #  PX4 publish
    # ───────────────────────────────────────────
    def _publish_hover_setpoint(self):
        """데이터 미준비 시 hover setpoint 발행 → PX4 failsafe 방지"""
        msg = VehicleAttitudeSetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        # yaw hold가 가능한 경우엔 yaw 유지 quaternion을 만들어서 넣어줌
        yaw_ned = float(self.yaw_sp_ned) if (self.enable_yaw_hold and self.yaw_sp_ned is not None) else 0.0
        q = self._euler_to_quaternion(0.0, 0.0, yaw_ned)

        msg.q_d = [float(q[0]), float(q[1]), float(q[2]), float(q[3])]

        T_hover = self.m * self.g
        msg.thrust_body = [0.0, 0.0, float(-T_hover / self.T_max)]
        self.attitude_pub.publish(msg)

    def _publish_attitude_setpoint(self, phi_d_enu, theta_d_enu, delta_T, X_ref=None):
        """
        MPC 출력은 ENU(세계)/FLU(바디) 가정 기반으로 생성되었다고 보고,
        PX4 setpoint는 NED(세계)/FRD(바디)로 보낸다.

        최소 변환:
        - ENU <-> NED 축 변환 + FLU -> FRD 부호 반영
        - (경험적으로 가장 문제를 많이 일으키는 것이 pitch 부호)

        현재 적용:
        - roll_ned  = theta_enu
        - pitch_ned = -phi_enu   # ✅ FLU->FRD 부호 반영 포함
        - yaw_ned   = yaw_hold (없으면 0)
        """
        roll_ned = float(theta_d_enu)
        pitch_ned = float(-phi_d_enu)

        if self.enable_yaw_hold and self.yaw_sp_ned is not None:
            yaw_ned = float(self.yaw_sp_ned)
        else:
            yaw_ned = 0.0

        q = self._euler_to_quaternion(roll_ned, pitch_ned, yaw_ned)

        T_hover = self.m * self.g
        thrust_normalized = np.clip(-(T_hover + float(delta_T)) / self.T_max, -1.0, 0.0)

        msg = VehicleAttitudeSetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.q_d = [float(q[0]), float(q[1]), float(q[2]), float(q[3])]
        msg.thrust_body = [0.0, 0.0, float(thrust_normalized)]
        self.attitude_pub.publish(msg)

        # DEBUG (1Hz)
        if X_ref is not None:
            self.get_logger().info(
                f'[DBG] MPC phi_enu={phi_d_enu:+.4f} theta_enu={theta_d_enu:+.4f} dT={delta_T:+.2f} '
                f'-> roll_ned={roll_ned:+.4f} pitch_ned={pitch_ned:+.4f} yaw_ned={yaw_ned:+.4f} '
                f'thr_norm={thrust_normalized:+.3f} '
                f'ref0=({X_ref[0]:.2f},{X_ref[1]:.2f},{X_ref[2]:.2f})',
                throttle_duration_sec=1.0
            )

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