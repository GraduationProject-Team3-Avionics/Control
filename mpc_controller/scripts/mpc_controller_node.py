#!/usr/bin/env python3
"""
MPC Controller Node for PX4
경로 추적을 위한 MPC 컨트롤러 ROS2 노드 (PX4 Offboard 통합)

Subscriptions:
    /uav/odom (nav_msgs/Odometry) - 현재 상태
    /mpc/reference_state (geometry_msgs/PoseStamped) - 목표 상태

Publications:
    /mpc/control_output (geometry_msgs/TwistStamped) - 제어 출력 (디버깅용)
    /fmu/in/offboard_control_mode (px4_msgs/OffboardControlMode) - Offboard 모드 heartbeat
    /fmu/in/vehicle_attitude_setpoint (px4_msgs/VehicleAttitudeSetpoint) - 자세/추력 명령
    /fmu/in/vehicle_command (px4_msgs/VehicleCommand) - Arm/Disarm/Mode 명령
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped
import numpy as np
from scipy.spatial.transform import Rotation

# PX4 Messages
from px4_msgs.msg import (
    OffboardControlMode,
    VehicleAttitudeSetpoint,
    VehicleCommand,
    VehicleStatus
)

# MPC Core
from mpc_core import QuadcopterModel, MPCParams, mpc_solve


class MPCControllerNode(Node):
    def __init__(self):
        super().__init__('mpc_controller')
        
        # ─────────── Parameters ───────────
        self.declare_parameters(
            namespace='',
            parameters=[
                ('mass', 2.0),
                ('gravity', 9.81),
                ('dt', 0.05),
                ('horizon_N', 20),
                ('Q_pos', [50.0, 50.0, 100.0]),
                ('Q_vel', [5.0, 5.0, 5.0]),
                ('R', [0.5, 0.5, 0.05]),
                ('phi_max_deg', 30.0),
                ('theta_max_deg', 30.0),
                ('v_max', 3.0),
                ('enable_state_constraints', True),
                ('control_rate', 20.0),
                ('auto_arm', False),  # 자동 Arm 여부 (QGC 사용시 False)
            ]
        )
        
        # 파라미터 로드
        mass = self.get_parameter('mass').value
        gravity = self.get_parameter('gravity').value
        dt = self.get_parameter('dt').value
        N = self.get_parameter('horizon_N').value
        Q_pos = self.get_parameter('Q_pos').value
        Q_vel = self.get_parameter('Q_vel').value
        R = self.get_parameter('R').value
        phi_max_deg = self.get_parameter('phi_max_deg').value
        theta_max_deg = self.get_parameter('theta_max_deg').value
        v_max = self.get_parameter('v_max').value
        enable_state_constraints = self.get_parameter('enable_state_constraints').value
        control_rate = self.get_parameter('control_rate').value
        self.auto_arm = self.get_parameter('auto_arm').value
        
        # ─────────── Model & MPC Setup ───────────
        self.model = QuadcopterModel(mass=mass, gravity=gravity, dt=dt)
        
        Q = np.diag(Q_pos + Q_vel)
        R_mat = np.diag(R)
        
        self.mpc_params = MPCParams(
            N=N,
            dt=dt,
            Q=Q,
            R=R_mat,
            phi_max=np.deg2rad(phi_max_deg),
            theta_max=np.deg2rad(theta_max_deg),
            delta_T_max=self.model.T_hover,
            v_max=v_max,
            enable_state_constraints=enable_state_constraints
        )
        
        # ─────────── State Variables ───────────
        self.current_state = None      # [px, py, pz, vx, vy, vz]
        self.reference_state = None    # [px, py, pz, vx, vy, vz]
        self.current_yaw = 0.0         # 현재 yaw (유지용)
        
        # PX4 상태
        self.offboard_setpoint_counter = 0
        self.vehicle_status = None
        self.is_armed = False
        self.is_offboard = False
        
        # ─────────── QoS Profiles ───────────
        # Sensor data QoS (BEST_EFFORT for PX4)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Reliable QoS for commands
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # ─────────── Subscribers ───────────
        self.odom_sub = self.create_subscription(
            Odometry,
            '/uav/odom',
            self.odom_callback,
            sensor_qos
        )
        
        self.ref_sub = self.create_subscription(
            PoseStamped,
            '/mpc/reference_state',
            self.reference_callback,
            reliable_qos
        )
        
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            sensor_qos
        )
        
        # ─────────── Publishers ───────────
        self.control_pub = self.create_publisher(
            TwistStamped,
            '/mpc/control_output',
            reliable_qos
        )
        
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            reliable_qos
        )
        
        self.attitude_setpoint_pub = self.create_publisher(
            VehicleAttitudeSetpoint,
            '/fmu/in/vehicle_attitude_setpoint',
            reliable_qos
        )
        
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            reliable_qos
        )
        
        # ─────────── Timers ───────────
        self.timer = self.create_timer(1.0 / control_rate, self.control_loop)
        
        # ─────────── Logging ───────────
        self.get_logger().info('MPC Controller Node 시작 (PX4 Offboard 통합)')
        self.get_logger().info(f'  Model: mass={mass}kg, T_hover={self.model.T_hover:.2f}N')
        self.get_logger().info(f'  MPC: N={N}, dt={dt}s')
        self.get_logger().info(f'  Constraints: phi/theta ±{phi_max_deg}°, v_max={v_max}m/s')
        self.get_logger().info(f'  Auto Arm: {self.auto_arm}')
    
    # ─────────── Callbacks ───────────
    def odom_callback(self, msg: Odometry):
        """현재 상태 업데이트"""
        pos = msg.pose.pose.position
        vel = msg.twist.twist.linear
        
        self.current_state = np.array([
            pos.x, pos.y, pos.z,
            vel.x, vel.y, vel.z
        ])
        
        # yaw 추출 (자세 유지용)
        q = msg.pose.pose.orientation
        rot = Rotation.from_quat([q.x, q.y, q.z, q.w])
        euler = rot.as_euler('xyz')
        self.current_yaw = euler[2]
    
    def reference_callback(self, msg: PoseStamped):
        """목표 상태 업데이트"""
        pos = msg.pose.position
        
        # 속도 레퍼런스는 0으로 설정 (정지 목표)
        self.reference_state = np.array([
            pos.x, pos.y, pos.z,
            0.0, 0.0, 0.0
        ])
    
    def vehicle_status_callback(self, msg: VehicleStatus):
        """PX4 상태 업데이트"""
        self.vehicle_status = msg
        self.is_armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        self.is_offboard = (msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)
    
    # ─────────── Control Loop ───────────
    def control_loop(self):
        """MPC 제어 루프"""
        # Offboard 모드 heartbeat (항상 발행해야 함)
        self.publish_offboard_control_mode()
        
        # 자동 Arm/Offboard (옵션)
        if self.auto_arm:
            self.offboard_setpoint_counter += 1
            
            # 상태 없으면 hover setpoint 발행 (PX4는 유효한 setpoint 스트림 필요)
            if self.current_state is None:
                self._publish_idle_setpoint()
                return
            
            # 충분한 setpoint 발행 후 offboard → arm 순차 전환
            if self.offboard_setpoint_counter >= 10 and not self.is_offboard:
                self.engage_offboard_mode()
            if self.offboard_setpoint_counter >= 15 and not self.is_armed:
                self.arm()
        else:
            if self.current_state is None:
                return
        
        if self.reference_state is None:
            # 레퍼런스 없으면 현재 위치 유지
            self.reference_state = self.current_state.copy()
            self.reference_state[3:] = 0  # 속도는 0
        
        # MPC 풀이
        u_opt, success = mpc_solve(
            self.current_state,
            self.reference_state,
            self.model,
            self.mpc_params
        )
        
        if not success:
            self.get_logger().warn('MPC 최적화 실패, 안전 모드')
            u_opt = np.zeros(3)
        
        # 제어 출력 발행
        self.publish_control_output(u_opt)
        self.publish_attitude_setpoint(u_opt)
    
    # ─────────── Publishers ───────────
    def _publish_idle_setpoint(self):
        """상태 수신 전 기본 hover setpoint 발행 (PX4 offboard 진입용)"""
        msg = VehicleAttitudeSetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.q_d = [1.0, 0.0, 0.0, 0.0]  # 수평 자세 (identity quaternion)
        msg.thrust_body = [0.0, 0.0, -0.5]  # hover 추력 (약 50%)
        self.attitude_setpoint_pub.publish(msg)
    
    def publish_offboard_control_mode(self):
        """Offboard 제어 모드 heartbeat 발행"""
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = True    # Attitude 제어 모드
        msg.body_rate = False
        
        self.offboard_control_mode_pub.publish(msg)
    
    def publish_attitude_setpoint(self, u_opt: np.ndarray):
        """PX4 Attitude Setpoint 발행"""
        phi_d, theta_d, delta_T = u_opt
        
        msg = VehicleAttitudeSetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        # Roll, Pitch, Yaw → Quaternion
        # 현재 yaw 유지
        rot = Rotation.from_euler('xyz', [phi_d, theta_d, self.current_yaw])
        q = rot.as_quat()  # [x, y, z, w]
        
        # PX4는 [w, x, y, z] 순서
        msg.q_d = [float(q[3]), float(q[0]), float(q[1]), float(q[2])]
        
        # 추력 (0~1 정규화)
        # PX4에서 thrust는 0~1 범위
        total_thrust = self.model.T_hover + delta_T
        max_thrust = 2.0 * self.model.T_hover  # 최대 추력 = 2 * hover
        normalized_thrust = np.clip(total_thrust / max_thrust, 0.0, 1.0)
        msg.thrust_body = [0.0, 0.0, -float(normalized_thrust)]  # NED 좌표계
        
        self.attitude_setpoint_pub.publish(msg)
    
    def publish_control_output(self, u_opt: np.ndarray):
        """디버깅용 제어 출력 발행"""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        phi_d, theta_d, delta_T = u_opt
        
        msg.twist.linear.x = delta_T
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = self.model.T_hover + delta_T
        
        msg.twist.angular.x = phi_d
        msg.twist.angular.y = theta_d
        msg.twist.angular.z = 0.0
        
        self.control_pub.publish(msg)
    
    # ─────────── PX4 Commands ───────────
    def arm(self):
        """기체 Arm"""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0  # 1: arm, 0: disarm
        )
        self.get_logger().info('Arm 명령 전송')
    
    def disarm(self):
        """기체 Disarm"""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=0.0
        )
        self.get_logger().info('Disarm 명령 전송')
    
    def engage_offboard_mode(self):
        """Offboard 모드 전환"""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,  # custom mode
            param2=6.0   # PX4_CUSTOM_MAIN_MODE_OFFBOARD
        )
        self.get_logger().info('Offboard 모드 전환 명령 전송')
    
    def publish_vehicle_command(self, command, **params):
        """VehicleCommand 발행"""
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        
        # 파라미터 설정
        msg.param1 = params.get('param1', 0.0)
        msg.param2 = params.get('param2', 0.0)
        msg.param3 = params.get('param3', 0.0)
        msg.param4 = params.get('param4', 0.0)
        msg.param5 = params.get('param5', 0.0)
        msg.param6 = params.get('param6', 0.0)
        msg.param7 = params.get('param7', 0.0)
        
        self.vehicle_command_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MPCControllerNode()
    
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
