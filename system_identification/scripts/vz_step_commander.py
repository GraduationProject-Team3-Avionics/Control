#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import os
from enum import Enum, auto
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleOdometry


def nan():
    return float('nan')

def wrap_pi(angle: float) -> float:
    import math
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

def enu_yaw_to_ned(yaw_enu: float) -> float:
    # yaw_ned = pi/2 - yaw_enu (wrap to [-pi, pi])
    import math
    return wrap_pi(math.pi / 2.0 - yaw_enu)


class Phase(Enum):
    INIT = auto()
    TAKEOFF = auto()
    HOVER_STABILIZE = auto()
    STEP = auto()
    DONE = auto()


class VzStepCommander(Node):
    """
    Z축 속도 스텝 입력 퍼블리셔 (Offboard 제어)
    상태기계:
      INIT -> (odom 준비) -> TAKEOFF (목표고도까지 상승)
           -> HOVER_STABILIZE (목표고도 근처에서 정착 대기)
           -> STEP (식별용 스텝 시퀀스)
           -> DONE
    PX4/px4_msgs 버전차는 hasattr로 보호.
    """

    def __init__(self):
        super().__init__('vz_step_commander')

        # -------------------------
        # Parameters
        # -------------------------
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('alt_min_m', 0.5)
        self.declare_parameter('alt_max_m', 5.0)
        self.declare_parameter('arm_on_start', True)
        self.declare_parameter('disarm_on_finish', False)
        self.declare_parameter('enable_mode_switch', True)

        # Auto takeoff / hover options
        self.declare_parameter('enable_auto_takeoff', True)
        self.declare_parameter('warmup_sec', 0.5)          # Offboard 예열 시간(세트포인트 스트림 후 모드 전환)
        self.declare_parameter('hover_alt_m', 1.5)         # 목표 고도
        self.declare_parameter('takeoff_vz_enu', 0.5)      # 이륙 상승 속도(+ 위로)
        self.declare_parameter('hover_band_m', 0.10)       # 목표 고도 근처 밴드
        self.declare_parameter('hover_settle_s', 2.0)      # 밴드 안에서 유지해야 할 시간
        # Yaw 고정 옵션
        self.declare_parameter('lock_yaw', True)           # 초기 yaw를 고정하여 회전 방지
        self.declare_parameter('yaw_hold_strategy', 'angle')  # 'angle' or 'rate'

        # 식별용 Step (ENU 기준 v_z_cmd, duration_s)
        default_steps = [0.5, 0.0, -0.5, 0.0]
        # Longer steps for clearer steady-state observation
        default_durs = [20.0, 20.0, 20.0, 20.0]
        self.declare_parameter('step_values_enu', default_steps)
        self.declare_parameter('step_durations_s', default_durs)

        # 기본 저장 경로: 프로젝트 data 폴더
        default_csv_path = 'src/Control/system_identification/data/z/vz_step_log.csv'
        self.declare_parameter('csv_path', default_csv_path)
        # 로깅 정책
        self.declare_parameter('log_only_step', True)       # STEP 구간만 로깅
        self.declare_parameter('log_step_margin_s', 0.5)    # 각 스텝 시작/끝 가장자리 제외 시간
        self.declare_parameter('log_only_when_armed_offboard', True)

        # -------------------------
        # Load params
        # -------------------------
        self.rate_hz: float = float(self.get_parameter('rate_hz').value)
        self.dt = 1.0 / self.rate_hz

        self.alt_min = float(self.get_parameter('alt_min_m').value)
        self.alt_max = float(self.get_parameter('alt_max_m').value)

        self.arm_on_start = bool(self.get_parameter('arm_on_start').value)
        self.disarm_on_finish = bool(self.get_parameter('disarm_on_finish').value)
        self.enable_mode_switch = bool(self.get_parameter('enable_mode_switch').value)

        self.enable_auto_takeoff = bool(self.get_parameter('enable_auto_takeoff').value)
        self.hover_alt = float(self.get_parameter('hover_alt_m').value)
        self.takeoff_vz_enu = float(self.get_parameter('takeoff_vz_enu').value)
        self.hover_band = float(self.get_parameter('hover_band_m').value)
        self.hover_settle_s = float(self.get_parameter('hover_settle_s').value)
        self.warmup_sec = float(self.get_parameter('warmup_sec').value)
        self.lock_yaw = bool(self.get_parameter('lock_yaw').value)
        self.yaw_hold_strategy = str(self.get_parameter('yaw_hold_strategy').value)

        step_vals = self.get_parameter('step_values_enu').value
        step_durs = self.get_parameter('step_durations_s').value
        if len(step_vals) != len(step_durs) or len(step_vals) == 0:
            raise ValueError('step_values_enu 과 step_durations_s 길이가 같아야 하며, 최소 1개 이상이어야 합니다.')
        self.step_sequence: List[Tuple[float, float]] = list(
            zip([float(v) for v in step_vals], [float(d) for d in step_durs])
        )

        csv_path_param = str(self.get_parameter('csv_path').value)
        self.csv_path = os.path.abspath(os.path.expanduser(csv_path_param))
        # logging policy
        self.log_only_step = bool(self.get_parameter('log_only_step').value)
        self.log_step_margin_s = float(self.get_parameter('log_step_margin_s').value)
        self.log_only_when_armed_offboard = bool(self.get_parameter('log_only_when_armed_offboard').value)

        # -------------------------
        # State
        # -------------------------
        self.phase: Phase = Phase.INIT

        self.odom_ready = False
        self.odom_frame = 'UNKNOWN'
        self.z_enu = 0.0
        self.vz_enu_meas = 0.0
        self.vz_ned_meas = float('nan')
        self.yaw_enu = 0.0
        self._yaw_hold_enu = None  # 초기 yaw(ENU) 래치 값
        self.vehicle_status = None  # type: VehicleStatus | None

        # STEP phase state
        self.current_step_idx = -1
        self.current_step_start = None  # rclpy.time.Time | None
        self.current_step_end = None    # rclpy.time.Time | None
        self.active_cmd_vz_enu = 0.0

        # HOVER stabilize state
        self._hover_enter_time = None  # rclpy.time.Time | None

        # Prepare CSV
        self._prepare_csv(self.csv_path)

        # -------------------------
        # QoS Profiles
        # -------------------------
        qos_px4_out = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # -------------------------
        # Pubs / Subs
        # -------------------------
        self.pub_offboard = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.pub_traj = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.pub_cmd = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)

        # PX4 topic name differs by version: try both
        self._status_src = None
        self.sub_status = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            lambda m: self._on_vehicle_status(m, '/fmu/out/vehicle_status'), qos_px4_out
        )
        self.sub_status_alt = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1',
            lambda m: self._on_vehicle_status(m, '/fmu/out/vehicle_status_v1'), qos_px4_out
        )
        self.sub_odom = self.create_subscription(Odometry, '/uav/odom', self._on_odom, qos_reliable)
        self.sub_vo = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self._on_vehicle_odometry, qos_px4_out)

        # Timer
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(self.dt, self._on_timer)

        self.get_logger().info('[vz_step_commander] Initialized.')
        self.get_logger().info(f'  rate={self.rate_hz} Hz | steps(ENU)={self.step_sequence}')
        self.get_logger().info(f'  altitude bounds: [{self.alt_min:.2f}, {self.alt_max:.2f}] m')
        self.get_logger().info(f'  hover target: {self.hover_alt:.2f} m, band: ±{self.hover_band:.2f} m, settle: {self.hover_settle_s:.1f} s')
        self.get_logger().info(f'  csv path: {self.csv_path}')

    # -------------------------
    # Callbacks
    # -------------------------
    def _on_vehicle_status(self, msg: VehicleStatus, source: str = ''):
        self.vehicle_status = msg
        if self._status_src is None and source:
            self._status_src = source
            self.get_logger().info(f'vehicle_status source: {source}')

    def _on_odom(self, msg: Odometry):
        self.odom_ready = True
        self.odom_frame = msg.header.frame_id
        self.z_enu = float(msg.pose.pose.position.z)
        self.vz_enu_meas = float(msg.twist.twist.linear.z)
        # extract ENU yaw from quaternion
        import math
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        self.yaw_enu = math.atan2(siny_cosp, cosy_cosp)
        # 초기 yaw 래치(한 번만)
        if self.lock_yaw and self._yaw_hold_enu is None:
            self._yaw_hold_enu = self.yaw_enu

    def _on_vehicle_odometry(self, msg: VehicleOdometry):
        # NED 기준 z-속도(+D가 양수)
        try:
            if hasattr(msg, 'velocity') and len(msg.velocity) >= 3:
                self.vz_ned_meas = float(msg.velocity[2])
        except Exception:
            pass

    # -------------------------
    # Helpers
    # -------------------------
    def _prepare_csv(self, path: str):
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
        except Exception:
            pass
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow([
                't_sec', 'phase', 'step_idx',
                'v_z_cmd_ENU', 'v_z_cmd_NED_z',
                'odom_frame', 'z_ENU', 'vz_ENU_meas', 'vz_NED_meas',
                'alt_min', 'alt_max',
                'armed', 'offboard'
            ])

    def _append_csv(self, row: list):
        with open(self.csv_path, 'a', newline='') as f:
            csv.writer(f).writerow(row)

    def _is_armed(self) -> bool:
        if self.vehicle_status is None:
            return False
        return int(self.vehicle_status.arming_state) == 2  # ARMED

    def _is_offboard(self) -> bool:
        if self.vehicle_status is None:
            return False
        return int(self.vehicle_status.nav_state) == 14  # OFFBOARD

    def _send_vehicle_command(self, command: int, **kwargs):
        msg = VehicleCommand()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000  # usec
        msg.command = command
        msg.target_system = kwargs.get('target_system', 1)
        msg.target_component = kwargs.get('target_component', 1)
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        for i in range(1, 8):
            setattr(msg, f'param{i}', float(kwargs.get(f'param{i}', 0.0)))
        self.pub_cmd.publish(msg)

    def _arm(self):
        self._send_vehicle_command(400, param1=1.0)  # ARM
        self.get_logger().info('Sent ARM command.')

    def _disarm(self):
        self._send_vehicle_command(400, param1=0.0)  # DISARM
        self.get_logger().info('Sent DISARM command.')

    def _set_offboard_mode(self):
        # VEHICLE_CMD_DO_SET_MODE = 176
        # param1 = 1 (custom), param2 = 6 (PX4_CUSTOM_MAIN_MODE_OFFBOARD)
        self._send_vehicle_command(176, param1=1.0, param2=6.0, param3=0.0)
        self.get_logger().info('Requested OFFBOARD mode.')

    def _publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        if hasattr(msg, 'position'):            msg.position = False
        if hasattr(msg, 'velocity'):            msg.velocity = True
        if hasattr(msg, 'acceleration'):        msg.acceleration = False
        if hasattr(msg, 'attitude'):            msg.attitude = False
        if hasattr(msg, 'body_rate'):           msg.body_rate = False
        if hasattr(msg, 'actuator'):            msg.actuator = False
        if hasattr(msg, 'direct_actuator'):     msg.direct_actuator = False
        if hasattr(msg, 'thrust'):
            try: msg.thrust = False
            except Exception: pass
        if hasattr(msg, 'thrust_and_torque'):
            try: msg.thrust_and_torque = False
            except Exception: pass
        self.pub_offboard.publish(msg)

    def _publish_velocity_setpoint_ned(self, vz_cmd_enu: float):
        """
        TrajectorySetpoint (NED):
          ENU +z(위) -> NED z는 음수
        """
        vz_cmd_ned = -float(vz_cmd_enu)
        msg = TrajectorySetpoint()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000

        if hasattr(msg, 'position'):
            try:
                # position unused in pure velocity mode
                msg.position = [nan(), nan(), nan()]
            except Exception: pass
        if hasattr(msg, 'velocity'):
            try:
                # Hold XY velocity to 0.0 explicitly; command Z velocity
                msg.velocity = [0.0, 0.0, vz_cmd_ned]
            except Exception: pass
        if hasattr(msg, 'acceleration'):
            try: msg.acceleration = [nan(), nan(), nan()]
            except Exception: pass
        if hasattr(msg, 'jerk'):
            try: msg.jerk = [nan(), nan(), nan()]
            except Exception: pass
        if hasattr(msg, 'yaw'):
            try:
                if self.lock_yaw and self.yaw_hold_strategy == 'angle':
                    # 초기 yaw 고정(초기값이 없으면 현재값 사용)
                    yaw_use_enu = self._yaw_hold_enu if self._yaw_hold_enu is not None else self.yaw_enu
                    msg.yaw = float(enu_yaw_to_ned(yaw_use_enu))
                elif self.lock_yaw and self.yaw_hold_strategy == 'rate':
                    msg.yaw = nan()
                else:
                    msg.yaw = nan()
            except Exception:
                pass
        if hasattr(msg, 'yawspeed'):
            try:
                if self.lock_yaw and self.yaw_hold_strategy == 'rate':
                    msg.yawspeed = 0.0  # yaw rate 0으로 고정
                else:
                    msg.yawspeed = nan()
            except Exception:
                pass
        # 일부 버전엔 thrust 필드가 없음. 있더라도 여기선 사용 X.

        self.pub_traj.publish(msg)
        return vz_cmd_ned

    def _within_alt_bounds(self) -> bool:
        return (self.z_enu >= self.alt_min) and (self.z_enu <= self.alt_max)

    def _in_hover_band(self) -> bool:
        return abs(self.z_enu - self.hover_alt) <= self.hover_band

    def _enter_phase(self, new_phase: Phase):
        self.phase = new_phase
        if new_phase == Phase.HOVER_STABILIZE:
            self._hover_enter_time = None
        self.get_logger().info(f'>>> Phase changed to: {self.phase.name}')

    def _maybe_start_step_sequence(self):
        self.current_step_idx = 0
        self.active_cmd_vz_enu = float(self.step_sequence[0][0])
        now = self.get_clock().now()
        self.current_step_start = now
        self.current_step_end = now + Duration(seconds=float(self.step_sequence[0][1]))
        self._enter_phase(Phase.STEP)
        self.get_logger().info(
            f'*** Step started: v_z_cmd(ENU)={self.active_cmd_vz_enu:.3f} m/s '
            f'for {self.step_sequence[0][1]:.2f}s'
        )

    # -------------------------
    # Main Timer
    # -------------------------
    def _on_timer(self):
        now = self.get_clock().now()

        # keep-alive for offboard (reflect current control mode)
        self._publish_offboard_control_mode()

        # wait for odom
        if self.phase == Phase.INIT:
            # 항상 TrajectorySetpoint 스트림을 유지 (예열 포함)
            cmd_vz_enu = 0.0
            vz_cmd_ned = self._publish_velocity_setpoint_ned(cmd_vz_enu)
            self._log_row(now, cmd_vz_enu, vz_cmd_ned)

            # odom 없으면 대기
            if not self.odom_ready:
                return

            # 예열(warmup) 이후 ARM / OFFBOARD 반복 시도
            if (now - self.start_time) >= Duration(seconds=self.warmup_sec):
                if self.arm_on_start and not self._is_armed():
                    self._arm()
                if self.enable_mode_switch and not self._is_offboard():
                    self._set_offboard_mode()

                # 모드 전환 성공 여부와 무관하게 다음 단계로 진행하여 TAKEOFF 명령 송신
                if self.enable_auto_takeoff:
                    self._enter_phase(Phase.TAKEOFF)
                else:
                    self._enter_phase(Phase.HOVER_STABILIZE)
            return

        # TAKEOFF: 목표고도까지 상승 명령
        if self.phase == Phase.TAKEOFF:
            # Offboard/ARM 보장: 아직 아니면 계속 시도
            if self.arm_on_start and not self._is_armed():
                self._arm()
            if self.enable_mode_switch and not self._is_offboard():
                self._set_offboard_mode()
            # 안전 범위 내인지 체크 (min/max 밖이면 그래도 목표를 향해 올라가되 max는 넘기지 않게 설정 가능)
            cmd_vz_enu = self.takeoff_vz_enu
            # alt_max를 너무 낮게 두면 목표에 못 가니, alt_max는 hover_alt보다 크게 설정 권장
            if self.z_enu >= self.hover_alt - self.hover_band:
                # 목표 근처 도달 → 호버 안정화 단계
                cmd_vz_enu = 0.0
                self._enter_phase(Phase.HOVER_STABILIZE)

            vz_cmd_ned = self._publish_velocity_setpoint_ned(cmd_vz_enu)
            self._log_row(now, cmd_vz_enu, vz_cmd_ned)
            return

        # HOVER_STABILIZE: 목표고도 근방에서 일정 시간 유지
        if self.phase == Phase.HOVER_STABILIZE:
            cmd_vz_enu = 0.0  # 호버 명령
            if self._in_hover_band():
                if self._hover_enter_time is None:
                    self._hover_enter_time = now
                # 밴드 안에서 지정된 settle 시간 유지되면 스텝 시작
                if (now - self._hover_enter_time) >= Duration(seconds=self.hover_settle_s):
                    self._maybe_start_step_sequence()
            else:
                # 밴드 밖으로 나가면 타이머 리셋
                self._hover_enter_time = None

            vz_cmd_ned = self._publish_velocity_setpoint_ned(cmd_vz_enu)
            self._log_row(now, cmd_vz_enu, vz_cmd_ned)
            return

        # STEP: 식별용 스텝 시퀀스
        if self.phase == Phase.STEP:
            # 안전 범위 밖이면 명령 0, 그리고 **스텝 진행을 일시 정지** (중요!)
            safe = self._within_alt_bounds()
            cmd_vz_enu = self.active_cmd_vz_enu if safe else 0.0
            vz_cmd_ned = self._publish_velocity_setpoint_ned(cmd_vz_enu)
            self._log_row(now, cmd_vz_enu, vz_cmd_ned)

            # 스텝 전환은 안전할 때만 진행
            if safe and (self.current_step_end is not None) and (now >= self.current_step_end):
                self.current_step_idx += 1
                if self.current_step_idx >= len(self.step_sequence):
                    self.active_cmd_vz_enu = 0.0
                    self._enter_phase(Phase.DONE)
                    return
                self.active_cmd_vz_enu = float(self.step_sequence[self.current_step_idx][0])
                dur = float(self.step_sequence[self.current_step_idx][1])
                self.current_step_start = now
                self.current_step_end = now + Duration(seconds=dur)
                self.get_logger().info(
                    f'*** Next step[{self.current_step_idx}]: v_z_cmd(ENU)={self.active_cmd_vz_enu:.3f} m/s '
                    f'for {dur:.2f}s'
                )
            return

        # DONE: 마무리
        if self.phase == Phase.DONE:
            # 마지막으로 0 명령 유지
            cmd_vz_enu = 0.0
            vz_cmd_ned = self._publish_velocity_setpoint_ned(cmd_vz_enu)
            self._log_row(now, cmd_vz_enu, vz_cmd_ned)
            if self.disarm_on_finish and self._is_armed():
                self._disarm()
            return

    # -------------------------
    # Utilities
    # -------------------------
    def _should_log(self, now) -> bool:
        # Armed+Offboard 조건
        if self.log_only_when_armed_offboard and not (self._is_armed() and self._is_offboard()):
            return False
        # STEP 구간만 로깅
        if self.log_only_step:
            if self.phase != Phase.STEP:
                return False
            if self.current_step_start is None or self.current_step_end is None:
                return False
            # 스텝 가장자리는 제외
            if (now - self.current_step_start) < Duration(seconds=self.log_step_margin_s):
                return False
            if (self.current_step_end - now) < Duration(seconds=self.log_step_margin_s):
                return False
        return True

    def _log_row(self, now, cmd_vz_enu: float, vz_cmd_ned: float):
        if not self._should_log(now):
            return
        armed = self._is_armed()
        offb = self._is_offboard()
        t_sec = now.nanoseconds * 1e-9
        self._append_csv([
            f'{t_sec:.6f}', self.phase.name, self.current_step_idx,
            f'{cmd_vz_enu:.6f}', f'{vz_cmd_ned:.6f}',
            self.odom_frame, f'{self.z_enu:.6f}', f'{self.vz_enu_meas:.6f}', f'{self.vz_ned_meas:.6f}',
            f'{self.alt_min:.3f}', f'{self.alt_max:.3f}',
            int(armed), int(offb)
        ])


def main(args=None):
    rclpy.init(args=args)
    node = VzStepCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f'vz_step_commander shutting down... CSV saved at: {node.csv_path}')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
