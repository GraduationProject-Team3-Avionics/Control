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
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
    VehicleOdometry,
)


def nan():
    return float("nan")


def wrap_pi(angle: float) -> float:
    import math

    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def enu_yaw_to_ned(yaw_enu: float) -> float:
    # yaw_ned = pi/2 - yaw_enu (wrap)
    import math

    return wrap_pi(math.pi / 2.0 - yaw_enu)


class Phase(Enum):
    INIT = auto()
    TAKEOFF = auto()
    HOVER_STABILIZE = auto()
    STEP = auto()
    DONE = auto()


class VxStepCommander(Node):
    """
    X축(NED x, 북/앞) 속도 스텝 입력 퍼블리셔.
    z로 올려서 호버 → 그 위에서 x속도 step을 줘서 식별용 CSV를 남긴다.
    흐름: INIT -> TAKEOFF -> HOVER_STABILIZE -> STEP -> DONE
    """

    def __init__(self):
        super().__init__("vx_step_commander")

        # -------------------------
        # Parameters
        # -------------------------
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("alt_min_m", 0.5)
        self.declare_parameter("alt_max_m", 5.0)
        self.declare_parameter("arm_on_start", True)
        self.declare_parameter("disarm_on_finish", False)
        self.declare_parameter("enable_mode_switch", True)

        # z 이륙/호버 관련
        self.declare_parameter("enable_auto_takeoff", True)
        self.declare_parameter("warmup_sec", 0.5)
        self.declare_parameter("hover_alt_m", 1.5)
        self.declare_parameter("takeoff_vz_enu", 0.5)
        self.declare_parameter("hover_band_m", 0.10)
        self.declare_parameter("hover_settle_s", 2.0)

        # yaw 고정
        self.declare_parameter("lock_yaw", True)
        self.declare_parameter("yaw_hold_strategy", "angle")  # 'angle' or 'rate'

        # X축 스텝 (NED x 기준)
        # 너가 말한 대로 20초씩으로 쓰고 싶으면 여기 파라미터에서 바꿔주면 됨
        default_steps = [0.5, 0.0, -0.5, 0.0]   # m/s
        default_durs = [20.0, 20.0, 20.0, 20.0]  # s
        self.declare_parameter("step_values_ned_x", default_steps)
        self.declare_parameter("step_durations_s", default_durs)

        # CSV 경로
        pkg_root = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
        default_csv_path = os.path.join(pkg_root, "data", "x", "vx_step_log.csv")
        self.declare_parameter("csv_path", default_csv_path)

        # 로깅 정책
        self.declare_parameter("log_only_step", True)
        self.declare_parameter("log_step_margin_s", 0.5)
        self.declare_parameter("log_only_when_armed_offboard", True)

        # -------------------------
        # Load params
        # -------------------------
        self.rate_hz: float = float(self.get_parameter("rate_hz").value)
        self.dt = 1.0 / self.rate_hz

        self.alt_min = float(self.get_parameter("alt_min_m").value)
        self.alt_max = float(self.get_parameter("alt_max_m").value)

        self.arm_on_start = bool(self.get_parameter("arm_on_start").value)
        self.disarm_on_finish = bool(self.get_parameter("disarm_on_finish").value)
        self.enable_mode_switch = bool(self.get_parameter("enable_mode_switch").value)

        self.enable_auto_takeoff = bool(self.get_parameter("enable_auto_takeoff").value)
        self.hover_alt = float(self.get_parameter("hover_alt_m").value)
        self.takeoff_vz_enu = float(self.get_parameter("takeoff_vz_enu").value)
        self.hover_band = float(self.get_parameter("hover_band_m").value)
        self.hover_settle_s = float(self.get_parameter("hover_settle_s").value)
        self.warmup_sec = float(self.get_parameter("warmup_sec").value)
        self.lock_yaw = bool(self.get_parameter("lock_yaw").value)
        self.yaw_hold_strategy = str(self.get_parameter("yaw_hold_strategy").value)

        step_vals = self.get_parameter("step_values_ned_x").value
        step_durs = self.get_parameter("step_durations_s").value
        if len(step_vals) != len(step_durs) or len(step_vals) == 0:
            raise ValueError("step_values_ned_x 와 step_durations_s 길이가 같아야 하고 1개 이상이어야 합니다.")
        self.step_sequence: List[Tuple[float, float]] = list(
            zip([float(v) for v in step_vals], [float(d) for d in step_durs])
        )

        csv_path_param = str(self.get_parameter("csv_path").value)
        self.csv_path = os.path.abspath(os.path.expanduser(csv_path_param))

        self.log_only_step = bool(self.get_parameter("log_only_step").value)
        self.log_step_margin_s = float(self.get_parameter("log_step_margin_s").value)
        self.log_only_when_armed_offboard = bool(
            self.get_parameter("log_only_when_armed_offboard").value
        )

        # -------------------------
        # State
        # -------------------------
        self.phase: Phase = Phase.INIT

        self.odom_ready = False
        self.odom_frame = "UNKNOWN"
        self.z_enu = 0.0
        self.vz_enu_meas = 0.0

        # ENU 속도
        self.vx_enu_meas = 0.0            # ENU x (east)
        self.vx_from_odom_ned_x = 0.0     # ENU y (north) == NED x  ← 이걸 쓸 거다

        # PX4 vehicle_odometry 쪽
        self.vx_ned_meas = float("nan")

        self.yaw_enu = 0.0
        self._yaw_hold_enu = None

        self.vehicle_status = None  # type: VehicleStatus | None

        # STEP 상태
        self.current_step_idx = -1
        self.current_step_start = None
        self.current_step_end = None
        self.active_cmd_vx_ned = 0.0

        # 호버 상태
        self._hover_enter_time = None

        # CSV 준비
        self._prepare_csv(self.csv_path)

        # -------------------------
        # QoS
        # -------------------------
        qos_px4_out = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # -------------------------
        # Pub/Sub
        # -------------------------
        self.pub_offboard = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", 10
        )
        self.pub_traj = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10
        )
        self.pub_cmd = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", 10
        )

        self._status_src = None
        self.sub_status = self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status",
            lambda m: self._on_vehicle_status(m, "/fmu/out/vehicle_status"),
            qos_px4_out,
        )
        self.sub_status_alt = self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status_v1",
            lambda m: self._on_vehicle_status(m, "/fmu/out/vehicle_status_v1"),
            qos_px4_out,
        )

        self.sub_odom = self.create_subscription(
            Odometry, "/uav/odom", self._on_odom, qos_reliable
        )
        self.sub_vo = self.create_subscription(
            VehicleOdometry,
            "/fmu/out/vehicle_odometry",
            self._on_vehicle_odometry,
            qos_px4_out,
        )

        # Timer
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(self.dt, self._on_timer)

        self.get_logger().info("[vx_step_commander] Initialized.")
        self.get_logger().info(f"  rate={self.rate_hz} Hz | steps(NED x)={self.step_sequence}")
        self.get_logger().info(
            f"  altitude bounds: [{self.alt_min:.2f}, {self.alt_max:.2f}] m"
        )
        self.get_logger().info(
            f"  hover target: {self.hover_alt:.2f} m, band: ±{self.hover_band:.2f} m, settle: {self.hover_settle_s:.1f} s"
        )
        self.get_logger().info(f"  csv path: {self.csv_path}")

    # -------------------------
    # Callbacks
    # -------------------------
    def _on_vehicle_status(self, msg: VehicleStatus, source: str = ""):
        self.vehicle_status = msg
        if self._status_src is None and source:
            self._status_src = source
            self.get_logger().info(f"vehicle_status source: {source}")

    def _on_odom(self, msg: Odometry):
        self.odom_ready = True
        self.odom_frame = msg.header.frame_id

        self.z_enu = float(msg.pose.pose.position.z)
        self.vz_enu_meas = float(msg.twist.twist.linear.z)

        # 여기서 실제로 본 그래프 기준으로:
        # /uav/odom/twist/twist/linear.y  가 NED x 와 같았다.
        self.vx_enu_meas = float(msg.twist.twist.linear.x)         # east
        self.vx_from_odom_ned_x = float(msg.twist.twist.linear.y)  # north == NED x

        # yaw 추출
        import math
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        self.yaw_enu = math.atan2(siny_cosp, cosy_cosp)
        if self.lock_yaw and self._yaw_hold_enu is None:
            self._yaw_hold_enu = self.yaw_enu

    def _on_vehicle_odometry(self, msg: VehicleOdometry):
        # PX4 쪽에서 오는 속도 (버전에 따라 프레임 다를 수 있음)
        try:
            if hasattr(msg, "velocity") and len(msg.velocity) >= 3:
                self.vx_ned_meas = float(msg.velocity[0])
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
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(
                [
                    "t_sec",
                    "phase",
                    "step_idx",
                    "v_x_cmd_NED_x",
                    "odom_frame",
                    "z_ENU",
                    "vz_ENU_meas",
                    "vx_ENU_meas",
                    "vx_from_odom_NED_x",  # ENU y == NED x
                    "vx_NED_meas",          # vehicle_odometry에서 온 것
                    "alt_min",
                    "alt_max",
                    "armed",
                    "offboard",
                ]
            )

    def _append_csv(self, row: list):
        with open(self.csv_path, "a", newline="") as f:
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
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.command = command
        msg.target_system = kwargs.get("target_system", 1)
        msg.target_component = kwargs.get("target_component", 1)
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        for i in range(1, 8):
            setattr(msg, f"param{i}", float(kwargs.get(f"param{i}", 0.0)))
        self.pub_cmd.publish(msg)

    def _arm(self):
        self._send_vehicle_command(400, param1=1.0)
        self.get_logger().info("Sent ARM command.")

    def _disarm(self):
        self._send_vehicle_command(400, param1=0.0)
        self.get_logger().info("Sent DISARM command.")

    def _set_offboard_mode(self):
        self._send_vehicle_command(176, param1=1.0, param2=6.0, param3=0.0)
        self.get_logger().info("Requested OFFBOARD mode.")

    def _publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        if hasattr(msg, "position"):
            msg.position = False
        if hasattr(msg, "velocity"):
            msg.velocity = True
        if hasattr(msg, "acceleration"):
            msg.acceleration = False
        if hasattr(msg, "attitude"):
            msg.attitude = False
        if hasattr(msg, "body_rate"):
            msg.body_rate = False
        if hasattr(msg, "actuator"):
            msg.actuator = False
        if hasattr(msg, "direct_actuator"):
            msg.direct_actuator = False
        if hasattr(msg, "thrust"):
            try:
                msg.thrust = False
            except Exception:
                pass
        if hasattr(msg, "thrust_and_torque"):
            try:
                msg.thrust_and_torque = False
            except Exception:
                pass
        self.pub_offboard.publish(msg)

    def _publish_velocity_setpoint_x_only(self, vx_cmd_ned: float):
        """
        STEP 단계에서 x만 주고 나머지는 0으로 두는 모드
        """
        msg = TrajectorySetpoint()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000

        if hasattr(msg, "position"):
            try:
                msg.position = [nan(), nan(), nan()]
            except Exception:
                pass

        if hasattr(msg, "velocity"):
            try:
                msg.velocity = [float(vx_cmd_ned), 0.0, 0.0]
            except Exception:
                pass

        if hasattr(msg, "acceleration"):
            try:
                msg.acceleration = [nan(), nan(), nan()]
            except Exception:
                pass
        if hasattr(msg, "jerk"):
            try:
                msg.jerk = [nan(), nan(), nan()]
            except Exception:
                pass

        # yaw 고정
        if hasattr(msg, "yaw"):
            try:
                if self.lock_yaw and self.yaw_hold_strategy == "angle":
                    yaw_use_enu = (
                        self._yaw_hold_enu if self._yaw_hold_enu is not None else self.yaw_enu
                    )
                    msg.yaw = float(enu_yaw_to_ned(yaw_use_enu))
                elif self.lock_yaw and self.yaw_hold_strategy == "rate":
                    msg.yaw = nan()
                else:
                    msg.yaw = nan()
            except Exception:
                pass

        if hasattr(msg, "yawspeed"):
            try:
                if self.lock_yaw and self.yaw_hold_strategy == "rate":
                    msg.yawspeed = 0.0
                else:
                    msg.yawspeed = nan()
            except Exception:
                pass

        self.pub_traj.publish(msg)

    def _publish_takeoff_velocity_setpoint(self, vz_cmd_enu: float):
        """
        이륙할 때: x,y는 0, z만 위로(+ENU)
        """
        msg = TrajectorySetpoint()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000

        vz_cmd_ned = -float(vz_cmd_enu)

        if hasattr(msg, "position"):
            try:
                msg.position = [nan(), nan(), nan()]
            except Exception:
                pass

        if hasattr(msg, "velocity"):
            try:
                msg.velocity = [0.0, 0.0, vz_cmd_ned]
            except Exception:
                pass

        if hasattr(msg, "acceleration"):
            try:
                msg.acceleration = [nan(), nan(), nan()]
            except Exception:
                pass
        if hasattr(msg, "jerk"):
            try:
                msg.jerk = [nan(), nan(), nan()]
            except Exception:
                pass

        # yaw 고정
        if hasattr(msg, "yaw"):
            try:
                if self.lock_yaw and self.yaw_hold_strategy == "angle":
                    yaw_use_enu = (
                        self._yaw_hold_enu if self._yaw_hold_enu is not None else self.yaw_enu
                    )
                    msg.yaw = float(enu_yaw_to_ned(yaw_use_enu))
                elif self.lock_yaw and self.yaw_hold_strategy == "rate":
                    msg.yaw = nan()
                else:
                    msg.yaw = nan()
            except Exception:
                pass

        if hasattr(msg, "yawspeed"):
            try:
                if self.lock_yaw and self.yaw_hold_strategy == "rate":
                    msg.yawspeed = 0.0
                else:
                    msg.yawspeed = nan()
            except Exception:
                pass

        self.pub_traj.publish(msg)

    def _within_alt_bounds(self) -> bool:
        return (self.z_enu >= self.alt_min) and (self.z_enu <= self.alt_max)

    def _in_hover_band(self) -> bool:
        return abs(self.z_enu - self.hover_alt) <= self.hover_band

    def _enter_phase(self, new_phase: Phase):
        self.phase = new_phase
        if new_phase == Phase.HOVER_STABILIZE:
            self._hover_enter_time = None
        self.get_logger().info(f">>> Phase changed to: {self.phase.name}")

    def _maybe_start_step_sequence(self):
        self.current_step_idx = 0
        self.active_cmd_vx_ned = float(self.step_sequence[0][0])
        now = self.get_clock().now()
        self.current_step_start = now
        self.current_step_end = now + Duration(seconds=float(self.step_sequence[0][1]))
        self._enter_phase(Phase.STEP)
        self.get_logger().info(
            f"*** Step started: v_x_cmd(NED)={self.active_cmd_vx_ned:.3f} m/s "
            f"for {self.step_sequence[0][1]:.2f}s"
        )

    # -------------------------
    # Main timer
    # -------------------------
    def _on_timer(self):
        now = self.get_clock().now()

        # offboard keep-alive
        self._publish_offboard_control_mode()

        # INIT
        if self.phase == Phase.INIT:
            # x속도 0으로 계속 스트림
            self._publish_velocity_setpoint_x_only(0.0)
            self._log_row(now, 0.0)

            if not self.odom_ready:
                return

            if (now - self.start_time) >= Duration(seconds=self.warmup_sec):
                if self.arm_on_start and not self._is_armed():
                    self._arm()
                if self.enable_mode_switch and not self._is_offboard():
                    self._set_offboard_mode()

                if self.enable_auto_takeoff:
                    self._enter_phase(Phase.TAKEOFF)
                else:
                    self._enter_phase(Phase.HOVER_STABILIZE)
            return

        # TAKEOFF
        if self.phase == Phase.TAKEOFF:
            if self.arm_on_start and not self._is_armed():
                self._arm()
            if self.enable_mode_switch and not self._is_offboard():
                self._set_offboard_mode()

            # z만 위로
            self._publish_takeoff_velocity_setpoint(self.takeoff_vz_enu)
            self._log_row(now, 0.0)

            if self.z_enu >= self.hover_alt - self.hover_band:
                self._enter_phase(Phase.HOVER_STABILIZE)
            return

        # HOVER_STABILIZE
        if self.phase == Phase.HOVER_STABILIZE:
            self._publish_velocity_setpoint_x_only(0.0)

            if self._in_hover_band():
                if self._hover_enter_time is None:
                    self._hover_enter_time = now
                if (now - self._hover_enter_time) >= Duration(seconds=self.hover_settle_s):
                    self._maybe_start_step_sequence()
            else:
                self._hover_enter_time = None

            self._log_row(now, 0.0)
            return

        # STEP
        if self.phase == Phase.STEP:
            safe = self._within_alt_bounds()
            cmd_vx = self.active_cmd_vx_ned if safe else 0.0
            self._publish_velocity_setpoint_x_only(cmd_vx)
            self._log_row(now, cmd_vx)

            # 스텝 전환
            if safe and (self.current_step_end is not None) and (now >= self.current_step_end):
                self.current_step_idx += 1
                if self.current_step_idx >= len(self.step_sequence):
                    self.active_cmd_vx_ned = 0.0
                    self._enter_phase(Phase.DONE)
                    return
                self.active_cmd_vx_ned = float(self.step_sequence[self.current_step_idx][0])
                dur = float(self.step_sequence[self.current_step_idx][1])
                self.current_step_start = now
                self.current_step_end = now + Duration(seconds=dur)
                self.get_logger().info(
                    f"*** Next step[{self.current_step_idx}]: v_x_cmd(NED)={self.active_cmd_vx_ned:.3f} m/s for {dur:.2f}s"
                )
            return

        # DONE
        if self.phase == Phase.DONE:
            self._publish_velocity_setpoint_x_only(0.0)
            self._log_row(now, 0.0)
            if self.disarm_on_finish and self._is_armed():
                self._disarm()
            return

    # -------------------------
    # Logging helpers
    # -------------------------
    def _should_log(self, now) -> bool:
        if self.log_only_when_armed_offboard and not (
            self._is_armed() and self._is_offboard()
        ):
            return False
        if self.log_only_step:
            if self.phase != Phase.STEP:
                return False
            if self.current_step_start is None or self.current_step_end is None:
                return False
            if (now - self.current_step_start) < Duration(seconds=self.log_step_margin_s):
                return False
            if (self.current_step_end - now) < Duration(seconds=self.log_step_margin_s):
                return False
        return True

    def _log_row(self, now, cmd_vx_ned: float):
        if not self._should_log(now):
            return
        armed = self._is_armed()
        offb = self._is_offboard()
        t_sec = now.nanoseconds * 1e-9
        self._append_csv(
            [
                f"{t_sec:.6f}",
                self.phase.name,
                self.current_step_idx,
                f"{cmd_vx_ned:.6f}",
                self.odom_frame,
                f"{self.z_enu:.6f}",
                f"{self.vz_enu_meas:.6f}",
                f"{self.vx_enu_meas:.6f}",
                f"{self.vx_from_odom_ned_x:.6f}",  # 여기!
                f"{self.vx_ned_meas:.6f}",
                f"{self.alt_min:.3f}",
                f"{self.alt_max:.3f}",
                int(armed),
                int(offb),
            ]
        )


def main(args=None):
    rclpy.init(args=args)
    node = VxStepCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f"vx_step_commander shutting down... CSV saved at: {node.csv_path}"
        )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
