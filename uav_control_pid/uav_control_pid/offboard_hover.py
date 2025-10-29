import math
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from nav_msgs.msg import Odometry
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand


def wrap_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def enu_to_ned_pos(x_enu: float, y_enu: float, z_enu: float):
    # ENU -> NED
    return y_enu, x_enu, -z_enu


def enu_yaw_to_ned(yaw_enu: float) -> float:
    # yaw_ned = pi/2 - yaw_enu (wrap to [-pi, pi])
    return wrap_pi(math.pi / 2.0 - yaw_enu)


class OffboardHover(Node):
    def __init__(self):
        super().__init__('offboard_hover')

        # Params
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('climb_m', 2.0)
        self.declare_parameter('arm_on_start', True)
        self.declare_parameter('warmup_sec', 0.5)

        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.climb_m = float(self.get_parameter('climb_m').value)
        self.arm_on_start = bool(self.get_parameter('arm_on_start').value)
        self.warmup_sec = float(self.get_parameter('warmup_sec').value)

        # State from /uav/odom (ENU)
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.sub_odom = self.create_subscription(Odometry, '/uav/odom', self.cb_odom, qos)

        # Publishers to PX4
        self.pub_offboard = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.pub_traj = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.pub_cmd = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)

        # Internal state
        self.have_home = False
        self.home_xy_enu = (0.0, 0.0)
        self.home_z_enu = 0.0
        self.home_yaw_enu = 0.0
        self.target_z_enu = 0.0

        self.armed = False
        self.offboard = False
        self.start_time = self.get_clock().now()
        self.lock = threading.Lock()

        # Main timer
        period = 1.0 / max(1.0, self.rate_hz)
        self.timer = self.create_timer(period, self.tick)
        self.get_logger().info(f'Offboard hover node started @ {self.rate_hz:.1f} Hz, climb {self.climb_m} m')

    def now_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    def cb_odom(self, msg: Odometry):
        with self.lock:
            if not self.have_home:
                # Initialize home (hold XY, climb in Z, keep yaw)
                x = msg.pose.pose.position.x
                y = msg.pose.pose.position.y
                z = msg.pose.pose.position.z
                # Yaw from quaternion
                qx = msg.pose.pose.orientation.x
                qy = msg.pose.pose.orientation.y
                qz = msg.pose.pose.orientation.z
                qw = msg.pose.pose.orientation.w
                # ENU yaw from quaternion
                # yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
                siny_cosp = 2.0 * (qw * qz + qx * qy)
                cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
                yaw = math.atan2(siny_cosp, cosy_cosp)

                self.home_xy_enu = (x, y)
                self.home_z_enu = z
                self.home_yaw_enu = yaw
                self.target_z_enu = z + self.climb_m
                self.have_home = True
                self.get_logger().info(
                    f'Home set from /uav/odom: xy=({x:.2f},{y:.2f}), z={z:.2f} → target_z={self.target_z_enu:.2f}, yaw={yaw:.2f} rad'
                )

    def publish_offboard_mode(self):
        m = OffboardControlMode()
        m.timestamp = self.now_us()
        m.position = True
        m.velocity = False
        m.acceleration = False
        m.attitude = False
        m.body_rate = False
        m.thrust_and_torque = False
        m.direct_actuator = False
        self.pub_offboard.publish(m)

    def publish_traj_setpoint(self):
        if not self.have_home:
            return
        x_enu, y_enu = self.home_xy_enu
        z_enu = self.target_z_enu
        yaw_enu = self.home_yaw_enu

        x_ned, y_ned, z_ned = enu_to_ned_pos(x_enu, y_enu, z_enu)
        yaw_ned = enu_yaw_to_ned(yaw_enu)

        t = TrajectorySetpoint()
        t.timestamp = self.now_us()
        t.position = [float(x_ned), float(y_ned), float(z_ned)]
        t.velocity = [math.nan, math.nan, math.nan]
        t.acceleration = [math.nan, math.nan, math.nan]
        t.jerk = [math.nan, math.nan, math.nan]
        t.yaw = float(yaw_ned)
        t.yawspeed = math.nan
        self.pub_traj.publish(t)

    def send_vehicle_command(self, command: int, param1: float = 0.0, param2: float = 0.0, param3: float = 0.0, param4: float = 0.0, param5: float = 0.0, param6: float = 0.0, param7: float = 0.0):
        msg = VehicleCommand()
        msg.timestamp = self.now_us()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = float(param3)
        msg.param4 = float(param4)
        msg.param5 = float(param5)
        msg.param6 = float(param6)
        msg.param7 = float(param7)
        msg.command = int(command)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.pub_cmd.publish(msg)

    def try_arm_and_offboard(self):
        # Warm-up before mode switching (PX4 expects prior setpoint stream)
        dt = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if dt < self.warmup_sec:
            return
        if self.arm_on_start and not self.armed:
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=float(VehicleCommand.ARMING_ACTION_ARM))
            self.get_logger().info('Sent ARM command')
            self.armed = True
        if self.armed and not self.offboard:
            # VEHICLE_CMD_DO_SET_MODE: param1=1 (custom), param2=6 (PX4_CUSTOM_MAIN_MODE_OFFBOARD)
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
            self.get_logger().info('Requested OFFBOARD mode')
            self.offboard = True

    def tick(self):
        # Always publish offboard control mode & setpoint at high rate
        self.publish_offboard_mode()
        self.publish_traj_setpoint()
        # Attempt arming & offboard after warm-up
        self.try_arm_and_offboard()


def main():
    rclpy.init()
    node = OffboardHover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
