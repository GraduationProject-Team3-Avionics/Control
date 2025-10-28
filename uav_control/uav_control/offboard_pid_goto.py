import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from tf2_ros import Buffer, TransformListener

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def enu_vel_to_ned(vx_e: float, vy_e: float, vz_e: float):
    # ENU -> NED velocity mapping
    return vy_e, vx_e, -vz_e


def enu_yawspeed_to_ned(yawspeed_enu: float) -> float:
    # Positive yaw in ENU is CCW about Up(+Z). In NED, yaw rate positive is CW about Down(+Z).
    return -yawspeed_enu


class OffboardPidGoto(Node):
    def __init__(self):
        super().__init__('offboard_pid_goto')

        # Parameters
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('arm_on_start', True)
        self.declare_parameter('warmup_sec', 0.5)
        # If odom is yaw-rotated relative to map (as in this repo),
        # rotate ENU velocities from odom frame into map before ENU->NED mapping.
        self.declare_parameter('apply_odom_to_map_yaw_correction', True)

        # Goal in ENU odom frame
        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_z', 2.0)
        self.declare_parameter('goal_yaw', 0.0)

        # PID gains (default: I,D=0)
        self.declare_parameter('kp_xy', 0.8)
        self.declare_parameter('ki_xy', 0.0)
        self.declare_parameter('kd_xy', 0.0)
        self.declare_parameter('kp_z', 0.8)
        self.declare_parameter('ki_z', 0.0)
        self.declare_parameter('kd_z', 0.0)
        self.declare_parameter('kp_yaw', 1.0)
        self.declare_parameter('ki_yaw', 0.0)
        self.declare_parameter('kd_yaw', 0.0)

        # Limits
        self.declare_parameter('max_speed_xy', 1.0)
        self.declare_parameter('max_speed_z', 0.8)
        self.declare_parameter('max_yawspeed', 0.8)

        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.arm_on_start = bool(self.get_parameter('arm_on_start').value)
        self.warmup_sec = float(self.get_parameter('warmup_sec').value)
        self.apply_yaw_corr = bool(self.get_parameter('apply_odom_to_map_yaw_correction').value)

        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.goal_z = float(self.get_parameter('goal_z').value)
        self.goal_yaw = float(self.get_parameter('goal_yaw').value)

        self.kp_xy = float(self.get_parameter('kp_xy').value)
        self.ki_xy = float(self.get_parameter('ki_xy').value)
        self.kd_xy = float(self.get_parameter('kd_xy').value)
        self.kp_z = float(self.get_parameter('kp_z').value)
        self.ki_z = float(self.get_parameter('ki_z').value)
        self.kd_z = float(self.get_parameter('kd_z').value)
        self.kp_yaw = float(self.get_parameter('kp_yaw').value)
        self.ki_yaw = float(self.get_parameter('ki_yaw').value)
        self.kd_yaw = float(self.get_parameter('kd_yaw').value)

        self.max_speed_xy = float(self.get_parameter('max_speed_xy').value)
        self.max_speed_z = float(self.get_parameter('max_speed_z').value)
        self.max_yawspeed = float(self.get_parameter('max_yawspeed').value)

        # State subscription
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST,
                         depth=5)
        self.sub_odom = self.create_subscription(Odometry, '/uav/odom', self.cb_odom, qos)

        # Publishers to PX4
        self.pub_offboard = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.pub_traj = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.pub_cmd = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)

        # Visualization/monitoring publishers
        self.pub_goal_pose = self.create_publisher(PoseStamped, '/uav_control/goal_pose', 10)
        self.pub_goal_marker = self.create_publisher(Marker, '/uav_control/goal_marker', 10)

        # Internals
        self.have_state = False
        self.x = self.y = self.z = 0.0
        self.yaw = 0.0

        self.armed = False
        self.offboard = False
        self.start_time = self.get_clock().now()
        self.lock = threading.Lock()

        # PID state
        self.prev_time = None
        self.prev_ex = 0.0
        self.prev_ey = 0.0
        self.prev_ez = 0.0
        self.prev_eyaw = 0.0
        self.sum_ex = 0.0
        self.sum_ey = 0.0
        self.sum_ez = 0.0
        self.sum_eyaw = 0.0

        # Timer
        period = 1.0 / max(1.0, self.rate_hz)
        self.timer = self.create_timer(period, self.tick)
        self.get_logger().info(
            f'Offboard PID goto started @ {self.rate_hz:.1f} Hz, goal=({self.goal_x:.1f},{self.goal_y:.1f},{self.goal_z:.1f}, yaw={self.goal_yaw:.2f})')

        # TF buffer for map->odom yaw (to rotate odom ENU to map ENU)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.yaw_map_to_odom = 0.0
        self.have_yaw_corr = False

    def now_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    def cb_odom(self, msg: Odometry):
        with self.lock:
            self.x = msg.pose.pose.position.x
            self.y = msg.pose.pose.position.y
            self.z = msg.pose.pose.position.z
            qx = msg.pose.pose.orientation.x
            qy = msg.pose.pose.orientation.y
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w
            # ENU yaw
            siny_cosp = 2.0 * (qw * qz + qx * qy)
            cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
            self.yaw = math.atan2(siny_cosp, cosy_cosp)
            self.have_state = True

    def publish_offboard_mode(self):
        m = OffboardControlMode()
        m.timestamp = self.now_us()
        m.position = False
        m.velocity = True
        m.acceleration = False
        m.attitude = False
        m.body_rate = False
        m.thrust_and_torque = False
        m.direct_actuator = False
        self.pub_offboard.publish(m)

    def publish_velocity_setpoint(self, vx_e: float, vy_e: float, vz_e: float, yawspeed_e: float):
        vx_n, vy_n, vz_n = enu_vel_to_ned(vx_e, vy_e, vz_e)
        ys_n = enu_yawspeed_to_ned(yawspeed_e)
        t = TrajectorySetpoint()
        t.timestamp = self.now_us()
        t.position = [math.nan, math.nan, math.nan]
        t.velocity = [float(vx_n), float(vy_n), float(vz_n)]
        t.acceleration = [math.nan, math.nan, math.nan]
        t.jerk = [math.nan, math.nan, math.nan]
        t.yaw = math.nan
        t.yawspeed = float(ys_n)
        self.pub_traj.publish(t)

    def send_vehicle_command(self, command: int, param1: float = 0.0, param2: float = 0.0):
        msg = VehicleCommand()
        msg.timestamp = self.now_us()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = 0.0
        msg.param4 = 0.0
        msg.param5 = 0.0
        msg.param6 = 0.0
        msg.param7 = 0.0
        msg.command = int(command)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.pub_cmd.publish(msg)

    def try_arm_and_offboard(self):
        dt = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if dt < self.warmup_sec:
            return
        if self.arm_on_start and not self.armed:
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=float(VehicleCommand.ARMING_ACTION_ARM))
            self.get_logger().info('Sent ARM command')
            self.armed = True
        if self.armed and not self.offboard:
            # VEHICLE_CMD_DO_SET_MODE: custom(1), OFFBOARD(6)
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
            self.get_logger().info('Requested OFFBOARD mode')
            self.offboard = True

    def tick(self):
        # Always stream OffboardControlMode
        self.publish_offboard_mode()

        if self.have_state:
            # Position errors (ENU)
            ex = self.goal_x - self.x
            ey = self.goal_y - self.y
            ez = self.goal_z - self.z

            # dt for D/I terms
            now = self.get_clock().now()
            dt = None
            if self.prev_time is not None:
                dt = (now - self.prev_time).nanoseconds / 1e9
            self.prev_time = now

            # Derivatives
            if dt is None or dt <= 1e-6:
                dex = dey = dez = 0.0
                de_yaw = 0.0
            else:
                dex = (ex - self.prev_ex) / dt
                dey = (ey - self.prev_ey) / dt
                dez = (ez - self.prev_ez) / dt

            # Integrals
            if dt is not None and dt > 1e-6:
                self.sum_ex += ex * dt
                self.sum_ey += ey * dt
                self.sum_ez += ez * dt

            # P(I)D control to velocity (ENU)
            vx_e = self.kp_xy * ex + self.ki_xy * self.sum_ex + self.kd_xy * dex
            vy_e = self.kp_xy * ey + self.ki_xy * self.sum_ey + self.kd_xy * dey
            vz_e = self.kp_z * ez + self.ki_z * self.sum_ez + self.kd_z * dez

            # XY magnitude clamp
            vxy = math.hypot(vx_e, vy_e)
            if vxy > self.max_speed_xy:
                s = self.max_speed_xy / max(1e-6, vxy)
                vx_e *= s
                vy_e *= s
            # Z clamp (signed)
            vz_e = max(-self.max_speed_z, min(self.max_speed_z, vz_e))

            # Yaw error and speed (ENU)
            e_yaw = wrap_pi(self.goal_yaw - self.yaw)
            if dt is None or dt <= 1e-6:
                de_yaw = 0.0
            else:
                de_yaw = (e_yaw - self.prev_eyaw) / dt
            yawspeed_e = self.kp_yaw * e_yaw + self.ki_yaw * self.sum_eyaw + self.kd_yaw * de_yaw
            # yaw integral update
            if dt is not None and dt > 1e-6:
                self.sum_eyaw += e_yaw * dt
            # Saturation
            yawspeed_e = max(-self.max_yawspeed, min(self.max_yawspeed, yawspeed_e))

            # Optional: rotate ENU velocities from odom frame into map frame
            if self.apply_yaw_corr and self.ensure_yaw_correction():
                # Rotate velocity from odom ENU into map ENU using map->odom yaw
                c = math.cos(self.yaw_map_to_odom)
                s = math.sin(self.yaw_map_to_odom)
                vx_map = c * vx_e - s * vy_e
                vy_map = s * vx_e + c * vy_e
            else:
                vx_map, vy_map = vx_e, vy_e

            # Publish velocity+yawspeed setpoint (map ENU → NED)
            self.publish_velocity_setpoint(vx_map, vy_map, vz_e, yawspeed_e)

            # Publish goal pose and marker for RViz/PlotJuggler
            self.publish_goal_visualization()

            # Update prev errors
            self.prev_ex = ex
            self.prev_ey = ey
            self.prev_ez = ez
            self.prev_eyaw = e_yaw

        # Mode switch and arm after warm-up
        self.try_arm_and_offboard()

    def ensure_yaw_correction(self) -> bool:
        if self.have_yaw_corr:
            return True
        try:
            ts = self.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time())
            # Extract yaw from quaternion
            qx = ts.transform.rotation.x
            qy = ts.transform.rotation.y
            qz = ts.transform.rotation.z
            qw = ts.transform.rotation.w
            siny_cosp = 2.0 * (qw * qz + qx * qy)
            cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
            yaw_map_to_odom = math.atan2(siny_cosp, cosy_cosp)
            # Store map->odom yaw (use directly to rotate odom→map vectors)
            self.yaw_map_to_odom = yaw_map_to_odom
            self.have_yaw_corr = True
            self.get_logger().info(f'Using map→odom yaw for odom→map rotation: {self.yaw_map_to_odom:.3f} rad')
        except Exception as e:
            # Throttle via logger rate if needed; here simple warning
            self.get_logger().warn(f'Waiting for TF map→odom to compute yaw correction: {e}')
            return False
        return True

    def publish_goal_visualization(self):
        # PoseStamped in ENU (odom)
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = 'odom'
        ps.pose.position.x = float(self.goal_x)
        ps.pose.position.y = float(self.goal_y)
        ps.pose.position.z = float(self.goal_z)
        # yaw -> quaternion (ENU, about +Z)
        half = 0.5 * float(self.goal_yaw)
        ps.pose.orientation.x = 0.0
        ps.pose.orientation.y = 0.0
        ps.pose.orientation.z = math.sin(half)
        ps.pose.orientation.w = math.cos(half)
        self.pub_goal_pose.publish(ps)

        # Marker (arrow) at goal with yaw orientation
        m = Marker()
        m.header = ps.header
        m.ns = 'uav_control_goal'
        m.id = 0
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.pose = ps.pose
        # Arrow points along +X of the marker frame; use orientation to rotate
        m.scale.x = 0.6   # shaft length
        m.scale.y = 0.06  # shaft diameter
        m.scale.z = 0.06  # head diameter (unused for simple arrow in Foxy/Humble)
        m.color.r = 0.1
        m.color.g = 0.9
        m.color.b = 0.1
        m.color.a = 1.0
        self.pub_goal_marker.publish(m)


def main():
    rclpy.init()
    node = OffboardPidGoto()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
