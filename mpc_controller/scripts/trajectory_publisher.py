#!/usr/bin/env python3
"""
Trajectory Publisher Node
웨이포인트 기반 참조 궤적을 등시간 리샘플링하여 발행

Topics:
    /mpc/reference_path (nav_msgs/Path) - 전체 경로 (RViz2 시각화용)
    /mpc/reference_state (geometry_msgs/PoseStamped) - 현재 목표 상태
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import numpy as np


class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        
        # ─────────── Parameters ───────────
        self.declare_parameter('dt', 0.05)  # 샘플링 시간 [s]
        self.declare_parameter('loop', True)  # 경로 반복 여부
        self.declare_parameter('publish_rate', 20.0)  # Hz
        
        self.dt = self.get_parameter('dt').value
        self.loop = self.get_parameter('loop').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # ─────────── Waypoints (MATLAB과 동일) ───────────
        # [x, y, z, time]
        self.waypoints = np.array([
            [0.0,  0.0,  0.0,  0.0],   # 시작점
            [0.0,  0.0,  2.0,  3.0],   # 상승 (3초)
            [4.0,  0.0,  3.0,  8.0],   # 앞으로 (5초)
            [4.0,  4.0,  4.0,  13.0],  # 옆으로 (5초)
            [0.0,  4.0,  3.0,  18.0],  # 뒤로 (5초)
            [0.0,  0.0,  2.0,  23.0],  # 원점 복귀 (5초)
            [0.0,  0.0,  0.0,  28.0],  # 착륙 (5초)
        ])
        
        # ─────────── 등시간 리샘플링 ───────────
        self.trajectory = self._resample_trajectory()
        self.trajectory_index = 0
        self.start_time = None
        
        # ─────────── QoS ───────────
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # ─────────── Publishers ───────────
        self.path_pub = self.create_publisher(
            Path, '/mpc/reference_path', qos_profile)
        self.state_pub = self.create_publisher(
            PoseStamped, '/mpc/reference_state', qos_profile)
        
        # ─────────── Timer ───────────
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        
        # ─────────── 초기 경로 발행 ───────────
        self._publish_full_path()
        
        self.get_logger().info(f'Trajectory Publisher 시작')
        self.get_logger().info(f'  - 웨이포인트: {len(self.waypoints)}개')
        self.get_logger().info(f'  - 리샘플링 포인트: {len(self.trajectory)}개')
        self.get_logger().info(f'  - 총 비행 시간: {self.waypoints[-1, 3]:.1f}s')
        self.get_logger().info(f'  - Loop: {self.loop}')
    
    def _resample_trajectory(self):
        """웨이포인트를 등시간 간격으로 리샘플링"""
        T_total = self.waypoints[-1, 3]
        N = int(T_total / self.dt) + 1
        t_samples = np.linspace(0, T_total, N)
        
        trajectory = []
        
        for t in t_samples:
            # 현재 시간이 어느 구간에 있는지 찾기
            for i in range(len(self.waypoints) - 1):
                t_start = self.waypoints[i, 3]
                t_end = self.waypoints[i + 1, 3]
                
                if t_start <= t <= t_end:
                    # 선형 보간
                    alpha = (t - t_start) / (t_end - t_start) if t_end > t_start else 0
                    
                    p_start = self.waypoints[i, :3]
                    p_end = self.waypoints[i + 1, :3]
                    p_ref = (1 - alpha) * p_start + alpha * p_end
                    
                    # 속도 계산 (일정 속도 가정)
                    v_ref = (p_end - p_start) / (t_end - t_start) if t_end > t_start else np.zeros(3)
                    
                    trajectory.append({
                        't': t,
                        'position': p_ref,
                        'velocity': v_ref
                    })
                    break
        
        return trajectory
    
    def _publish_full_path(self):
        """전체 경로를 Path 메시지로 발행 (RViz2 시각화용)"""
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'odom'
        
        for point in self.trajectory:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = point['position'][0]
            pose.pose.position.y = point['position'][1]
            pose.pose.position.z = point['position'][2]
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)
        self.get_logger().info('전체 경로 발행 완료')
    
    def timer_callback(self):
        """현재 목표 상태 발행"""
        if self.start_time is None:
            self.start_time = self.get_clock().now()
        
        # 현재 시간 계산
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        T_total = self.waypoints[-1, 3]
        
        if self.loop:
            elapsed = elapsed % T_total
        else:
            elapsed = min(elapsed, T_total)
        
        # 해당 시간의 인덱스 찾기
        idx = int(elapsed / self.dt)
        idx = min(idx, len(self.trajectory) - 1)
        
        point = self.trajectory[idx]
        
        # PoseStamped 메시지 생성
        state_msg = PoseStamped()
        state_msg.header.stamp = self.get_clock().now().to_msg()
        state_msg.header.frame_id = 'odom'
        state_msg.pose.position.x = point['position'][0]
        state_msg.pose.position.y = point['position'][1]
        state_msg.pose.position.z = point['position'][2]
        state_msg.pose.orientation.w = 1.0
        
        self.state_pub.publish(state_msg)
        
        # 주기적으로 전체 경로 재발행 (RViz2 연결 시)
        if idx == 0:
            self._publish_full_path()


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    
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
