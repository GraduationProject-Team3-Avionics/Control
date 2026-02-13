#!/usr/bin/env python3
"""
Trajectory Resampler Node
전역 경로(공간 기반)를 등시간격 reference trajectory로 변환

역할:
    - /guidance/global_path (nav_msgs/Path) 구독
    - Arc-length 기반 등속 리샘플링 수행
    - /mpc/reference_trajectory (nav_msgs/Path) 발행
      각 PoseStamped.header.stamp 에 상대 시각 기입

MPC 연동:
    - dt = 0.05 s (20 Hz) — MATLAB 모델과 일치
    - 등속 프로파일 (제어기 튜닝 단계용)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time
import numpy as np
from scipy.interpolate import interp1d


class TrajectoryResamplerNode(Node):
    def __init__(self):
        super().__init__('trajectory_resampler_node')

        # ─────────── Parameters ───────────
        self.declare_parameters(
            namespace='',
            parameters=[
                ('desired_velocity', 1.0),    # [m/s] 등속 비행 속도
                ('dt', 0.05),                 # [s]   리샘플링 간격 (MPC dt와 일치)
                ('interp_kind', 'cubic'),     # 보간 방식: 'linear' / 'cubic'
            ]
        )

        self.v_des = self.get_parameter('desired_velocity').value
        self.dt = self.get_parameter('dt').value
        self.interp_kind = self.get_parameter('interp_kind').value

        # ✅ (방법 1) 리샘플링 완료 로그를 1번만 찍기 위한 플래그
        self._printed_once = False

        # ─────────── Subscriber ───────────
        # Guidance 쪽이 TRANSIENT_LOCAL로 발행하므로 동일 QoS
        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        self.path_sub = self.create_subscription(
            Path, '/guidance/global_path', self._on_global_path, latched_qos
        )

        # ─────────── Publisher ───────────
        self.traj_pub = self.create_publisher(
            Path, '/mpc/reference_trajectory', latched_qos
        )

        self.get_logger().info(
            f'Trajectory Resampler 시작  '
            f'v_des={self.v_des} m/s, dt={self.dt} s, interp={self.interp_kind}'
        )

    # ──────────────────────────────────────────────
    # Callback
    # ──────────────────────────────────────────────
    def _on_global_path(self, msg: Path):
        """전역 경로 수신 → 등시간격 리샘플링 → 발행"""
        poses = msg.poses
        n = len(poses)
        if n < 2:
            self.get_logger().warn('경로 포인트가 2개 미만, 리샘플링 스킵')
            return

        # 1) 포인트 배열 추출
        pts = np.array([
            [p.pose.position.x, p.pose.position.y, p.pose.position.z]
            for p in poses
        ])  # (N, 3)

        # 2) 누적 호장(arc length) 계산
        diffs = np.diff(pts, axis=0)                  # (N-1, 3)
        seg_lengths = np.linalg.norm(diffs, axis=1)    # (N-1,)
        s = np.zeros(n)
        s[1:] = np.cumsum(seg_lengths)
        total_length = s[-1]

        if total_length < 1e-6:
            self.get_logger().warn('경로 총 길이가 0에 가까움, 리샘플링 스킵')
            return

        # 3) 등시간격 호장 위치 계산 (등속 프로파일)
        if self.v_des <= 1e-6:
            self.get_logger().warn('desired_velocity가 0에 가까움, 리샘플링 스킵')
            return

        total_time = total_length / self.v_des
        num_samples = int(total_time / self.dt) + 1
        t_uniform = np.linspace(0.0, total_time, num_samples)
        s_uniform = self.v_des * t_uniform

        # 마지막 값이 total_length를 살짝 넘을 수 있으므로 clamp
        s_uniform = np.clip(s_uniform, 0.0, total_length)

        # 4) 보간: s → (x, y, z)
        # cubic 보간은 점이 충분히 있어야 안정적이므로, 부족하면 linear로 폴백
        kind = self.interp_kind
        if kind == 'cubic' and n < 4:
            kind = 'linear'

        interp_x = interp1d(s, pts[:, 0], kind=kind)
        interp_y = interp1d(s, pts[:, 1], kind=kind)
        interp_z = interp1d(s, pts[:, 2], kind=kind)

        x_ref = interp_x(s_uniform)
        y_ref = interp_y(s_uniform)
        z_ref = interp_z(s_uniform)

        # 5) Path 메시지 구성
        traj = Path()
        traj.header.frame_id = msg.header.frame_id  # odom
        traj.header.stamp = self.get_clock().now().to_msg()

        for i in range(num_samples):
            ps = PoseStamped()
            ps.header.frame_id = msg.header.frame_id

            # 상대 시각을 stamp에 기록 (MPC가 시간 정보 활용)
            t_sec = float(t_uniform[i])
            sec = int(t_sec)
            nsec = int((t_sec - sec) * 1e9)
            ps.header.stamp = Time(sec=sec, nanosec=nsec)

            ps.pose.position.x = float(x_ref[i])
            ps.pose.position.y = float(y_ref[i])
            ps.pose.position.z = float(z_ref[i])
            ps.pose.orientation.w = 1.0
            traj.poses.append(ps)

        self.traj_pub.publish(traj)

        # ✅ (방법 1) 리샘플링 완료 로그는 최초 1회만 출력
        if not self._printed_once:
            self.get_logger().info(
                f'리샘플링 완료: {n}pts → {num_samples}pts  '
                f'총 거리={total_length:.2f}m, 총 시간={total_time:.2f}s'
            )
            self._printed_once = True


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryResamplerNode()

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
