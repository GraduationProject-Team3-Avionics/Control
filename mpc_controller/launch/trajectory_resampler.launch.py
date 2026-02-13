"""
Trajectory Resampler Launch File
전역 경로 → 등시간격 reference trajectory 변환 노드 실행
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('mpc_controller')
    config_file = os.path.join(pkg_dir, 'config', 'resampler_params.yaml')

    return LaunchDescription([
        Node(
            package='mpc_controller',
            executable='trajectory_resampler_node.py',
            name='trajectory_resampler_node',
            output='screen',
            parameters=[config_file],
        ),
    ])
