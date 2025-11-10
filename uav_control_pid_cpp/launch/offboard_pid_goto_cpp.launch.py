from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('uav_control_pid_cpp')
    cfg = os.path.join(pkg_share, 'config', 'offboard_pid_goto.yaml')

    return LaunchDescription([
        Node(
            package='uav_control_pid_cpp',
            executable='offboard_pid_goto_cpp',
            name='offboard_pid_goto',
            output='screen',
            parameters=[cfg],
        ),
    ])

