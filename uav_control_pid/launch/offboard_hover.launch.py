from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    rate = DeclareLaunchArgument('rate_hz', default_value='50.0')
    climb = DeclareLaunchArgument('climb_m', default_value='2.0')
    arm = DeclareLaunchArgument('arm_on_start', default_value='true')
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')

    # Default params file inside package share
    default_params = os.path.join(get_package_share_directory('uav_control_pid'), 'config', 'offboard_hover.yaml')

    n = Node(
        package='uav_control_pid',
        executable='offboard_hover',
        name='offboard_hover',
        output='screen',
        parameters=[default_params, {
            'rate_hz': LaunchConfiguration('rate_hz'),
            'climb_m': LaunchConfiguration('climb_m'),
            'arm_on_start': LaunchConfiguration('arm_on_start'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    return LaunchDescription([
        rate, climb, arm, use_sim_time, n
    ])
