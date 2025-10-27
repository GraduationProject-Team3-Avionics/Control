from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    args = [
        DeclareLaunchArgument('rate_hz', default_value='50.0'),
        DeclareLaunchArgument('arm_on_start', default_value='true'),
        DeclareLaunchArgument('warmup_sec', default_value='0.5'),
        DeclareLaunchArgument('goal_x', default_value='0.0'),
        DeclareLaunchArgument('goal_y', default_value='0.0'),
        DeclareLaunchArgument('goal_z', default_value='2.0'),
        DeclareLaunchArgument('goal_yaw', default_value='0.0'),
        DeclareLaunchArgument('kp_xy', default_value='0.8'),
        DeclareLaunchArgument('kp_z', default_value='0.8'),
        DeclareLaunchArgument('kp_yaw', default_value='1.0'),
        DeclareLaunchArgument('max_speed_xy', default_value='1.0'),
        DeclareLaunchArgument('max_speed_z', default_value='0.8'),
        DeclareLaunchArgument('max_yawspeed', default_value='0.8'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
    ]

    default_params = os.path.join(get_package_share_directory('uav_control'), 'config', 'offboard_pid_goto.yaml')

    node = Node(
        package='uav_control',
        executable='offboard_pid_goto',
        name='offboard_pid_goto',
        output='screen',
        parameters=[default_params, {
            'rate_hz': LaunchConfiguration('rate_hz'),
            'arm_on_start': LaunchConfiguration('arm_on_start'),
            'warmup_sec': LaunchConfiguration('warmup_sec'),
            'goal_x': LaunchConfiguration('goal_x'),
            'goal_y': LaunchConfiguration('goal_y'),
            'goal_z': LaunchConfiguration('goal_z'),
            'goal_yaw': LaunchConfiguration('goal_yaw'),
            'kp_xy': LaunchConfiguration('kp_xy'),
            'kp_z': LaunchConfiguration('kp_z'),
            'kp_yaw': LaunchConfiguration('kp_yaw'),
            'max_speed_xy': LaunchConfiguration('max_speed_xy'),
            'max_speed_z': LaunchConfiguration('max_speed_z'),
            'max_yawspeed': LaunchConfiguration('max_yawspeed'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    return LaunchDescription(args + [node])
