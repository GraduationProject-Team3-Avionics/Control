from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('uav_control_pid'), 'config', 'offboard_pid_goto.yaml'
    )

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Parameter YAML for offboard_pid_goto'
    )

    node = Node(
        package='uav_control_pid',
        executable='offboard_pid_goto',
        name='offboard_pid_goto',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )

    return LaunchDescription([params_arg, node])
