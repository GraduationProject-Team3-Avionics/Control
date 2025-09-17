from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Waypoint manager: map → odom goal/path conversion
        Node(
            package='uav_control_offboard',
            executable='waypoint_manager',
            name='waypoint_manager',
            output='screen',
            parameters=[
                {'frame_world': 'odom'},
                {'rate': 10.0},
                {'track_path': False},
                {'lookahead': 1.0},
            ]
        ),
    ])
