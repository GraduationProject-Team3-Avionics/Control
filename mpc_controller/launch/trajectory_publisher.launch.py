#!/usr/bin/env python3
"""
Trajectory Publisher Launch File
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 패키지 경로
    pkg_dir = get_package_share_directory('mpc_controller')
    
    # Launch arguments
    use_rviz = LaunchConfiguration('use_rviz')
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz2 for visualization'
    )
    
    # Config 파일 경로
    config_file = os.path.join(pkg_dir, 'config', 'trajectory_params.yaml')
    rviz_config = os.path.join(pkg_dir, 'config', 'mpc_viz.rviz')
    
    # Trajectory Publisher Node
    trajectory_publisher_node = Node(
        package='mpc_controller',
        executable='trajectory_publisher.py',
        name='trajectory_publisher',
        parameters=[config_file],
        output='screen'
    )
    
    # RViz2 (use_rviz:=true 로 실행시에만 활성화)
    rviz_node = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config],
        output='screen',
        condition=IfCondition(use_rviz)
    )
    
    return LaunchDescription([
        declare_use_rviz,
        trajectory_publisher_node,
        rviz_node,
    ])
