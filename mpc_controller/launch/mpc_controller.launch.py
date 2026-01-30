#!/usr/bin/env python3
"""
MPC Controller Launch File
Trajectory Publisher + MPC Controller 동시 실행
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
    
    # Config 파일 경로
    trajectory_config = os.path.join(pkg_dir, 'config', 'trajectory_params.yaml')
    mpc_config = os.path.join(pkg_dir, 'config', 'mpc_params.yaml')
    rviz_config = os.path.join(pkg_dir, 'config', 'mpc_viz.rviz')
    
    # Launch arguments
    use_rviz = LaunchConfiguration('use_rviz')
    use_trajectory = LaunchConfiguration('use_trajectory')
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz2 for visualization'
    )
    
    declare_use_trajectory = DeclareLaunchArgument(
        'use_trajectory',
        default_value='true',
        description='Launch Trajectory Publisher'
    )
    
    # Trajectory Publisher Node
    trajectory_publisher_node = Node(
        package='mpc_controller',
        executable='trajectory_publisher.py',
        name='trajectory_publisher',
        parameters=[trajectory_config],
        output='screen',
        condition=IfCondition(use_trajectory)
    )
    
    # MPC Controller Node
    mpc_controller_node = Node(
        package='mpc_controller',
        executable='mpc_controller_node.py',
        name='mpc_controller',
        parameters=[mpc_config],
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
        declare_use_trajectory,
        trajectory_publisher_node,
        mpc_controller_node,
        rviz_node,
    ])
