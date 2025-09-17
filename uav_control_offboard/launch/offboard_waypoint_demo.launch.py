from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Placeholder launch to document intended nodes and params.
    # No executables yet — swap in when controllers are implemented.
    return LaunchDescription([
        # Example: Waypoint follower (PID outer loop) — TBD
        # Node(
        #     package='uav_control_offboard',
        #     executable='waypoint_follower',
        #     name='waypoint_follower',
        #     parameters=[
        #         {'frame_world': 'odom'},
        #         {'topic_state': '/uav/odom'},
        #         {'topic_setpoint_px4': '/fmu/in/trajectory_setpoint'},
        #         {'topic_offboard_mode': '/fmu/in/offboard_control_mode'},
        #         {'topic_vehicle_cmd': '/fmu/in/vehicle_command'},
        #         {'limits.v_xy': 3.0},
        #         {'limits.v_z': 1.0},
        #         {'limits.yaw_rate': 1.0},
        #         {'pid.x': [1.0, 0.2, 0.05]},
        #         {'pid.y': [1.0, 0.2, 0.05]},
        #         {'pid.z': [1.5, 0.3, 0.05]},
        #         {'pid.yaw': [1.5, 0.1, 0.0]},
        #     ]
        # ),

        # Example: PX4 Offboard adapter node — converts ENU (odom) setpoints to NED/FRD
        # Node(
        #     package='uav_control_offboard',
        #     executable='offboard_adapter',
        #     name='offboard_adapter',
        #     parameters=[{'mode': 'velocity'}]
        # ),
    ])

