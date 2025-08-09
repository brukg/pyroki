#!/usr/bin/env python3
"""Launch file for PyRoki IK Service."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for PyRoki IK service."""
    
    # Declare launch arguments
    service_name_arg = DeclareLaunchArgument(
        'service_name',
        default_value='/pyroki/get_position_ik',
        description='Name of the IK service'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )
    
    # Create IK service node
    # This node will read robot_description and robot_description_semantic 
    # from the parameter server (set by MoveIt)
    ik_service_node = Node(
        package='pyroki',
        executable='pyroki_ik_service',
        name='pyroki_ik_service',
        output='screen',
        parameters=[{
            'service_name': LaunchConfiguration('service_name'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        remappings=[
            ('/pyroki/joint_states', '/joint_states'),
        ]
    )
    
    return LaunchDescription([
        service_name_arg,
        use_sim_time_arg,
        LogInfo(msg='Starting PyRoki IK Service - will read robot_description from parameter server'),
        ik_service_node,
    ])