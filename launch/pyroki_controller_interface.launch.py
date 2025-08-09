#!/usr/bin/env python3
"""Launch file for PyRoki Controller Interface."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for PyRoki controller interface."""
    
    # Declare launch arguments
    robot_description_package_arg = DeclareLaunchArgument(
        'robot_description_package',
        default_value='panda_description',
        description='Name of the robot description package'
    )
    
    target_link_name_arg = DeclareLaunchArgument(
        'target_link_name',
        default_value='panda_hand',
        description='Name of the target link for IK'
    )
    
    controller_name_arg = DeclareLaunchArgument(
        'controller_name',
        default_value='joint_trajectory_controller',
        description='Name of the joint trajectory controller'
    )
    
    update_rate_arg = DeclareLaunchArgument(
        'update_rate',
        default_value='50.0',
        description='Control update rate in Hz'
    )
    
    trajectory_duration_arg = DeclareLaunchArgument(
        'trajectory_duration',
        default_value='2.0',
        description='Default trajectory duration in seconds'
    )
    
    # Create controller interface node
    controller_interface_node = Node(
        package='pyroki',
        executable='pyroki_controller_interface',
        name='pyroki_controller_interface',
        output='screen',
        parameters=[{
            'robot_description_package': LaunchConfiguration('robot_description_package'),
            'target_link_name': LaunchConfiguration('target_link_name'),
            'controller_name': LaunchConfiguration('controller_name'),
            'update_rate': LaunchConfiguration('update_rate'),
            'trajectory_duration': LaunchConfiguration('trajectory_duration'),
        }],
        remappings=[
            ('/joint_states', '/joint_states'),
        ]
    )
    
    return LaunchDescription([
        robot_description_package_arg,
        target_link_name_arg,
        controller_name_arg,
        update_rate_arg,
        trajectory_duration_arg,
        LogInfo(msg='Starting PyRoki Controller Interface'),
        controller_interface_node,
    ])