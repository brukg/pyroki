#!/usr/bin/env python3
"""Complete launch file for PyRoki ROS2 integration."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate complete launch description for PyRoki."""
    
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
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz'
    )
    
    # Create all PyRoki nodes
    ik_service_node = Node(
        package='pyroki',
        executable='pyroki_ik_service',
        name='pyroki_ik_service',
        output='screen',
        parameters=[{
            'robot_description_package': LaunchConfiguration('robot_description_package'),
            'target_link_name': LaunchConfiguration('target_link_name'),
            'service_name': '/pyroki/get_position_ik',
        }]
    )
    
    controller_interface_node = Node(
        package='pyroki',
        executable='pyroki_controller_interface',
        name='pyroki_controller_interface',
        output='screen',
        parameters=[{
            'robot_description_package': LaunchConfiguration('robot_description_package'),
            'target_link_name': LaunchConfiguration('target_link_name'),
            'controller_name': LaunchConfiguration('controller_name'),
            'update_rate': 50.0,
            'trajectory_duration': 2.0,
        }]
    )
    
    trajectory_planner_node = Node(
        package='pyroki',
        executable='pyroki_trajectory_planner',
        name='pyroki_trajectory_planner',
        output='screen',
        parameters=[{
            'robot_description_package': LaunchConfiguration('robot_description_package'),
            'target_link_name': LaunchConfiguration('target_link_name'),
            'planning_time': 5.0,
            'num_waypoints': 10,
            'collision_checking': True,
        }]
    )
    
    # Robot state publisher (for visualization)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': LaunchConfiguration('robot_description_package'),
        }]
    )
    
    # Joint state publisher (for manual control)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )
    
    # RViz for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('pyroki'),
            'config',
            'pyroki_visualization.rviz'
        ])],
        condition=LaunchConfiguration('use_rviz')
    )
    
    return LaunchDescription([
        robot_description_package_arg,
        target_link_name_arg,
        controller_name_arg,
        use_rviz_arg,
        LogInfo(msg='Starting PyRoki Complete System'),
        ik_service_node,
        controller_interface_node,
        trajectory_planner_node,
        robot_state_publisher,
        joint_state_publisher,
        rviz_node,
    ])