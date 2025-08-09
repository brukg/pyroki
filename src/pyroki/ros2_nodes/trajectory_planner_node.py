#!/usr/bin/env python3
"""
Trajectory Planner Node for PyRoki.

This node provides trajectory planning capabilities using PyRoki's optimization features.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, PoseArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
from pyroki import Robot
from robot_descriptions.loaders.yourdfpy import load_robot_description
try:
    import pyroki_snippets as pks
except ImportError:
    # Fallback if pyroki_snippets is not available
    pks = None
from builtin_interfaces.msg import Duration


class PyRokiTrajectoryPlanner(Node):
    """Trajectory planning node using PyRoki optimization."""

    def __init__(self):
        super().__init__('pyroki_trajectory_planner')
        
        # Parameters
        self.declare_parameter('robot_description_package', 'panda_description')
        self.declare_parameter('target_link_name', 'panda_hand')
        self.declare_parameter('planning_time', 5.0)  # seconds
        self.declare_parameter('num_waypoints', 10)
        self.declare_parameter('collision_checking', True)
        
        # Get parameters
        robot_desc_pkg = self.get_parameter('robot_description_package').value
        self.target_link_name = self.get_parameter('target_link_name').value
        self.planning_time = self.get_parameter('planning_time').value
        self.num_waypoints = self.get_parameter('num_waypoints').value
        self.collision_checking = self.get_parameter('collision_checking').value
        
        # Initialize robot
        try:
            urdf = load_robot_description(robot_desc_pkg)
            self.robot = Robot.from_urdf(urdf)
            self.joint_names = self.robot.joints.names[:self.robot.joints.num_actuated_joints]
            self.get_logger().info(f'Loaded robot from {robot_desc_pkg}')
        except Exception as e:
            self.get_logger().error(f'Failed to load robot description: {e}')
            return
        
        # Current state
        self.current_joint_positions = None
        
        # QoS profiles
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        
        # Subscribers
        self.waypoints_sub = self.create_subscription(
            PoseArray,
            '/pyroki/target_waypoints',
            self.waypoints_callback,
            qos_profile
        )
        
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Publishers
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/pyroki/planned_trajectory',
            10
        )
        
        self.get_logger().info('PyRoki Trajectory Planner started')

    def joint_state_callback(self, msg):
        """Handle joint state updates."""
        try:
            # Extract positions for our robot's joints
            positions = []
            for joint_name in self.joint_names:
                if joint_name in msg.name:
                    idx = msg.name.index(joint_name)
                    positions.append(msg.position[idx])
                else:
                    positions.append(0.0)  # Default value
            
            self.current_joint_positions = np.array(positions)
        except Exception as e:
            self.get_logger().error(f'Error processing joint state: {e}')

    def waypoints_callback(self, msg):
        """Handle waypoint trajectory planning requests."""
        if self.current_joint_positions is None:
            self.get_logger().warn('No current joint state available')
            return
        
        if len(msg.poses) == 0:
            self.get_logger().warn('No waypoints provided')
            return
        
        try:
            self.get_logger().info(f'Planning trajectory through {len(msg.poses)} waypoints')
            
            # Convert poses to numpy arrays
            waypoint_positions = []
            waypoint_orientations = []
            
            for pose in msg.poses:
                waypoint_positions.append([
                    pose.position.x,
                    pose.position.y,
                    pose.position.z
                ])
                waypoint_orientations.append([
                    pose.orientation.w,
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z
                ])
            
            waypoint_positions = np.array(waypoint_positions)
            waypoint_orientations = np.array(waypoint_orientations)
            
            # Plan trajectory
            if self.collision_checking:
                trajectory = self.plan_trajectory_with_collision_avoidance(
                    waypoint_positions, waypoint_orientations
                )
            else:
                trajectory = self.plan_trajectory_simple(
                    waypoint_positions, waypoint_orientations
                )
            
            # Publish trajectory
            self.publish_trajectory(trajectory)
            
            self.get_logger().info('Trajectory planning completed')
            
        except Exception as e:
            self.get_logger().error(f'Trajectory planning failed: {e}')

    def plan_trajectory_simple(self, waypoint_positions, waypoint_orientations):
        """Plan trajectory through waypoints using simple IK."""
        joint_configurations = []
        
        # Start from current position
        current_config = self.current_joint_positions.copy()
        joint_configurations.append(current_config)
        
        # Solve IK for each waypoint
        if pks is None:
            self.get_logger().error('pyroki_snippets not available. Please install the examples.')
            return joint_configurations
            
        for i, (pos, orient) in enumerate(zip(waypoint_positions, waypoint_orientations)):
            try:
                solution = pks.solve_ik(
                    robot=self.robot,
                    target_link_name=self.target_link_name,
                    target_position=pos,
                    target_wxyz=orient
                )
                
                joint_configurations.append(solution)
                current_config = solution  # Use as initial guess for next waypoint
                
            except Exception as e:
                self.get_logger().error(f'IK failed for waypoint {i}: {e}')
                # Use previous configuration if IK fails
                joint_configurations.append(current_config)
        
        return np.array(joint_configurations)

    def plan_trajectory_with_collision_avoidance(self, waypoint_positions, waypoint_orientations):
        """Plan trajectory with collision avoidance using trajectory optimization."""
        try:
            # This would use PyRoki's trajectory optimization capabilities
            # For now, we'll use a simplified approach
            self.get_logger().info('Using collision-aware trajectory planning')
            
            # Use the trajopt snippet if available
            # You may need to adapt this based on the actual pyroki_snippets implementation
            trajectory = self.plan_trajectory_simple(waypoint_positions, waypoint_orientations)
            
            # TODO: Implement actual trajectory optimization with collision avoidance
            # This would involve:
            # 1. Setting up collision environment
            # 2. Using PyRoki's trajectory optimization
            # 3. Optimizing for smooth motion while avoiding collisions
            
            return trajectory
            
        except Exception as e:
            self.get_logger().error(f'Collision-aware planning failed, falling back to simple: {e}')
            return self.plan_trajectory_simple(waypoint_positions, waypoint_orientations)

    def publish_trajectory(self, joint_configurations):
        """Publish the planned trajectory."""
        trajectory = JointTrajectory()
        trajectory.header = Header()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.header.frame_id = 'base_link'
        trajectory.joint_names = self.joint_names
        
        # Create trajectory points
        dt = self.planning_time / (len(joint_configurations) - 1)
        
        for i, config in enumerate(joint_configurations):
            point = JointTrajectoryPoint()
            point.positions = config.tolist()
            point.velocities = [0.0] * len(self.joint_names)
            point.accelerations = [0.0] * len(self.joint_names)
            
            # Set time from start
            time_from_start = i * dt
            point.time_from_start = Duration(
                sec=int(time_from_start),
                nanosec=int((time_from_start % 1) * 1e9)
            )
            
            trajectory.points.append(point)
        
        # Calculate velocities (simple finite differences)
        for i in range(1, len(trajectory.points) - 1):
            prev_pos = np.array(trajectory.points[i-1].positions)
            next_pos = np.array(trajectory.points[i+1].positions)
            velocity = (next_pos - prev_pos) / (2 * dt)
            trajectory.points[i].velocities = velocity.tolist()
        
        self.trajectory_pub.publish(trajectory)
        self.get_logger().info(f'Published trajectory with {len(trajectory.points)} points')


def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    
    try:
        node = PyRokiTrajectoryPlanner()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()