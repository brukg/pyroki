#!/usr/bin/env python3
"""
Controller Interface Node for PyRoki.

This node provides an interface between PyRoki IK solver and ROS2 controllers.
It subscribes to target poses and publishes joint trajectories for controllers.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from pyroki import Robot
from robot_descriptions.loaders.yourdfpy import load_robot_description
try:
    import pyroki_snippets as pks
except ImportError:
    # Fallback if pyroki_snippets is not available
    pks = None
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from builtin_interfaces.msg import Duration


class PyRokiControllerInterface(Node):
    """Interface node between PyRoki and ROS2 controllers."""

    def __init__(self):
        super().__init__('pyroki_controller_interface')
        
        # Parameters
        self.declare_parameter('robot_description_package', 'panda_description')
        self.declare_parameter('target_link_name', 'panda_hand')
        self.declare_parameter('controller_name', 'joint_trajectory_controller')
        self.declare_parameter('update_rate', 50.0)  # Hz
        self.declare_parameter('trajectory_duration', 2.0)  # seconds
        
        # Get parameters
        robot_desc_pkg = self.get_parameter('robot_description_package').value
        self.target_link_name = self.get_parameter('target_link_name').value
        controller_name = self.get_parameter('controller_name').value
        update_rate = self.get_parameter('update_rate').value
        self.trajectory_duration = self.get_parameter('trajectory_duration').value
        
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
        self.target_pose = None
        self.last_solution = None
        
        # QoS profiles
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        
        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/pyroki/target_pose',
            self.target_pose_callback,
            qos_profile
        )
        
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.twist_sub = self.create_subscription(
            Twist,
            '/pyroki/target_twist',
            self.target_twist_callback,
            10
        )
        
        # Publishers
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            f'/{controller_name}/joint_trajectory',
            10
        )
        
        # Action clients
        self.trajectory_action_client = ActionClient(
            self,
            FollowJointTrajectory,
            f'/{controller_name}/follow_joint_trajectory'
        )
        
        # Timer for continuous control
        self.control_timer = self.create_timer(
            1.0 / update_rate,
            self.control_loop
        )
        
        self.get_logger().info('PyRoki Controller Interface started')

    def target_pose_callback(self, msg):
        """Handle target pose updates."""
        self.target_pose = msg
        self.get_logger().debug('Received new target pose')

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

    def target_twist_callback(self, msg):
        """Handle target twist (velocity) commands."""
        if self.current_joint_positions is None or self.target_pose is None:
            return
        
        # Convert twist to pose offset and update target
        # This is a simplified implementation - you might want more sophisticated velocity control
        dt = 0.02  # 50Hz
        
        # Create new target pose by applying twist
        new_pose = PoseStamped()
        new_pose.header = self.target_pose.header
        new_pose.header.stamp = self.get_clock().now().to_msg()
        
        new_pose.pose.position.x = self.target_pose.pose.position.x + msg.linear.x * dt
        new_pose.pose.position.y = self.target_pose.pose.position.y + msg.linear.y * dt
        new_pose.pose.position.z = self.target_pose.pose.position.z + msg.linear.z * dt
        
        # For simplicity, keep orientation the same for now
        new_pose.pose.orientation = self.target_pose.pose.orientation
        
        self.target_pose = new_pose

    def control_loop(self):
        """Main control loop."""
        if (self.target_pose is None or 
            self.current_joint_positions is None):
            return
        
        try:
            # Solve IK for current target
            target_position = np.array([
                self.target_pose.pose.position.x,
                self.target_pose.pose.position.y,
                self.target_pose.pose.position.z
            ])
            
            target_wxyz = np.array([
                self.target_pose.pose.orientation.w,
                self.target_pose.pose.orientation.x,
                self.target_pose.pose.orientation.y,
                self.target_pose.pose.orientation.z
            ])
            
            # Use current joint positions as initial guess
            if pks is None:
                self.get_logger().error('pyroki_snippets not available. Please install the examples.')
                return
                
            solution = pks.solve_ik(
                robot=self.robot,
                target_link_name=self.target_link_name,
                target_position=target_position,
                target_wxyz=target_wxyz
            )
            
            # Check if solution is significantly different
            if (self.last_solution is not None and 
                np.allclose(solution, self.last_solution, atol=0.01)):
                return  # No significant change
            
            self.last_solution = solution.copy()
            
            # Create and send trajectory
            self.send_trajectory(solution)
            
        except Exception as e:
            self.get_logger().error(f'Control loop error: {e}')

    def send_trajectory(self, target_positions):
        """Send joint trajectory to controller."""
        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.header.frame_id = 'base_link'
        trajectory.joint_names = self.joint_names
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = target_positions.tolist()
        point.velocities = [0.0] * len(self.joint_names)
        point.accelerations = [0.0] * len(self.joint_names)
        point.time_from_start = Duration(sec=int(self.trajectory_duration), 
                                       nanosec=int((self.trajectory_duration % 1) * 1e9))
        
        trajectory.points = [point]
        
        # Publish trajectory
        self.trajectory_pub.publish(trajectory)
        
        self.get_logger().debug('Sent joint trajectory to controller')

    def send_trajectory_action(self, target_positions):
        """Send trajectory using action interface."""
        if not self.trajectory_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Trajectory action server not available')
            return
        
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.header.stamp = self.get_clock().now().to_msg()
        goal_msg.trajectory.header.frame_id = 'base_link'
        goal_msg.trajectory.joint_names = self.joint_names
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = target_positions.tolist()
        point.velocities = [0.0] * len(self.joint_names)
        point.time_from_start = Duration(sec=int(self.trajectory_duration),
                                       nanosec=int((self.trajectory_duration % 1) * 1e9))
        
        goal_msg.trajectory.points = [point]
        
        # Send goal
        future = self.trajectory_action_client.send_goal_async(goal_msg)
        self.get_logger().debug('Sent trajectory action goal')


def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    
    try:
        node = PyRokiControllerInterface()
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