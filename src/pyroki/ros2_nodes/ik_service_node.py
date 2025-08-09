#!/usr/bin/env python3
"""
IK Service Node for PyRoki.

This node provides an inverse kinematics service that can be used with ROS2 controllers.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotState
from std_msgs.msg import String
from pyroki import Robot
from robot_descriptions.loaders.yourdfpy import load_robot_description
import yourdfpy
import xml.etree.ElementTree as ET
try:
    import pyroki_snippets as pks
except ImportError:
    # Fallback if pyroki_snippets is not available
    pks = None
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs


class PyRokiIKService(Node):
    """ROS2 service node for inverse kinematics using PyRoki."""

    def __init__(self):
        super().__init__('pyroki_ik_service')
        
        # Parameters
        self.declare_parameter('service_name', '/pyroki/get_position_ik')
        service_name = self.get_parameter('service_name').value
        
        # Initialize state
        self.robot_description_content = None
        self.robot_description_semantic_content = None
        self.robot = None
        self.planning_groups = {}
        
        # Subscribe to robot description topics with latching QoS
        latching_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        
        self.robot_description_sub = self.create_subscription(
            String,
            '/robot_description',
            self.robot_description_callback,
            latching_qos
        )
        
        self.robot_description_semantic_sub = self.create_subscription(
            String,
            '/robot_description_semantic',
            self.robot_description_semantic_callback,
            latching_qos
        )
        
        self.get_logger().info('Waiting for robot_description on /robot_description topic...')
        

        
        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Services
        self.ik_service = self.create_service(
            GetPositionIK, 
            service_name, 
            self.solve_ik_callback
        )
        
        # Publishers
        self.joint_state_pub = self.create_publisher(
            JointState, 
            '/pyroki/joint_states', 
            10
        )
        
        self.get_logger().info(f'PyRoki IK Service started on {service_name}')
        
    def robot_description_callback(self, msg):
        """Handle robot_description topic updates."""
        self.robot_description_content = msg.data
        self.get_logger().info('Received robot_description from topic')
        self.initialize_robot()
        
    def robot_description_semantic_callback(self, msg):
        """Handle robot_description_semantic topic updates.""" 
        self.robot_description_semantic_content = msg.data
        self.get_logger().info('Received robot_description_semantic from topic')
        if self.robot is not None:
            self.parse_srdf(self.robot_description_semantic_content)
            
    def fallback_timer_callback(self):
        """Fallback to panda_description if no topics received."""
        if self.robot is None:
            self.get_logger().warn('No robot_description topic received, falling back to panda_description')
            try:
                urdf = load_robot_description('panda_description')
                self.robot = Robot.from_urdf(urdf)
                self.get_logger().info('Loaded fallback robot from panda_description')
            except Exception as e:
                self.get_logger().error(f'Failed to load fallback robot: {e}')
                
    def initialize_robot(self):
        """Initialize robot from received URDF content."""
        if not self.robot_description_content:
            return
            
        try:
            import tempfile
            import os
            import xml.etree.ElementTree as ET
            
            # Parse SRDF first to get planning groups
            if self.robot_description_semantic_content:
                self.parse_srdf(self.robot_description_semantic_content)
            
            # Try to load full robot first
            # Write URDF content to temporary file and load it
            with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as f:
                f.write(self.robot_description_content)
                temp_urdf_path = f.name
            
            try:
                urdf = yourdfpy.URDF.load(temp_urdf_path)
                self.robot = Robot.from_urdf(urdf)
                self.get_logger().info('Successfully loaded robot from topic')
                    
            except Exception as e:
                self.get_logger().warn(f'Failed to load full robot ({e}), trying with joint filtering...')
                
                # If full robot loading fails, try filtering out problematic joints
                try:
                    filtered_urdf_content = self.filter_problematic_joints(self.robot_description_content)
                    
                    with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as f2:
                        f2.write(filtered_urdf_content)
                        temp_filtered_path = f2.name
                    
                    try:
                        urdf = yourdfpy.URDF.load(temp_filtered_path)
                        self.robot = Robot.from_urdf(urdf)
                        self.get_logger().info('Successfully loaded robot with joint filtering')
                    finally:
                        os.unlink(temp_filtered_path)
                        
                except Exception as e2:
                    self.get_logger().error(f'Failed to load robot even with filtering: {e2}')
                    
            finally:
                os.unlink(temp_urdf_path)
                
        except Exception as e:
            self.get_logger().error(f'Failed to initialize robot: {e}')
            
    def filter_problematic_joints(self, urdf_content):
        """Remove continuous joints without velocity limits."""
        try:
            import xml.etree.ElementTree as ET
            root = ET.fromstring(urdf_content)
            
            # Find joints that are continuous but lack velocity limits
            joints_to_remove = []
            
            for joint in root.findall('.//joint[@type="continuous"]'):
                joint_name = joint.get('name', 'unknown')
                
                # Check if joint has velocity limits
                limit_elem = joint.find('limit')
                has_velocity_limit = (limit_elem is not None and 
                                    'velocity' in limit_elem.attrib)
                
                if not has_velocity_limit:
                    # Check if this joint is needed for any planning group
                    joint_needed = False
                    for group_name, group_info in self.planning_groups.items():
                        if joint_name in group_info['joints']:
                            joint_needed = True
                            break
                    
                    if not joint_needed:
                        joints_to_remove.append(joint_name)
                        self.get_logger().info(f'Removing problematic joint: {joint_name}')
            
            # Remove the problematic joints
            for joint_name in joints_to_remove:
                # Find and remove joint elements
                joints_to_delete = []
                
                # Find all joints with this name
                for joint in root.findall('.//joint'):
                    if joint.get('name') == joint_name:
                        joints_to_delete.append(joint)
                
                # Remove each joint element
                for joint_elem in joints_to_delete:
                    # Find parent and remove from it
                    for parent in root.iter():
                        if joint_elem in parent:
                            parent.remove(joint_elem)
                            break
            
            return ET.tostring(root, encoding='unicode')
            
        except Exception as e:
            self.get_logger().warn(f'Failed to filter joints: {e}, using original URDF')
            return urdf_content
        
    def parse_srdf(self, srdf_content):
        """Parse SRDF content to extract planning groups."""
        try:
            root = ET.fromstring(srdf_content)
            for group in root.findall('group'):
                group_name = group.get('name')
                joints = []
                chains = []
                links = []
                
                # Extract joints
                for joint in group.findall('joint'):
                    joints.append(joint.get('name'))
                
                # Extract chains  
                for chain in group.findall('chain'):
                    chains.append({
                        'base_link': chain.get('base_link'),
                        'tip_link': chain.get('tip_link')
                    })
                
                # Extract links
                for link in group.findall('link'):
                    links.append(link.get('name'))
                
                self.planning_groups[group_name] = {
                    'joints': joints,
                    'chains': chains,
                    'links': links
                }
                
                self.get_logger().info(f'Found planning group: {group_name}')
                
        except Exception as e:
            self.get_logger().error(f'Failed to parse SRDF: {e}')
    
    def get_group_tip_link(self, group_name):
        """Get the tip link for a planning group."""
        if group_name not in self.planning_groups:
            return None
            
        group = self.planning_groups[group_name]
        
        # If there are chains, use the tip of the first chain
        if group['chains']:
            return group['chains'][0]['tip_link']
        
        # If there are links, use the last one
        if group['links']:
            return group['links'][-1]
            
        return None

    def solve_ik_callback(self, request, response):
        """Handle IK service requests."""
        # Check if robot is initialized
        if self.robot is None:
            self.get_logger().error('Robot not initialized yet - waiting for robot_description topic')
            response.error_code.val = response.error_code.NO_IK_SOLUTION
            return response
            
        try:
            # Extract group name and target pose
            group_name = request.ik_request.group_name
            target_pose = request.ik_request.pose_stamped.pose
            target_position = np.array([
                target_pose.position.x,
                target_pose.position.y,
                target_pose.position.z
            ])
            target_wxyz = np.array([
                target_pose.orientation.w,
                target_pose.orientation.x,
                target_pose.orientation.y,
                target_pose.orientation.z
            ])
            
            # Determine target link from planning group
            if group_name and group_name in self.planning_groups:
                target_link_name = self.get_group_tip_link(group_name)
                if not target_link_name:
                    self.get_logger().error(f'Could not determine tip link for group {group_name}')
                    response.error_code.val = response.error_code.NO_IK_SOLUTION
                    return response
                self.get_logger().info(f'Using tip link {target_link_name} for group {group_name}')
            else:
                # Fall back to first available link if no group specified
                if hasattr(self, 'target_link_name'):
                    target_link_name = self.target_link_name
                else:
                    # Use last link as default
                    target_link_name = self.robot.links.names[-1]
                self.get_logger().warn(f'No valid group specified, using default link: {target_link_name}')
            
            # Get initial joint configuration if provided
            if request.ik_request.robot_state.joint_state.position:
                initial_cfg = np.array(request.ik_request.robot_state.joint_state.position)
            else:
                # Use default joint configuration
                initial_cfg = None
            
            # Solve IK
            if pks is None:
                self.get_logger().error('pyroki_snippets not available. Please install the examples.')
                response.error_code.val = response.error_code.NO_IK_SOLUTION
                return response
            
            # Get joints for the planning group
            if group_name and group_name in self.planning_groups:
                group_joints = self.planning_groups[group_name]['joints']
                self.get_logger().info(f'Solving IK for group {group_name} with joints: {group_joints}')
                
                # Create a sub-robot with only the planning group joints
                try:
                    # This is a workaround - PyRoki doesn't directly support joint subsets
                    # We'll solve for the full robot but only return group joints
                    solution = pks.solve_ik(
                        robot=self.robot,
                        target_link_name=target_link_name,
                        target_position=target_position,
                        target_wxyz=target_wxyz
                    )
                    
                    # Filter solution to only include group joints
                    all_joint_names = self.robot.joints.actuated_names
                    group_solution = []
                    group_joint_names = []
                    
                    for joint_name in group_joints:
                        if joint_name in all_joint_names:
                            joint_idx = all_joint_names.index(joint_name)
                            group_solution.append(solution[joint_idx])
                            group_joint_names.append(joint_name)
                    
                    solution = np.array(group_solution)
                    joint_names_for_response = group_joint_names
                    
                except Exception as e:
                    self.get_logger().error(f'Failed to solve IK for group {group_name}: {e}')
                    response.error_code.val = response.error_code.NO_IK_SOLUTION
                    return response
            else:
                # Solve for full robot if no valid group
                solution = pks.solve_ik(
                    robot=self.robot,
                    target_link_name=target_link_name,
                    target_position=target_position,
                    target_wxyz=target_wxyz
                )
                joint_names_for_response = self.robot.joints.actuated_names
            
            # Prepare response
            response.solution.joint_state.header.stamp = self.get_clock().now().to_msg()
            response.solution.joint_state.header.frame_id = 'base_link'
            
            # Use joint names from IK solution (filtered for planning group)
            response.solution.joint_state.name = joint_names_for_response
            response.solution.joint_state.position = solution.tolist()
            
            response.error_code.val = response.error_code.SUCCESS
            
            # Publish joint state
            self.publish_joint_state(solution, joint_names_for_response)
            
            self.get_logger().debug('IK solution found successfully')
            
        except Exception as e:
            self.get_logger().error(f'IK solving failed: {e}')
            response.error_code.val = response.error_code.NO_IK_SOLUTION
            
        return response
    
    def publish_joint_state(self, joint_positions, joint_names):
        """Publish joint state for visualization."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.name = joint_names
        msg.position = joint_positions.tolist()
        msg.velocity = [0.0] * len(joint_names)
        msg.effort = [0.0] * len(joint_names)
        
        self.joint_state_pub.publish(msg)


def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    
    try:
        node = PyRokiIKService()
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