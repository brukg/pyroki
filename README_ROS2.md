# PyRoki ROS2 Integration

This document describes how to use PyRoki as a ROS2 package with ROS2 controllers.

## Overview

PyRoki has been converted to a ROS2 package that provides:

- **IK Service**: Inverse kinematics service compatible with MoveIt
- **Controller Interface**: Real-time interface with ROS2 controllers
- **Trajectory Planner**: Advanced trajectory planning with collision avoidance

## Installation

### Prerequisites

- ROS2 (Humble or later)
- Python 3.10+
- JAX (with appropriate backend - CPU, GPU, or TPU)

### Build from Source

1. Create a ROS2 workspace:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. Clone the repository:
```bash
git clone https://github.com/chungmin99/pyroki.git
```

3. Install Python dependencies:
```bash
cd pyroki
pip install -e .[ros2]
```

4. Build the ROS2 package:
```bash
cd ~/ros2_ws
colcon build --packages-select pyroki
source install/setup.bash
```

## Usage

### Basic IK Service

Start the IK service for a Panda robot:

```bash
ros2 launch pyroki pyroki_ik_service.launch.py robot_description_package:=panda_description
```

Call the service:

```bash
ros2 service call /pyroki/get_position_ik moveit_msgs/srv/GetPositionIK "{
  ik_request: {
    group_name: 'panda_arm',
    pose_stamped: {
      header: {frame_id: 'base_link'},
      pose: {
        position: {x: 0.6, y: 0.0, z: 0.6},
        orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}
      }
    }
  }
}"
```

### Controller Interface

Start the controller interface to work with ros2_control:

```bash
ros2 launch pyroki pyroki_controller_interface.launch.py \
    robot_description_package:=panda_description \
    controller_name:=joint_trajectory_controller
```

Publish target poses:

```bash
ros2 topic pub /pyroki/target_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'base_link'},
  pose: {
    position: {x: 0.6, y: 0.1, z: 0.5},
    orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}
  }
}"
```

### Complete System

Launch the complete PyRoki system:

```bash
ros2 launch pyroki pyroki_complete.launch.py \
    robot_description_package:=panda_description \
    use_rviz:=true
```

### Trajectory Planning

Plan trajectories through multiple waypoints:

```bash
ros2 topic pub /pyroki/target_waypoints geometry_msgs/msg/PoseArray "{
  header: {frame_id: 'base_link'},
  poses: [
    {
      position: {x: 0.5, y: 0.0, z: 0.5},
      orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}
    },
    {
      position: {x: 0.6, y: 0.1, z: 0.6},
      orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}
    }
  ]
}"
```

## Configuration

### Parameters

Key parameters can be configured in `config/pyroki_params.yaml`:

- `robot_description_package`: Robot description package name
- `target_link_name`: End-effector link name  
- `controller_name`: ROS2 controller name
- `update_rate`: Control loop frequency (Hz)
- `collision_checking`: Enable collision avoidance

### Robot-Specific Configuration

For different robots, update the parameters:

**Panda Robot:**
```yaml
robot_description_package: "panda_description"
target_link_name: "panda_hand"
```

**UR5 Robot:**
```yaml
robot_description_package: "ur_description"  
target_link_name: "tool0"
```

## Integration with ros2_control

PyRoki works seamlessly with ros2_control. Ensure your robot has:

1. A joint trajectory controller:
```yaml
controller_manager:
  ros__parameters:
    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController
```

2. Joint state broadcaster:
```yaml
joint_state_broadcaster:
  type: joint_state_broadcaster/JointStateBroadcaster
```

## Topics and Services

### Services

- `/pyroki/get_position_ik` - IK service (moveit_msgs/srv/GetPositionIK)

### Topics

**Subscribed:**
- `/joint_states` - Current robot joint states
- `/pyroki/target_pose` - Target end-effector pose
- `/pyroki/target_twist` - Target end-effector velocity
- `/pyroki/target_waypoints` - Multiple waypoints for trajectory planning

**Published:**
- `/pyroki/joint_states` - Computed joint states
- `/pyroki/planned_trajectory` - Planned trajectories
- `/{controller_name}/joint_trajectory` - Commands to trajectory controller

## Advanced Features

### Collision Avoidance

Enable collision checking in the trajectory planner:

```bash
ros2 param set /pyroki_trajectory_planner collision_checking true
```

### Manipulability Optimization

PyRoki can optimize for manipulability during IK solving. This is automatically handled in the background.

### Real-time Performance

For real-time applications:
- Use JAX with XLA compilation
- Set appropriate update rates (typically 50-250 Hz)
- Pre-warm JAX functions during initialization

## Troubleshooting

### Common Issues

1. **"Robot description not found"**
   - Ensure the robot description package is installed
   - Check the `robot_description_package` parameter

2. **"IK solver failed"**
   - Check if target pose is reachable
   - Verify joint limits in URDF
   - Try different initial configurations

3. **"Controller not responding"**
   - Ensure ros2_control is running
   - Check controller namespace and topic names
   - Verify joint names match between PyRoki and controller

### Performance Optimization

- Use GPU acceleration with JAX if available
- Adjust update rates based on your hardware
- Pre-compile JAX functions for better performance

## Examples

See the `examples/` directory for complete usage examples with different robots and scenarios.

## Contributing

Contributions to the ROS2 integration are welcome! Please see the main PyRoki repository for contribution guidelines.