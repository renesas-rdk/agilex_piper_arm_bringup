# agilex_piper_arm_bringup

ROS 2 package that provides launch files, controller configurations, robot descriptions, and test scripts for the AgileX Piper arm and gripper. This package contains everything needed to bring up and operate the robot.

## Features
- Launch files for different control modes (joint trajectory, joint position, Cartesian motion)
- Controller configurations for all supported control modes
- Complete robot URDF descriptions (arm-only and arm+gripper configurations)
- Test scripts for validating robot functionality
- Foxglove Studio configuration for visualization

## Related Packages
- **agilex_piper_ros2_control**: Contains the hardware interface implementation for CAN communication
- **agilex_piper_arm_description**: Contains the robot's visual and collision meshes, joint definitions

## Package layout
- `launch/`: Launch files for different control modes
- `config/`: Controller and controller_manager YAML configurations
- `urdf/`: Complete robot URDF descriptions
- `test/`: Example scripts for testing robot functionality

## Prerequisites
- ROS 2 (Jazzy or newer) with `ros2_control` and `ros2_controllers` ecosystem
- A colcon workspace (e.g., `~/ros2_ws`)
- `agilex_piper_ros2_control` package for hardware interface
- `agilex_piper_arm_description` package for robot description

## Launch Modes

### Joint Trajectory Control
Provides FollowJointTrajectory action interface for smooth trajectory execution:
```bash
ros2 launch agilex_piper_arm_bringup agilex_piper_joint_trajectory_control.launch.py
```

### Joint Position Control
Provides direct joint position command interface:
```bash
ros2 launch agilex_piper_arm_bringup agilex_piper_joint_position_control.launch.py
```

### Cartesian Motion Control
Provides Cartesian space motion control:
```bash
ros2 launch agilex_piper_arm_bringup agilex_piper_cartesian_motion_control.launch.py
```

## Launch Arguments
All launch files support the following arguments:
- `can_interface`: CAN interface for hardware communication (default: "can2")
- `use_mock_hardware`: Use mock hardware for testing (default: "false")
- `include_gripper`: Include gripper controller and interfaces (default: "true")

### Examples
```bash
# For physical robot with CAN interface can1
ros2 launch agilex_piper_arm_bringup agilex_piper_joint_trajectory_control.launch.py can_interface:=can1

# For arm-only configuration (without gripper)
ros2 launch agilex_piper_arm_bringup agilex_piper_joint_trajectory_control.launch.py include_gripper:=false

# For simulation/testing without physical robot
ros2 launch agilex_piper_arm_bringup agilex_piper_joint_trajectory_control.launch.py use_mock_hardware:=true
```

## Controllers
Controller configurations are provided in `config/`:
- `controller_manager.yaml`: Controller manager settings and available controllers
- `agilex_piper_joint_trajectory_controller.yaml`: Joint trajectory controller parameters
- `agilex_piper_joint_position_controller.yaml`: Joint position controller parameters
- `agilex_piper_gripper_action_controller.yaml`: Gripper action controller parameters
- `agilex_piper_cartesian_motion_controller.yaml`: Cartesian motion controller parameters
- `agilex_piper_motion_control_handle.yaml`: Motion control handle configuration

## Robot Descriptions
Complete robot URDF files in `urdf/`:
- `agilex_piper_arm.urdf.xacro`: Arm-only configuration
- `agilex_piper_arm_wi_gripper.urdf.xacro`: Arm with gripper configuration

## Testing
### Joint Trajectory Test
Run the included test script to validate joint trajectory control:
```bash
# Terminal 1: Launch trajectory controller
ros2 launch agilex_piper_arm_bringup agilex_piper_joint_trajectory_control.launch.py use_mock_hardware:=true

# Terminal 2: Run test script
python3 /home/ubuntu/ros2_ws/src/robots/agilex_piper_arm/agilex_piper_arm_bringup/test/test_joint_trajectory.py
```

### Manual Commands
Test different control modes manually:

**Joint Trajectory Control:**
```bash
ros2 action send_goal /agilex_piper_joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [joint1, joint2, joint3, joint4, joint5, joint6],
    points: [
      { positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: { sec: 2 } },
      { positions: [0.785, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: { sec: 4 } }
    ]
  }
}"
```

**Joint Position Control:**
```bash
ros2 topic pub --once /agilex_piper_joint_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.785, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

**Cartesian Motion Control:**
```bash
ros2 topic pub --once /agilex_piper_cartesian_motion_controller/target_frame geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'base_link'},
  pose: {
    position: {x: 0.2, y: 0.0, z: 0.2},
    orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}
  }
}"
```

**Gripper Control (when include_gripper=true):**
```bash
ros2 action send_goal /agilex_piper_gripper_action_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.05, max_effort: 10.0}}"
```

## Introspection
After launching, you can inspect the system:
```bash
ros2 control list_hardware_interfaces
ros2 control list_controllers
ros2 topic list
ros2 service list
```

## Foxglove Studio
An optional layout is available at `config/foxglove/arm_ros2_control.json`. Import it into Foxglove Studio to visualize:
- Joint states
- Controller feedback
- Robot model
- Control topics

Connect Foxglove Studio to `ws://localhost:8765` after launching any of the control modes.

## License and maintainers
Refer to `package.xml` for license and maintainer information.