#!/usr/bin/env python3
# *********************************************************************************************************************
# Copyright [2025] Renesas Electronics Corporation and/or its licensors. All Rights Reserved.
#
# The contents of this file (the "contents") are proprietary and confidential to Renesas Electronics Corporation
# and/or its licensors ("Renesas") and subject to statutory and contractual protections.
#
# Unless otherwise expressly agreed in writing between Renesas and you: 1) you may not use, copy, modify, distribute,
# display, or perform the contents; 2) you may not use any name or mark of Renesas for advertising or publicity
# purposes or in connection with your use of the contents; 3) RENESAS MAKES NO WARRANTY OR REPRESENTATIONS ABOUT THE
# SUITABILITY OF THE CONTENTS FOR ANY PURPOSE; THE CONTENTS ARE PROVIDED "AS IS" WITHOUT ANY EXPRESS OR IMPLIED
# WARRANTY, INCLUDING THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
# NON-INFRINGEMENT; AND 4) RENESAS SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, OR CONSEQUENTIAL DAMAGES,
# INCLUDING DAMAGES RESULTING FROM LOSS OF USE, DATA, OR PROJECTS, WHETHER IN AN ACTION OF CONTRACT OR TORT, ARISING
# OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THE CONTENTS. Third-party contents included in this file may
# be subject to different terms.
# *********************************************************************************************************************

"""
Launch file for Agilex Piper arm with native Cartesian control.

This launch file starts:
- ros2_control_node: Main controller manager for hardware interface
- robot_state_publisher: Publishes TF transforms from URDF
- joint_state_broadcaster: Publishes joint states from hardware
- gripper_controller: (Optional) Provides gripper action interface when include_gripper=true
- gpio_controller: Provides extended arm features (administrative control, pose feedback, status monitoring)
- pose_to_gpio_converter: Converts PoseStamped messages to GPIO controller commands
- foxglove_bridge: WebSocket bridge for Foxglove Studio visualization

Key differences from other launch files:
- Uses native Cartesian control (motion_mode=0) instead of ROS2 controllers
- No joint trajectory or Cartesian motion controllers - uses hardware's native Cartesian control
- Includes pose_to_gpio_converter for easy PoseStamped message interface
- Pose commands can be sent via standard geometry_msgs/PoseStamped messages
- Suitable for applications requiring hardware-level Cartesian control

Parameters:
  can_interface (string, default='can2'):
    CAN interface name for hardware communication (e.g., 'can0', 'can1', 'can2').
    Only relevant when use_mock_hardware=false.

  use_mock_hardware (bool, default='false'):
    Enable mock hardware simulation for testing without physical robot.
    Set to 'true' for safe testing and development.

  include_gripper (bool, default='true'):
    Include gripper controller and action adapter in the system.
    When enabled, provides both action (/gripper_cmd) and topic (/gripper_command) interfaces.

  speed (int, default='50'):
    Speed percentage for arm movement (1-100).
    Controls the maximum velocity and acceleration limits.

Usage:
  # For physical robot with CAN interface (with gripper):
  ros2 launch agilex_piper_arm_bringup agilex_piper_native_cartesian_control.launch.py
  ros2 launch agilex_piper_arm_bringup agilex_piper_native_cartesian_control.launch.py can_interface:=can1

  # For arm-only configuration (without gripper):
  ros2 launch agilex_piper_arm_bringup agilex_piper_native_cartesian_control.launch.py include_gripper:=false

  # For custom speed (25% for slow motion):
  ros2 launch agilex_piper_arm_bringup agilex_piper_native_cartesian_control.launch.py speed:=25

  Then connect Foxglove Studio to ws://<foxglove_bridge_ip>:8765

Test native Cartesian control commands:
  # Primary interface - always use this topic for target poses (works with or without gripper)
  # With gripper: Pose is interpreted as TCP pose and automatically projected to tool0
  # Without gripper: Pose is used directly as tool0 pose
  ros2 topic pub --once /agilex_piper_gpio_controller/target_pose geometry_msgs/msg/PoseStamped "
  {
    header: {frame_id: 'base_link'},
    pose: {
      position: {x: 0.2, y: 0.0, z: 0.2},
      orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}
    }
  }"

  # Or set target pose directly via GPIO controller
  ros2 topic pub --once /agilex_piper_gpio_controller/commands
    control_msgs/msg/DynamicInterfaceGroupValues
    "{interface_groups: ['arm_target_pose'], interface_values: [{interface_names: ['x', 'y', 'z', 'rx', 'ry', 'rz'], values: [0.2, 0.0, 0.2, 0.0, 3.14159, 0.0]}]}"

  # Change motion speed dynamically
  ros2 topic pub --once /agilex_piper_gpio_controller/commands
    control_msgs/msg/DynamicInterfaceGroupValues
    "{interface_groups: ['arm_motion_mode'], interface_values: [{interface_names: ['speed'], values: [75.0]}]}"

  # Monitor current pose and arm status
  ros2 topic echo /agilex_piper_gpio_controller/gpio_states

  # Enable/disable arm
  ros2 topic pub --once /agilex_piper_gpio_controller/commands
    control_msgs/msg/DynamicInterfaceGroupValues
    "{interface_groups: ['arm_admin'], interface_values: [{interface_names: ['enable_arm'], values: [1.0]}]}"

Test gripper commands (when include_gripper=true):
  # Use standard gripper action interface (position = total opening width):
  ros2 action send_goal /gripper_cmd control_msgs/action/ParallelGripperCommand "{command: {position: [0.05], effort: [10.0]}}"

  # Or use simple topic interface:
  ros2 topic pub /gripper_command control_msgs/msg/GripperCommand "{position: 0.05, max_effort: 10.0}"

Or use Foxglove Studio's native publisher panel for interactive control.

Observe the arm moving in Foxglove Studio visualization.

NOTE: This launch file requires physical hardware as native Cartesian control
      cannot be simulated with mock hardware. For testing, use other launch files
      with 'use_mock_hardware:=true' option.
"""

import os
from typing import List

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs) -> List[Node]:
    """Setup function to evaluate launch configurations at runtime."""
    # Get launch configurations
    can_interface_value = LaunchConfiguration('can_interface').perform(context)
    include_gripper_value = LaunchConfiguration('include_gripper').perform(context)
    speed_value = LaunchConfiguration('speed').perform(context)

    # Get package directories
    pkg_share = get_package_share_directory('agilex_piper_arm_bringup')

    # Robot description - choose based on gripper configuration
    if include_gripper_value.lower() == 'true':
        robot_description_xacro = os.path.join(
            pkg_share, 'urdf', 'agilex_piper_arm_wi_gripper.urdf.xacro'
        )
    else:
        robot_description_xacro = os.path.join(
            pkg_share, 'urdf', 'agilex_piper_arm.urdf.xacro'
        )

    # Process XACRO file with parameters (native Cartesian control uses hardware Cartesian mode)
    robot_description_raw = xacro.process_file(
        robot_description_xacro,
        mappings={
            'can_interface': can_interface_value,
            'use_mock_hardware': 'false',
            'motion_mode': '0',  # Cartesian mode for native hardware control
            'speed': speed_value
        }
    ).toxml()

    robot_description = {'robot_description': robot_description_raw}

    # Controller configurations
    controller_config = os.path.join(
        pkg_share, 'config', 'controller_manager.yaml'
    )

    gripper_config = os.path.join(
        pkg_share, 'config', 'agilex_piper_gripper_position_controller.yaml'
    )

    gpio_config = os.path.join(
        pkg_share, 'config', 'agilex_piper_gpio_controller.yaml'
    )

    # Foxglove bridge launch file
    foxglove_bridge_launch = os.path.join(
        get_package_share_directory('foxglove_bridge'),
        'launch',
        'foxglove_bridge_launch.xml'
    )

    # Nodes
    nodes: List[Node] = [
        # Controller manager - only loads basic controllers, no motion controllers
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='controller_manager',
            output='screen',
            parameters=[
                robot_description,
                controller_config,
                gripper_config if include_gripper_value.lower() == 'true' else {},
                gpio_config,
            ],
            remappings=[
                ('/controller_manager/robot_description', '/robot_description'),
            ],
        ),
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        ),
        # Joint state broadcaster (always start for state publishing)
        Node(
            package='controller_manager',
            executable='spawner',
            name='joint_state_broadcaster_spawner',
            output='screen',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        ),
        # GPIO controller for native Cartesian control interface
        Node(
            package='controller_manager',
            executable='spawner',
            name='gpio_controller_spawner',
            output='screen',
            arguments=[
                'agilex_piper_gpio_controller',
                '--controller-manager', '/controller_manager',
            ],
        ),
        # Pose link projector for TCP to tool0 transformation (when gripper is enabled)
        Node(
            package='agilex_piper_utils',
            executable='pose_link_projector',
            name='tcp_to_tool0_projector',
            output='screen',
            parameters=[
                {'base_frame': 'base_link'},
                {'source_link': 'tcp'},          # TCP link
                {'target_link': 'tool0'},        # Tool0 link
            ],
            remappings=[
                ('~/in/pose', '/agilex_piper_gpio_controller/target_pose'),
                ('~/out/pose', '/tool0_target_pose'),  # Feeds pose_to_gpio_converter
            ],
        ) if include_gripper_value.lower() == 'true' else None,
        # Pose to GPIO converter for easy Cartesian control
        Node(
            package='agilex_piper_utils',
            executable='pose_to_gpio_converter',
            name='pose_to_gpio_converter',
            output='screen',
            remappings=[
                ('target_pose', '/agilex_piper_gpio_controller/target_pose' if include_gripper_value.lower() != 'true' else '/tool0_target_pose'),
                ('gpio_commands', '/agilex_piper_gpio_controller/commands'),
            ],
        ),
        # Foxglove bridge for web-based visualization
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(foxglove_bridge_launch)
        ),
    ]

    # Filter out None values from conditional nodes
    nodes = [node for node in nodes if node is not None]

    # Add gripper controller if requested
    if include_gripper_value.lower() == 'true':
        # Add gripper controller and action adapter
        nodes.extend([
            Node(
                package='controller_manager',
                executable='spawner',
                name='gripper_controller_spawner',
                output='screen',
                arguments=[
                    'agilex_piper_gripper_position_controller',
                    '--controller-manager', '/controller_manager',
                ],
            ),
            Node(
                package='agilex_piper_utils',
                executable='gripper_action_adapter',
                name='gripper_action_adapter',
                output='screen',
                parameters=[{
                    'action_server_name': 'gripper_cmd',
                    'gripper_command_topic': 'gripper_command',
                    'position_controller_topic': '/agilex_piper_gripper_position_controller/commands',
                    'max_gripper_width': 0.07,
                }],
            )
        ])

    return nodes


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for Agilex Piper arm with native Cartesian control."""
    # Declare arguments
    can_interface_arg = DeclareLaunchArgument(
        'can_interface',
        default_value='can2',
        description='CAN interface for hardware communication'
    )

    include_gripper_arg = DeclareLaunchArgument(
        'include_gripper',
        default_value='true',
        description='Include gripper controller and interfaces (true/false)'
    )

    speed_arg = DeclareLaunchArgument(
        'speed',
        default_value='50',
        description='Speed percentage (1-100, default: 50)'
    )

    return LaunchDescription([
        can_interface_arg,
        include_gripper_arg,
        speed_arg,
        OpaqueFunction(function=launch_setup)
    ])