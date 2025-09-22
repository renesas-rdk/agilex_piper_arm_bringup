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
Launch file for Agilex Piper arm with Cartesian motion control.

This launch file starts:
- ros2_control_node: Main controller manager for hardware interface
- robot_state_publisher: Publishes TF transforms from URDF
- joint_state_broadcaster: Publishes joint states from hardware
- cartesian_motion_controller: Provides Cartesian space motion control
- gripper_controller: (Optional) Provides gripper action interface when include_gripper=true
- gpio_controller: Provides extended arm features (administrative control, pose feedback, status monitoring)
- foxglove_bridge: WebSocket bridge for Foxglove Studio visualization

Usage:
  # For physical robot with CAN interface (with gripper):
  ros2 launch agilex_piper_arm_bringup agilex_piper_cartesian_motion_control.launch.py
  ros2 launch agilex_piper_arm_bringup agilex_piper_cartesian_motion_control.launch.py can_interface:=can1

  # For arm-only configuration (without gripper):
  ros2 launch agilex_piper_arm_bringup agilex_piper_cartesian_motion_control.launch.py include_gripper:=false

  # For SIMULATION/TESTING without physical robot (RECOMMENDED for testing):
  ros2 launch agilex_piper_arm_bringup agilex_piper_cartesian_motion_control.launch.py use_mock_hardware:=true

  Then connect Foxglove Studio to ws://<foxglove_bridge_ip>:8765

Test Cartesian motion commands in another terminal with:
  ros2 topic pub --once /agilex_piper_cartesian_motion_controller/target_frame geometry_msgs/msg/PoseStamped "{
    header: {frame_id: 'base_link'},
    pose: {
      position: {x: 0.2, y: 0.0, z: 0.2},
      orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}
    }
  }"

Test gripper commands (when include_gripper=true):
  # Use standard gripper action interface (position = total opening width):
  ros2 action send_goal /agilex_piper_gripper_action_controller/gripper_cmd \
    control_msgs/action/GripperCommand "{command: {position: 0.05, max_effort: 10.0}}"

Test extended arm features with GPIO controller:
  # Enable arm functionality
  ros2 topic pub --once /agilex_piper_gpio_controller/commands
    control_msgs/msg/DynamicInterfaceGroupValues
    "{interface_groups: ['arm_admin'], interface_values: [{interface_names: ['enable_arm'], values: [1.0]}]}"

  # Switch to built-in Cartesian mode for comparison
  ros2 topic pub --once /agilex_piper_gpio_controller/commands
    control_msgs/msg/DynamicInterfaceGroupValues
    "{interface_groups: ['arm_motion_mode'], interface_values: [{interface_names: ['mode'], values: [0.0]}]}"

  # Monitor pose feedback during Cartesian motion
  ros2 topic echo /agilex_piper_gpio_controller/gpio_states

Or you can publish from foxglove studio's built-in publisher panel.

Observe the arm moving in foxglove studio.

NOTE: Use 'use_mock_hardware:=true' for safe testing without physical hardware!
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
    use_mock_hardware_value = LaunchConfiguration('use_mock_hardware').perform(context)
    include_gripper_value = LaunchConfiguration('include_gripper').perform(context)

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

    # Process XACRO file with parameters
    robot_description_raw = xacro.process_file(
        robot_description_xacro,
        mappings={
            'can_interface': can_interface_value,
            'use_mock_hardware': use_mock_hardware_value
        }
    ).toxml()

    robot_description = {'robot_description': robot_description_raw}

    # Controller configurations
    controller_config = os.path.join(
        pkg_share, 'config',
        'controller_manager_mock.yaml' if use_mock_hardware_value.lower() == 'true'
        else 'controller_manager.yaml'
    )

    # Select cartesian motion controller config based on gripper (static YAMLs)
    cartesian_motion_config = os.path.join(
        pkg_share, 'config',
        'agilex_piper_cartesian_motion_controller_wi_gripper.yaml' if include_gripper_value.lower() == 'true'
        else 'agilex_piper_cartesian_motion_controller.yaml'
    )

    gripper_config = os.path.join(
        pkg_share, 'config', 'agilex_piper_gripper_action_controller.yaml'
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
        # Controller manager
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='controller_manager',
            output='screen',
            parameters=[
                robot_description,
                controller_config,
                cartesian_motion_config,
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
        # Joint state broadcaster (always start)
        Node(
            package='controller_manager',
            executable='spawner',
            name='joint_state_broadcaster_spawner',
            output='screen',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        ),
        # Cartesian motion controller spawner
        Node(
            package='controller_manager',
            executable='spawner',
            name='cartesian_motion_controller_spawner',
            output='screen',
            arguments=[
                'agilex_piper_cartesian_motion_controller',
                '--controller-manager', '/controller_manager',
            ],
        ),
        # Foxglove bridge for web-based visualization
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(foxglove_bridge_launch)
        ),
        # GPIO controller for extended arm features
        Node(
            package='controller_manager',
            executable='spawner',
            name='gpio_controller_spawner',
            output='screen',
            arguments=[
                'agilex_piper_gpio_controller',
                '--controller-manager', '/controller_manager',
            ],
        )
    ]

    # Add gripper controller if requested
    if include_gripper_value.lower() == 'true':
        nodes.append(
            Node(
                package='controller_manager',
                executable='spawner',
                name='gripper_controller_spawner',
                output='screen',
                arguments=[
                    'agilex_piper_gripper_action_controller',
                    '--controller-manager', '/controller_manager',
                ],
            )
        )

    return nodes


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for Agilex Piper arm with Cartesian motion control."""
    # Declare arguments
    can_interface_arg = DeclareLaunchArgument(
        'can_interface',
        default_value='can2',
        description='CAN interface for hardware communication'
    )

    use_mock_hardware_arg = DeclareLaunchArgument(
        'use_mock_hardware',
        default_value='false',
        description='Use mock hardware for testing (true/false)'
    )

    include_gripper_arg = DeclareLaunchArgument(
        'include_gripper',
        default_value='true',
        description='Include gripper controller and interfaces (true/false)'
    )

    return LaunchDescription([
        can_interface_arg,
        use_mock_hardware_arg,
        include_gripper_arg,
        OpaqueFunction(function=launch_setup)
    ])
