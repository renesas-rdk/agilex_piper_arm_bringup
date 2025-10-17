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
Launch file for Agilex Piper arm companion nodes for MuJoCo simulation.

This launch file is designed to run on the TARGET BOARD while the MuJoCo simulation
runs on the PC host. Both systems must be on the same ROS 2 network (using ROS_DOMAIN_ID
or DDS discovery). This launch file provides:

- gripper_action_adapter: Converts standard gripper action interface to position commands
- foxglove_bridge: WebSocket bridge for Foxglove Studio visualization

The gripper action adapter listens to the standard /gripper_cmd action interface
and publishes to /agilex_piper_gripper_position_controller/commands topic,
which is consumed by the gripper position controller running in the MuJoCo simulation on PC host.

Parameters:
  max_gripper_width (float, default='0.07'):
    Maximum gripper opening width in meters.
    This is used by the action adapter to validate gripper commands.

Usage:
  # On PC host - Start MuJoCo simulation with ros2_control:
  ros2 launch agilex_piper_mujoco bringup_mujoco_cartesian_motion_controller.launch.py
  # This includes:
  # - ros2_control_node with MuJoCo hardware interface
  # - robot_state_publisher
  # - joint_state_broadcaster
  # - cartesian_motion_controller
  # - gripper_position_controller

  # On target board - Start this companion launch file:
  ros2 launch agilex_piper_arm_bringup agilex_piper_mujoco_cartesian_control.launch.py

  # Connect Foxglove Studio to ws://<target_board_ip>:8765

Test Cartesian motion commands (from any machine on the ROS 2 network):
  # These commands are sent to the controllers running in MuJoCo simulation on PC host
  ros2 topic pub --once /agilex_piper_cartesian_motion_controller/target_frame geometry_msgs/msg/PoseStamped "{
    header: {frame_id: 'base_link'},
    pose: {
      position: {x: 0.2, y: 0.0, z: 0.2},
      orientation: {x: 0.7071, y: 0.7071, z: 0.0, w: 0.0}
    }
  }"

Test gripper commands (from any machine on the ROS 2 network):
  # Use standard gripper action interface (position = total opening width):
  ros2 action send_goal /gripper_cmd \
    control_msgs/action/ParallelGripperCommand "{command: {position: [0.05], effort: [10.0]}}"

  # Or use the topic interface:
  ros2 topic pub --once /gripper_command control_msgs/msg/GripperCommand "{position: 0.05, max_effort: 10.0}"

Run automated pick and place test in a new terminal from target board:
  # Use the pick and place test script from the install folder:
  python3 $(ros2 pkg prefix agilex_piper_arm_bringup)/share/agilex_piper_arm_bringup/test/test_mujoco_pick_and_place.py

  # This script performs:
  # - Multiple pick and place cycles with configurable poses
  # - Automatic gripper control with both action and topic interfaces

Observe the arm and gripper moving in MuJoCo (on PC host) and Foxglove Studio (connected to target board).

NOTE:
  - Ensure both PC host and target board are on the same ROS 2 network (same ROS_DOMAIN_ID)
  - Start MuJoCo simulation on PC host first before starting this launch file on target board
"""

import os
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs) -> List[Node]:
    """Setup function to evaluate launch configurations at runtime."""
    # Get launch configurations
    max_gripper_width_value = LaunchConfiguration('max_gripper_width').perform(context)

    # Foxglove bridge launch file
    foxglove_bridge_launch = os.path.join(
        get_package_share_directory('foxglove_bridge'),
        'launch',
        'foxglove_bridge_launch.xml'
    )

    # Nodes
    nodes: List[Node] = [
        # Foxglove bridge for web-based visualization
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(foxglove_bridge_launch)
        ),
        # Gripper action adapter
        Node(
            package='agilex_piper_utils',
            executable='gripper_action_adapter',
            name='gripper_action_adapter',
            output='screen',
            parameters=[{
                'action_server_name': 'gripper_cmd',
                'gripper_command_topic': 'gripper_command',
                'position_controller_topic': '/agilex_piper_gripper_position_controller/commands',
                'max_gripper_width': float(max_gripper_width_value),
            }],
        ),
    ]

    return nodes


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for Agilex Piper MuJoCo Cartesian control companion nodes."""
    # Declare arguments
    max_gripper_width_arg = DeclareLaunchArgument(
        'max_gripper_width',
        default_value='0.07',
        description='Maximum gripper opening width in meters (default: 0.07)'
    )

    return LaunchDescription([
        max_gripper_width_arg,
        OpaqueFunction(function=launch_setup)
    ])
