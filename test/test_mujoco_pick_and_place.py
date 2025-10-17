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
Test script for pick and place operations using MuJoCo simulation with Cartesian motion control.

This script performs a pick and place sequence in a loop using:
- Cartesian motion controller for arm movement
- Gripper action adapter for gripper control

The script is designed to work with the agilex_piper_mujoco_cartesian_control.launch.py setup
where MuJoCo simulation runs on PC host and this script can run from any machine on the ROS2 network.

Usage:
1. Start MuJoCo simulation on PC host:
   ros2 launch agilex_piper_mujoco bringup_mujoco_cartesian_motion_controller.launch.py

2. Start companion nodes on target board:
   ros2 launch agilex_piper_arm_bringup agilex_piper_mujoco_cartesian_control.launch.py

3. Run this test script from any machine on the ROSS2 network:
   python3 test_mujoco_pick_and_place.py

Configuration:
All poses, gripper positions, and timing can be configured in the CONFIGURATION section below.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from control_msgs.action import ParallelGripperCommand
from control_msgs.msg import GripperCommand
import time
import math
from dataclasses import dataclass
from typing import List


@dataclass
class Pose:
    """Simple pose representation."""
    x: float
    y: float
    z: float
    rx: float  # Roll (rotation around x-axis)
    ry: float  # Pitch (rotation around y-axis)
    rz: float  # Yaw (rotation around z-axis)


@dataclass
class PickPlaceStep:
    """Single step in pick and place sequence."""
    name: str
    pose: Pose
    gripper_position: float  # Gripper opening width in meters
    duration: float  # Time to wait after reaching this step


# ============================================================================
# CONFIGURATION SECTION - Modify these values to customize the behavior
# ============================================================================

# Initial home position (executed once at start)
INITIAL_HOME_POSITION = PickPlaceStep(
    name="Initial Home Position",
    pose=Pose(x=0.3, y=0.0, z=0.10, rx=0.0, ry=math.pi, rz=0.0),
    gripper_position=0.06,  # Fully open
    duration=3.0
)

# Pick and place sequence configuration (executed in each loop)
PICK_PLACE_SEQUENCE = [
    # Step 1: Approach pick position
    PickPlaceStep(
        name="Approach Pick",
        pose=Pose(x=0.3, y=-0.21, z=0.10, rx=0.0, ry=math.pi, rz=math.pi/2),
        gripper_position=0.06,  # Keep open
        duration=5.0
    ),

    # Step 2: Pick position (lower to object)
    PickPlaceStep(
        name="Pick Position",
        pose=Pose(x=0.3, y=-0.21, z=0.006, rx=0.0, ry=math.pi, rz=math.pi/2),
        gripper_position=0.06,  # Keep open for picking
        duration=6.0
    ),

    # Step 3: Close gripper (pick object)
    PickPlaceStep(
        name="Grasp Object",
        pose=Pose(x=0.3, y=-0.215, z=0.01, rx=0.0, ry=math.pi, rz=math.pi/2),
        gripper_position=0.02,  # Close to grasp
        duration=2.0
    ),

    # Step 4: Lift object
    PickPlaceStep(
        name="Lift Object",
        pose=Pose(x=0.3, y=-0.21, z=0.10, rx=0.0, ry=math.pi, rz=math.pi/2),
        gripper_position=0.02,  # Keep grasped
        duration=5.0
    ),

    # Step 5: Move to place approach position
    PickPlaceStep(
        name="Approach Place",
        pose=Pose(x=0.3, y=0.15, z=0.10, rx=0.0, ry=math.pi, rz=math.pi/2),
        gripper_position=0.02,  # Keep grasped
        duration=5.0
    ),

    # Step 6: Release object
    PickPlaceStep(
        name="Release Object",
        pose=Pose(x=0.3, y=0.15, z=0.10, rx=0.0, ry=math.pi, rz=math.pi/2),
        gripper_position=0.06,  # Open to release
        duration=2.0
    ),
]

# Loop configuration
NUM_LOOPS = 10  # Number of pick and place cycles to perform
LOOP_PAUSE_DURATION = 0.0  # Pause between complete cycles (seconds)

# Timing configuration
GRIPPER_TIMEOUT = 5.0  # Timeout for gripper commands (seconds)

# ============================================================================
# END CONFIGURATION SECTION
# ============================================================================


class MuJoCoPickAndPlaceTester(Node):
    """Test node for pick and place operations in MuJoCo simulation."""

    def __init__(self):
        super().__init__('mujoco_pick_and_place_tester')

        # Publishers for Cartesian motion control
        self.pose_publisher = self.create_publisher(
            PoseStamped,
            '/agilex_piper_cartesian_motion_controller/target_frame',
            10
        )

        # Action client for gripper control
        self.gripper_action_client = ActionClient(
            self,
            ParallelGripperCommand,
            '/gripper_cmd'
        )

        # Publisher for simple gripper commands (alternative interface)
        self.gripper_topic_publisher = self.create_publisher(
            GripperCommand,
            '/gripper_command',
            10
        )

        self.get_logger().info('MuJoCo Pick and Place Tester initialized')
        self.get_logger().info(f'Configured for {NUM_LOOPS} pick and place cycles')
        self.get_logger().info(f'Sequence has {len(PICK_PLACE_SEQUENCE)} steps per cycle')

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles (roll, pitch, yaw) to quaternion (x, y, z, w)."""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return x, y, z, w

    def send_pose_command(self, pose: Pose) -> bool:
        """Send a pose command to the Cartesian motion controller."""
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'base_link'
        pose_msg.header.stamp = self.get_clock().now().to_msg()

        # Set position
        pose_msg.pose.position.x = pose.x
        pose_msg.pose.position.y = pose.y
        pose_msg.pose.position.z = pose.z

        # Convert Euler angles to quaternion
        qx, qy, qz, qw = self.euler_to_quaternion(pose.rx, pose.ry, pose.rz)
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw

        self.get_logger().info(
            f'Sending pose command: pos=({pose.x:.3f}, {pose.y:.3f}, {pose.z:.3f}), '
            f'euler=({pose.rx:.3f}, {pose.ry:.3f}, {pose.rz:.3f})'
        )

        self.pose_publisher.publish(pose_msg)
        return True

    def send_gripper_command_action(self, position: float) -> bool:
        """Send gripper command using action interface."""
        self.get_logger().info('Waiting for gripper action server...')
        if not self.gripper_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Gripper action server not available')
            return False

        goal_msg = ParallelGripperCommand.Goal()
        goal_msg.command.position = [position]
        goal_msg.command.effort = [10.0]  # Max effort

        self.get_logger().info(f'Sending gripper command (action): position={position:.4f}m')

        future = self.gripper_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=GRIPPER_TIMEOUT)

        if not future.done():
            self.get_logger().error('Gripper action goal timed out')
            return False

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Gripper action goal rejected')
            return False

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=GRIPPER_TIMEOUT)

        if not result_future.done():
            self.get_logger().error('Gripper action result timed out')
            return False

        result = result_future.result()
        success = result.result.reached_goal
        self.get_logger().info(f'Gripper action completed: reached_goal={success}')
        return success

    def send_gripper_command_topic(self, position: float) -> bool:
        """Send gripper command using topic interface."""
        gripper_msg = GripperCommand()
        gripper_msg.position = position
        gripper_msg.max_effort = 10.0

        self.get_logger().info(f'Sending gripper command (topic): position={position:.4f}m')
        self.gripper_topic_publisher.publish(gripper_msg)
        return True

    def send_pose_command_incremental(self, start_pose: Pose, end_pose: Pose, step_size: float, axis: str, duration_per_step: float = 0.1) -> bool:
        """Send pose commands incrementally along a specified axis."""
        self.get_logger().info(f'Moving incrementally along {axis} axis with step size {step_size:.3f}m')

        # Determine start and end values for the specified axis
        if axis == 'x':
            start_val, end_val = start_pose.x, end_pose.x
        elif axis == 'y':
            start_val, end_val = start_pose.y, end_pose.y
        elif axis == 'z':
            start_val, end_val = start_pose.z, end_pose.z
        else:
            self.get_logger().error(f'Invalid axis: {axis}. Must be x, y, or z')
            return False

        # Calculate number of steps
        distance = abs(end_val - start_val)
        num_steps = int(distance / step_size)
        if num_steps == 0:
            num_steps = 1

        actual_step_size = (end_val - start_val) / num_steps

        self.get_logger().info(f'Moving {distance:.3f}m in {num_steps} steps of {actual_step_size:.3f}m each')

        # Send incremental pose commands
        for i in range(num_steps + 1):
            # Create intermediate pose
            intermediate_pose = Pose(
                x=start_pose.x,
                y=start_pose.y,
                z=start_pose.z,
                rx=start_pose.rx,
                ry=start_pose.ry,
                rz=start_pose.rz
            )

            # Update the moving axis
            current_val = start_val + (actual_step_size * i)
            if axis == 'x':
                intermediate_pose.x = current_val
            elif axis == 'y':
                intermediate_pose.y = current_val
            elif axis == 'z':
                intermediate_pose.z = current_val

            # Send pose command
            if not self.send_pose_command(intermediate_pose):
                self.get_logger().error(f'Failed to send incremental pose command at step {i}')
                return False

            # Wait between steps
            time.sleep(duration_per_step)

        self.get_logger().info(f'Completed incremental movement along {axis} axis')
        return True

    def execute_step(self, step: PickPlaceStep) -> bool:
        """Execute a single pick and place step."""
        self.get_logger().info(f'--- Executing step: {step.name} ---')

        # Send pose command
        if not self.send_pose_command(step.pose):
            self.get_logger().error(f'Failed to send pose command for step: {step.name}')
            return False

        # Send gripper command (using action interface - more reliable)
        if not self.send_gripper_command_action(step.gripper_position):
            self.get_logger().warning(f'Gripper action failed, trying topic interface for step: {step.name}')
            # Fallback to topic interface
            self.send_gripper_command_topic(step.gripper_position)

        # Wait for step completion
        self.get_logger().info(f'Waiting {step.duration:.1f}s for step completion...')
        time.sleep(step.duration)

        self.get_logger().info(f'Step completed: {step.name}')
        return True

    def execute_step_incremental(self, step: PickPlaceStep, previous_pose: Pose, axis: str, step_size: float = 0.01) -> bool:
        """Execute a step with incremental movement along specified axis."""
        self.get_logger().info(f'--- Executing step with incremental movement: {step.name} ---')

        # Send gripper command first (if different from previous)
        if not self.send_gripper_command_action(step.gripper_position):
            self.get_logger().warning(f'Gripper action failed, trying topic interface for step: {step.name}')
            # Fallback to topic interface
            self.send_gripper_command_topic(step.gripper_position)

        # Send incremental pose commands
        if not self.send_pose_command_incremental(previous_pose, step.pose, step_size, axis, duration_per_step=0.1):
            self.get_logger().error(f'Failed to send incremental pose commands for step: {step.name}')
            return False

        # Final wait for step completion
        self.get_logger().info(f'Waiting {step.duration:.1f}s for step completion...')
        time.sleep(step.duration)

        self.get_logger().info(f'Step completed: {step.name}')
        return True

    def run_pick_and_place_cycle(self, cycle_number: int) -> bool:
        """Run a complete pick and place cycle."""
        self.get_logger().info(f'=== Starting Pick and Place Cycle {cycle_number} ===')

        previous_pose = None
        for i, step in enumerate(PICK_PLACE_SEQUENCE):
            self.get_logger().info(f'Cycle {cycle_number}, Step {i+1}/{len(PICK_PLACE_SEQUENCE)}')

            # Handle special incremental steps
            if step.name == "Lift Object" and previous_pose:
                # Step 4: Lift object - incremental movement in z direction
                if not self.execute_step_incremental(step, previous_pose, axis='z', step_size=0.005):
                    self.get_logger().error(f'Step failed: {step.name}')
                    return False
            elif step.name == "Approach Place" and previous_pose:
                # Step 5: Approach place - incremental movement in y direction
                if not self.execute_step_incremental(step, previous_pose, axis='y', step_size=0.01):
                    self.get_logger().error(f'Step failed: {step.name}')
                    return False
            else:
                # Normal execution for other steps
                if not self.execute_step(step):
                    self.get_logger().error(f'Step failed: {step.name}')
                    return False

            # Update previous pose for next iteration
            previous_pose = step.pose

        self.get_logger().info(f'=== Completed Pick and Place Cycle {cycle_number} ===')
        return True

    def run_test_sequence(self) -> bool:
        """Run the complete test sequence with multiple cycles."""
        self.get_logger().info('Starting MuJoCo Pick and Place Test Sequence')
        self.get_logger().info('Make sure MuJoCo simulation and companion nodes are running!')

        # Wait a moment for publishers to be ready
        time.sleep(1.0)

        # Execute initial home position once
        self.get_logger().info('=== Moving to Initial Home Position ===')
        if not self.execute_step(INITIAL_HOME_POSITION):
            self.get_logger().error('Failed to reach initial home position!')
            return False

        for cycle in range(1, NUM_LOOPS + 1):
            try:
                if not self.run_pick_and_place_cycle(cycle):
                    self.get_logger().error(f'Pick and place cycle {cycle} failed!')
                    return False

                # Pause between cycles (except after the last one)
                if cycle < NUM_LOOPS:
                    self.get_logger().info(f'Pausing {LOOP_PAUSE_DURATION:.1f}s before next cycle...')
                    time.sleep(LOOP_PAUSE_DURATION)

            except KeyboardInterrupt:
                self.get_logger().info('Test interrupted by user')
                return False
            except Exception as e:
                self.get_logger().error(f'Cycle {cycle} failed with exception: {e}')
                return False

        self.get_logger().info('=== ALL PICK AND PLACE CYCLES COMPLETED SUCCESSFULLY! ===')
        return True


def main(args=None):
    rclpy.init(args=args)

    tester = MuJoCoPickAndPlaceTester()

    try:
        # Run the test sequence
        success = tester.run_test_sequence()
        if success:
            tester.get_logger().info('Test completed successfully!')
        else:
            tester.get_logger().error('Test failed!')

    except KeyboardInterrupt:
        tester.get_logger().info('Test interrupted by user')
    except Exception as e:
        tester.get_logger().error(f'Test failed with exception: {e}')
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()