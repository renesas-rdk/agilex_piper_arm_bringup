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
Example script for testing joint trajectory controller.
This script sends a simple joint trajectory to move the arm.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math


class JointTrajectoryTester(Node):
    """Simple test node for joint trajectory controller."""

    def __init__(self):
        super().__init__('joint_trajectory_tester')

        # Action client for joint trajectory controller
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/agilex_piper_joint_trajectory_controller/follow_joint_trajectory'
        )

        self.joint_names = [
            'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'
        ]

        self.get_logger().info('Joint trajectory tester initialized')

    def send_trajectory(self, positions, duration_sec=3.0):
        """Send a trajectory with given joint positions."""

        # Wait for action server
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        # Create trajectory message
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names

        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * len(self.joint_names)
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))

        goal_msg.trajectory.points = [point]

        self.get_logger().info(f'Sending trajectory to positions: {positions}')

        # Send goal
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return False

        self.get_logger().info('Goal accepted, waiting for result...')

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()
        self.get_logger().info(f'Result: {result.result.error_code}')

        return result.result.error_code == 0

    def run_test_sequence(self):
        """Run a sequence of test movements."""

        test_positions = [
            # Home position
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],

            # Move joint 1 (base rotation)
            [math.pi/4, 0.0, 0.0, 0.0, 0.0, 0.0],

            # Move joint 2 (shoulder)
            [math.pi/4, math.pi/6, 0.0, 0.0, 0.0, 0.0],

            # Move joint 3 (elbow)
            [math.pi/4, math.pi/6, -math.pi/6, 0.0, 0.0, 0.0],

            # Move multiple joints
            [0.0, math.pi/4, -math.pi/4, math.pi/6, 0.0, math.pi/4],

            # Return to home
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ]

        for i, positions in enumerate(test_positions):
            self.get_logger().info(f'--- Test movement {i+1}/{len(test_positions)} ---')

            success = self.send_trajectory(positions, duration_sec=2.0)
            if not success:
                self.get_logger().error(f'Movement {i+1} failed!')
                return False

            # Wait between movements
            self.get_logger().info('Waiting before next movement...')
            rclpy.spin_once(self, timeout_sec=1.0)

        self.get_logger().info('All test movements completed successfully!')
        return True


def main(args=None):
    rclpy.init(args=args)

    tester = JointTrajectoryTester()

    try:
        # Run the test sequence
        tester.run_test_sequence()

    except KeyboardInterrupt:
        tester.get_logger().info('Test interrupted by user')
    except Exception as e:
        tester.get_logger().error(f'Test failed with exception: {e}')
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
