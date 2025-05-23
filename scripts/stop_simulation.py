#!/usr/bin/env python3

# SPDX-License-Identifier: Apache-2.0

# Author: Damien SIX (damien@robotsix.net)
# Copyright: Robotsix
# Date: 2025


import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from robotsix_px4_sim_interface.action import StopSimulation


class StopSimulationClient(Node):
    def __init__(self):
        super().__init__("stop_simulation_client")
        self._action_client = ActionClient(self, StopSimulation, "stop_simulation")

    def send_goal(self):
        goal_msg = StopSimulation.Goal()

        # Wait for action server
        self._action_client.wait_for_server()

        # Send goal
        self.get_logger().info("Stopping simulation...")

        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected!")
            return

        self.get_logger().info("Goal accepted!")

        # Get result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        self.get_logger().info(f"Result: {result.message}")


def main():
    rclpy.init()

    action_client = StopSimulationClient()
    action_client.send_goal()

    action_client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
