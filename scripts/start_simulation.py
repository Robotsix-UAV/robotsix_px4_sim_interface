#!/usr/bin/env python3

# SPDX-License-Identifier: Apache-2.0

# Author: Damien SIX (damien@robotsix.net)
# Copyright: Robotsix
# Date: 2025

import argparse
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from robotsix_px4_sim_interface.action import StartSimulation
from robotsix_px4_sim_interface.msg import ModelInfo


class StartSimulationClient(Node):
    def __init__(self):
        super().__init__("start_simulation_client")
        self._action_client = ActionClient(self, StartSimulation, "start_simulation")

    def send_goal(self, world, models=[]):
        goal_msg = StartSimulation.Goal()
        goal_msg.world = world
        goal_msg.models = models

        # Wait for action server
        self._action_client.wait_for_server()

        # Send goal
        self.get_logger().info(f"Starting simulation with world: {world}")

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
    parser = argparse.ArgumentParser(description="Start a Gazebo simulation")
    parser.add_argument(
        "--world",
        type=str,
        default="robotsix_px4_simulation/worlds/default.sdf",
        help="Path to world file (default: %(default)s)",
    )
    parser.add_argument(
        "--model",
        action="append",
        nargs=5,
        metavar=("MODEL_DIR", "MODEL_NAME", "PX4_PARAM", "X", "Y"),
        help="Add a model to the simulation: MODEL_DIR MODEL_NAME PX4_PARAM X Y",
    )

    args = parser.parse_args()

    rclpy.init()

    action_client = StartSimulationClient()

    # Process models if specified
    models = []

    # If no models were specified, use the default
    if not args.model:
        default_model = [
            "robotsix_px4_simulation/models/quad_gps",
            "uav0",
            "6661",
            "0.0",
            "0.0",
        ]
        model = ModelInfo()
        model.model_dir = default_model[0]
        model.model_name = default_model[1]
        model.px4_parameters = int(default_model[2])
        model.x = float(default_model[3])
        model.y = float(default_model[4])
        model.z = 0.0  # Default to ground level
        models.append(model)
    else:
        # Process user-specified models
        for model_info in args.model:
            model = ModelInfo()
            model.model_dir = model_info[0]
            model.model_name = model_info[1]
            model.px4_parameters = int(model_info[2])
            model.x = float(model_info[3])
            model.y = float(model_info[4])
            model.z = 0.0  # Default to ground level
            models.append(model)

    # Execute the action
    action_client.send_goal(args.world, models)

    action_client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
