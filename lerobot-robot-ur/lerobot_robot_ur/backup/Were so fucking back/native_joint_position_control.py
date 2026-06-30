# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import logging


from std_msgs.msg import Float64MultiArray
from lerobot.utils.errors import DeviceNotConnectedError, DeviceAlreadyConnectedError
from rclpy import qos
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
from .config_ur import UrConfig
from typing import Any

logger = logging.getLogger(__name__)


class NativeJointPositionControl:
    """
    Python interface for MoveIt2 FollowJointTrajectory.
    """

    def __init__(
        self,
        node: "Node",
        config: UrConfig,
        callback_group: "CallbackGroup",
    ):
        self.config = config
        self._node = node
        self._enabled = False
        self.pos_cmd_pub = None

    def connect(self) -> None:
        self.pos_cmd_pub = self._node.create_publisher(
            Float64MultiArray,
            self.config.ros2_interface.joint_position_controller_commands,
            10,
        )

    def enable(self, wait_for_server_timeout_sec=1.0) -> bool:
        return True

    def disable(self, wait_for_server_timeout_sec=1.0) -> bool:
        pass

    def send_action(self, action: dict[str, Any], dummy) -> dict[str, Any]:
        """
        Send an action command to the robot.
        Args:
            action (dict[str, Any]): The action command to send.
        Returns:
            dict[str, Any]: The response from the robot.
        """
        joint_positions = action.get("joint_positions")
        if joint_positions is None:
            raise ValueError("Action must contain 'joint_positions' key.")
        
        self.send_joint_position_command(joint_positions)
        return {}
    
    def send_joint_position_command(self, joint_positions: list[float], unnormalize: bool = True) -> None:
        """
        Send a command to the robot's joints.
        Args:
            joint_positions (list[float]): The target positions for the joints.
            unnormalize (bool): Whether to unnormalize the joint positions based on the robot's configuration.
        """
        if not self._node:
            raise DeviceNotConnectedError("ROS2Interface is not connected. You need to call `connect()`.")

        if unnormalize:
            min_joint_positions = self.config.ros2_interface.min_joint_positions
            max_joint_positions = self.config.ros2_interface.max_joint_positions
            if min_joint_positions is None or max_joint_positions is None:
                raise ValueError(
                    "Joint position normalization requires min and max joint positions to be set."
                )
            joint_positions = [
                min(max(pos, min_pos), max_pos)
                for pos, min_pos, max_pos in zip(
                    joint_positions,
                    min_joint_positions,
                    max_joint_positions,
                    strict=True,
                )
            ]

        # sent by publishig to topic
        arm_joint_names = self.config.ros2_interface.arm_joint_names
        if len(joint_positions) != len(arm_joint_names):
            raise ValueError(
                f"Expected {len(arm_joint_names)} joint positions, but got {len(joint_positions)}."
            )

        if self.pos_cmd_pub is None:
            raise DeviceNotConnectedError("Joint position controller publisher is not initialized.")
        msg = Float64MultiArray()
        msg.data = list(joint_positions)
        #print(f"Publishing joint positions: {joint_positions}")
        self.pos_cmd_pub.publish(msg)


    def destroy(self) -> None:
        if self.pos_cmd_pub is not None:
            self._node.destroy_publisher(self.pos_cmd_pub)
            self.pos_cmd_pub = None