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


from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import ServoCommandType
from rclpy import qos
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
from std_srvs.srv import SetBool
from .config_ur import UrConfig
from typing import Any

logger = logging.getLogger(__name__)



class Movegroup2ServoPose:
    """
    Python interface for MoveIt2 Servo.
    """

    def __init__(
        self,
        node: "Node",
        config: UrConfig,
        callback_group: "CallbackGroup",
    ):
        logger.error("Not implemented yet: Movegroup2ServoPose. Please use Movegroup2ServoTwist or Movegroup2ServoJog instead.")
        self.config = config
        self._node = node
        self._enabled = False   
        self._callback_group = callback_group

    def connect(self) -> None:

        self._pose_pub = self._node.create_publisher(
            PoseStamped,
            self.config.ros2_interface.pose_cmds,
            qos.QoSProfile(
                durability=qos.QoSDurabilityPolicy.VOLATILE,
                reliability=qos.QoSReliabilityPolicy.RELIABLE,
                history=qos.QoSHistoryPolicy.KEEP_ALL,
            ),
            callback_group=self._callback_group,
        )
        self._pause_srv = self._node.create_client(
            SetBool, self.config.ros2_interface.servo_pause, callback_group=self._callback_group
        )
        self._cmd_type_srv = self._node.create_client(
            ServoCommandType, self.config.ros2_interface.servo_switch_command_type, callback_group=self._callback_group
        )
        self._pose_msg = PoseStamped()
        self._enable_req = SetBool.Request(data=False)
        self._disable_req = SetBool.Request(data=True)
        self._pose_type_req = ServoCommandType.Request(command_type=ServoCommandType.Request.POSE)

    def enable(self, wait_for_server_timeout_sec=1.0) -> bool:
        if not self._pause_srv.wait_for_service(timeout_sec=wait_for_server_timeout_sec):
            logger.warning("Pause service not available.")
            return False
        if not self._cmd_type_srv.wait_for_service(timeout_sec=wait_for_server_timeout_sec):
            logger.warning("Command type service not available.")
            return False
        result = self._pause_srv.call(self._enable_req)
        if not result or not result.success:
            logger.error(f"Enable failed: {getattr(result, 'message', '')}")
            self._enabled = False
            return False
        cmd_result = self._cmd_type_srv.call(self._pose_type_req)
        if not cmd_result or not cmd_result.success:
            logger.error("Switch to POSE command type failed.")
            self._enabled = False
            return False
        logger.info("MoveIt Servo enabled.")
        self._enabled = True
        return True

    def disable(self, wait_for_server_timeout_sec=1.0) -> bool:
        if not self._pause_srv.wait_for_service(timeout_sec=wait_for_server_timeout_sec):
            logger.warning("Pause service not available.")
            return False
        result = self._pause_srv.call(self._disable_req)
        self._enabled = not (result and result.success)
        return bool(result and result.success)

    def send_action(self, action: dict[str, Any], dummy) -> dict[str, Any]:
        linear = action.get("linear", (0.0, 0.0, 0.0))
        angular = action.get("angular", (0.0, 0.0, 0.0))
        self.pose(linear=linear, angular=angular)
        return {}

    def pose(self, linear=(0.0, 0.0, 0.0), angular=(0.0, 0.0, 0.0), enable_if_disabled=True):
        if not self._enabled and enable_if_disabled and not self.enable():
            logger.warning("Dropping servo command because MoveIt2 Servo is not enabled.")
            return

        self._pose_msg.header.frame_id = self.config.ros2_interface.base_link
        self._pose_msg.header.stamp = self._node.get_clock().now().to_msg()
        self._pose_msg.pose.position.x = float(linear[0])
        self._pose_msg.pose.position.y = float(linear[1])
        self._pose_msg.pose.position.z = float(linear[2])
        self._pose_msg.pose.orientation.x = float(angular[0])
        self._pose_msg.pose.orientation.y = float(angular[1])
        self._pose_msg.pose.orientation.z = float(angular[2])
        self._pose_pub.publish(self._pose_msg)

    def destroy(self) -> None:
        if self._pose_pub:
            self._pose_pub.destroy()
            self._pose_pub = None