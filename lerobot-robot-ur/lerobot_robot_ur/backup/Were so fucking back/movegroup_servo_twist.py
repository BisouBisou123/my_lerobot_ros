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


from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import ServoCommandType
from rclpy import qos
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
from std_srvs.srv import SetBool
from .config_ur import UrConfig
from typing import Any

logger = logging.getLogger(__name__)


class Movegroup2ServoTwist:
    """
    Python interface for MoveIt2 Servo.
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
        self._callback_group = callback_group
        self._twist_pub = None

    def connect(self) -> None:
        self._twist_pub = self._node.create_publisher(
            TwistStamped,
            self.config.ros2_interface.servo_delta_twist_cmds,
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
        self._twist_msg = TwistStamped()
        self._enable_req = SetBool.Request(data=False)
        self._disable_req = SetBool.Request(data=True)
        self._twist_type_req = ServoCommandType.Request(command_type=ServoCommandType.Request.TWIST)

    def enable(self, wait_for_server_timeout_sec=1.0) -> bool:
        if not self._pause_srv.wait_for_service(timeout_sec=wait_for_server_timeout_sec):
            logger.warning("Pause service not available.")
            return False
        if not self._cmd_type_srv.wait_for_service(timeout_sec=wait_for_server_timeout_sec):
            logger.warning("Command type service not available.")
            return False
        if 0:
            result = self._pause_srv.call(self._enable_req)
            if not result or not result.success:
                logger.error(f"Enable failed: {getattr(result, 'message', '')}")
                self._enabled = False
                return False
        cmd_result = self._cmd_type_srv.call(self._twist_type_req)
        if not cmd_result or not cmd_result.success:
            logger.error("Switch to TWIST command type failed.")
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

        linear = (
            float(action.get("linear_x.vel", 0.0)),
            float(action.get("linear_y.vel", 0.0)),
            float(action.get("linear_z.vel", 0.0)),
        )


        angular = (
            float(action.get("angular_x.vel", 0.0)),
            float(action.get("angular_y.vel", 0.0)),
            float(action.get("angular_z.vel", 0.0)),
        )

        #print(f"Received action: linear={linear}, angular={angular}")
        self.twist(linear=linear, angular=angular)
        return {}

    def twist(self, linear=(0.0, 0.0, 0.0), angular=(0.0, 0.0, 0.0), enable_if_disabled=True):
        if not self._enabled and enable_if_disabled and not self.enable():
            logger.warning("Dropping servo command because MoveIt2 Servo is not enabled.")
            return

        self._twist_msg.header.frame_id = self.config.ros2_interface.base_link
        self._twist_msg.header.stamp = self._node.get_clock().now().to_msg()
        self._twist_msg.twist.linear.x = float(linear[0])
        self._twist_msg.twist.linear.y = float(linear[1])
        self._twist_msg.twist.linear.z = float(linear[2])
        self._twist_msg.twist.angular.x = float(angular[0])
        self._twist_msg.twist.angular.y = float(angular[1])
        self._twist_msg.twist.angular.z = float(angular[2])
        self._twist_pub.publish(self._twist_msg)

    def destroy(self) -> None:
        pub = getattr(self, "_twist_pub", None)
        if pub is None:
            return

        try:
            pub.destroy()
        except Exception as err:
            # Avoid teardown crashes if destruction was already requested.
            logger.debug(f"Ignoring publisher destroy error during shutdown: {err}")
        finally:
            self._twist_pub = None