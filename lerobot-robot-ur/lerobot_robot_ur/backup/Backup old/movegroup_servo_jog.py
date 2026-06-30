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

from control_msgs.msg import JointJog
from sensor_msgs.msg import JointState
from moveit_msgs.srv import ServoCommandType
import rclpy
from rclpy import qos
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
from std_srvs.srv import SetBool
from .config_ur import UrConfig
from typing import Any

logger = logging.getLogger(__name__)



class Movegroup2ServoJog:
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
        self._jog_pub = None

    def connect(self) -> None:
        self._jog_pub = self._node.create_publisher(
            JointJog,
            self.config.ros2_interface.servo_delta_joint_cmds,
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
        self._jog_msg = JointJog()
        self._enable_req = SetBool.Request(data=False)
        self._disable_req = SetBool.Request(data=True)
        self._jog_type_req = ServoCommandType.Request(command_type=ServoCommandType.Request.JOINT_JOG)
        self.previous_time = self._node.get_clock().now()

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
        cmd_result = self._cmd_type_srv.call(self._jog_type_req)
        if not cmd_result or not cmd_result.success:
            logger.error("Switch to JOINT_JOG command type failed.")
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

    def send_action(self, action: dict[str, Any], last_joint_state: dict[str, dict[str, float]] | None) -> dict[str, Any]:
        """ calcualte differences between current joint state and target joint state, then send as a jog command  """
        """ calculate velocities for each joint based on the difference and a gain factor, then send as a jog command  """
        print("Received action:", action)
        print("Current joint state:", last_joint_state)
        time = self._node.get_clock().now()
        dt = (time - self.previous_time).nanoseconds / 1e9
        if dt <= 0.0:
            logger.warning("Non-positive time difference between commands, skipping jog command.")
            return {}
        joint_positions = action.get("joint_positions")
        if joint_positions is None:
            raise ValueError("Action must contain 'joint_positions' key.")
        if not last_joint_state or "position" not in last_joint_state:
            logger.warning("Joint state not available yet, skipping jog command.")
            return {}

        current_positions = [
            float(last_joint_state["position"].get(joint_name, 0.0))
            for joint_name in self.config.ros2_interface.arm_joint_names
        ]
        target_positions = joint_positions
        displacements = [target - current for target, current in zip(target_positions, current_positions)]
        #print("Position displacements:", displacements)
        """ velocity = position_difference / dt """       
        velocities = [diff / dt for diff in displacements]
        if any(abs(vel) > self.config.ros2_interface.max_joint_jog_velocity for vel in velocities):
            #logger.warning("Calculated velocity exceeds max_joint_jog_velocity, scaling down.")
            scale = self.config.ros2_interface.max_joint_jog_velocity / max(abs(vel) for vel in velocities)
            velocities = [vel * scale for vel in velocities]
        #print("Calculated velocities:", velocities)             
        self.jog(displacements, velocities, dt, enable_if_disabled=True)

        self.previous_time = time

        return {}

    def jog(self, displacements, velocities, duration, enable_if_disabled=True):
        if enable_if_disabled and not self._enabled:
            self.enable()
        self._jog_msg.header.stamp = self._node.get_clock().now().to_msg()
        self._jog_msg.header.frame_id = self.config.ros2_interface.base_link    
        self._jog_msg.joint_names = list(self.config.ros2_interface.arm_joint_names)
        # rosidl conversion expects native Python floats for sequence fields.
        self._jog_msg.displacements = [float(value) for value in displacements]
        self._jog_msg.velocities = [float(value) for value in velocities]
        self._jog_msg.duration = duration 
        #print("Publishing JointJog message:", self._jog_msg)
        self._jog_pub.publish(self._jog_msg)

    def destroy(self) -> None:
        pub = getattr(self, "_jog_pub", None)
        if pub is None:
            return

        try:
            pub.destroy()
        except Exception as err:
            # Avoid teardown crashes if destruction was already requested.
            logger.debug(f"Ignoring publisher destroy error during shutdown: {err}")
        finally:
            self._jog_pub = None