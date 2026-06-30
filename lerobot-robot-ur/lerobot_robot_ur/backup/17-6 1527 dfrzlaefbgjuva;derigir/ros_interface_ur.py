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
import threading
import time

import rclpy
from typing import Any
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import Executor, SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from control_msgs.action import GripperCommand
from .config_ur import UrConfig, ActionType
from lerobot.utils.errors import DeviceNotConnectedError, DeviceAlreadyConnectedError
from .movegroup_servo_twist import Movegroup2ServoTwist
from .movegroup_servo_jog import Movegroup2ServoJog
from .movegroup_servo_pose import Movegroup2ServoPose
from .movegroup_follow_joint_trajectory import Movegroup2FollowJointTrajectory
from .native_joint_position_control import NativeJointPositionControl


logger = logging.getLogger(__name__)


class ROS2Interface:
    """Class to interface with a MoveIt2 manipulator.

    This class supports both JointGroupPositionController and JointTrajectoryController
    from ros2_control for arm control, depending on the configuration:

    - ActionType.JOINT_POSITION:
      Uses JointGroupPositionController.
      Publishes Float64MultiArray messages to '/position_controller/commands'

    - ActionType.MOVEGROUP_FOLLOW_JOINT_TRAJECTION:
      Uses JointTrajectoryController.
      Publishes JointTrajectory messages to controller service via FollowJointTrajectory action.

    - ActionType.MOVEGROUP_SERVO_JOG:
        Uses MoveIt Servo for real-time joint jogging.
        Publishes control_msgs/msg/JointJog messages to the appropriate topic for MoveIt Servo.

    The gripper control also supports both trajectory and action-based control
    via the gripper_action_type configuration option.
    """

    def __init__(self, config: UrConfig):
        self.config = config
        self.robot_node: Node | None = None
        self._callback_group = ReentrantCallbackGroup()
        self.joint_state_sub = None
        self.executor: Executor | None = None
        self.executor_thread: threading.Thread | None = None
        self.is_connected = False
        # Cache latest arm joint data keyed by configured joint names.
        self._last_joint_state: dict[str, dict[str, float]] | None = {"position": {}, "velocity": {}}
        self.ros_control = None

    def _create_ros_control(self) -> None:
        if self.robot_node is None:
            raise DeviceNotConnectedError("ROS2 node is not initialized. Call connect() before creating controllers.")

        if self.config.ros2_interface.action_type == ActionType.MOVEGROUP_SERVO_POSE:
            self.ros_control = Movegroup2ServoPose(
                node=self.robot_node,
                config=self.config,
                callback_group=self._callback_group,
            )
        elif self.config.ros2_interface.action_type == ActionType.MOVEGROUP_SERVO_TWIST:
            self.ros_control = Movegroup2ServoTwist(
                node=self.robot_node,
                config=self.config,
                callback_group=self._callback_group,
            )
        elif self.config.ros2_interface.action_type == ActionType.MOVEGROUP_SERVO_JOG:
            self.ros_control = Movegroup2ServoJog(
                node=self.robot_node,
                config=self.config,
                callback_group=self._callback_group,
            )
        elif self.config.ros2_interface.action_type == ActionType.MOVEGROUP_FOLLOW_JOINT_TRAJECTION:
            self.ros_control = Movegroup2FollowJointTrajectory(
                node=self.robot_node,
                config=self.config,
                callback_group=self._callback_group,
            )
        elif self.config.ros2_interface.action_type == ActionType.JOINT_POSITION:
            self.ros_control = NativeJointPositionControl(
                node=self.robot_node,
                config=self.config,
                callback_group=self._callback_group,
            )
        else:
            self.ros_control = None


    def connect(self) -> None:
## print toegevoegd
        print("ARM JOINT NAMES BEGIN:")
        print(self.config.ros2_interface.arm_joint_names)
        
        
        if not rclpy.ok():
            rclpy.init()

        self.robot_node = Node("ur_interface_node")

        # Register the node in a temporary executor before controller setup.
        temp_executor = SingleThreadedExecutor()
        temp_executor.add_node(self.robot_node)

        self._create_ros_control()
        if self.ros_control is None:
            raise ValueError(f"Unsupported action_type: {self.config.ros2_interface.action_type}")

        self.ros_control.connect()

        self.arm_joint_state_sub = self.robot_node.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_callback,
            10,
        )
# zelf toegevoegd
        self.gripper_joint_state_sub = self.robot_node.create_subscription(
            JointState,
            "/robotiq_gripper_controller/joint_states",
            self._gripper_state_callback,
            10,
        )     

        # Create and start the executor in a separate thread
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.robot_node)
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()
        time.sleep(3)  # Give some time to connect to services and receive messages

        self.ros_control.enable()  # Enable the controller after connection and initial setup
        self.is_connected = True

        print("ARM JOINT NAMES END:")
        print(self.config.ros2_interface.arm_joint_names)

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError("ROS2Interface is not connected. Call connect() before sending actions.")
        return self.ros_control.send_action(action, self._last_joint_state)

    @property
    def joint_state(self) -> dict[str, dict[str, float]] | None:
        """Get the last received joint state."""
        return self._last_joint_state

# #zelf aangemaakt
    def _gripper_state_callback(self, msg):
        self._last_joint_state = self._last_joint_state or {
            "position": {},
            "velocity": {},
        }

        if len(msg.position) > 0:
            self._last_joint_state["position"]["gripper_joint"] = msg.position[0]


    def _joint_state_callback(self, msg: "JointState") -> None:
        print(f"Received joint state: {msg}")
        self._last_joint_state = self._last_joint_state or {
            "position": {},
            "velocity": {},
        }
        
        name_to_index = {name: i for i, name in enumerate(msg.name)}
        
        for joint_name in self.config.ros2_interface.arm_joint_names:
            print("LOOKING FOR:", joint_name)
            idx = name_to_index.get(joint_name)
            print("IDX:", idx)     
            # if idx is None:
            #     raise ValueError(f"Joint '{joint_name}' not found in joint state.")
            if idx is None:
                print("NOT FOUND:", joint_name)
                continue
            
            self._last_joint_state["position"][joint_name] = msg.position[idx]
            self._last_joint_state["velocity"][joint_name] = 0.0
            
            # positions[joint_name] = msg.position[idx]
        # Keep velocity at zero for now; we currently read only positions.
            # velocities[joint_name] = 0.0

        # self._last_joint_state["position"] = positions
        # self._last_joint_state["velocity"] = velocities

        print(self._last_joint_state)
     
        #print(f"Updated joint state: {self._last_joint_state}")

    def disconnect(self):

        if self.arm_joint_state_sub:
            self.arm_joint_state_sub.destroy()
            self.arm_joint_state_sub = None

        if self.gripper_joint_state_sub:
            self.gripper_joint_state_sub.destroy()
            self.gripper_joint_state_sub = None

        if self.ros_control:
            self.ros_control.destroy()
            self.ros_control = None

        if self.robot_node:
            self.robot_node.destroy_node()
            self.robot_node = None

        if self.executor:
            self.executor.shutdown()
            self.executor = None
        if self.executor_thread:
            self.executor_thread.join()
            self.executor_thread = None

        self.is_connected = False
