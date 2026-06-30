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


from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.action import ActionClient
from rclpy import qos
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from lerobot.utils.errors import DeviceNotConnectedError, DeviceAlreadyConnectedError
from .config_ur import UrConfig
from typing import Any

logger = logging.getLogger(__name__)


class Movegroup2FollowJointTrajectory:
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

    def connect(self) -> None:
        controller_name = (
            self.config.ros2_interface.joint_trajectory_controller_sim
            if self.config.ros2_interface.sim
            else self.config.ros2_interface.joint_trajectory_controller
        )
        self._movegroup_follow_joint_trajection_client = ActionClient(
            self._node,
            FollowJointTrajectory,
            controller_name,
        )
        if not self._movegroup_follow_joint_trajection_client.wait_for_server(timeout_sec=5.0):
            raise ConnectionError("Joint trajectory action server is not available.") 
    pass

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


        # Create and send a FollowJointTrajectory action goal
        if self._movegroup_follow_joint_trajection_client is None:
            raise DeviceNotConnectedError("Joint trajectory action client is not initialized.")
        # Check if the previous action is busy, then abort the previous goal
        if 0:
            if self._current_goal_handle is not None:
                self._current_goal_handle.cancel_goal_async()
                self._current_goal_handle = None
        goal_msg = FollowJointTrajectory.Goal()
        arm_joint_names = self.config.ros2_interface.arm_joint_names
        if len(joint_positions) != len(arm_joint_names):
            raise ValueError(
                f"Expected {len(arm_joint_names)} joint positions, but got {len(joint_positions)}."
            )
        goal_msg.trajectory.joint_names = arm_joint_names
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start.sec = 1  # Set a default duration for the trajectory
        #point.time_from_start.nanosec = int(0.3 * 1e9)  # Set a default duration for the trajectory
        goal_msg.trajectory.points = [point]
        #print(f"Sending FollowJointTrajectory action goal: {goal_msg}")
        
        self.is_executing = True

        # Send the goal and store the future
        self._send_goal_future = self._movegroup_follow_joint_trajection_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        # When the goal response is received, store the goal handle
        def _store_goal_handle(future):
            goal_handle = future.result()
            self._current_goal_handle = goal_handle
            self.goal_response_callback(future)

        self._send_goal_future.add_done_callback(_store_goal_handle)
        return {}

    def goal_response_callback(self, future):
        """Handle goal response."""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                #print('Goal rejected by action server')
                #print('Possible reasons: controller not ready, invalid trajectory, or joints mismatch')
                #print('Check that the controller is running: ros2 control list_controllers')
                #self.is_executing = False
                return
            
            #print('Goal accepted')
        except Exception as e:
            print(f'Exception in goal response: {str(e)}')
            self.is_executing = False
            return
        
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handle result."""
        from control_msgs.action import FollowJointTrajectory as FJT
        result = future.result().result
        
        # Map error codes to messages
        error_messages = {
            FJT.Result.SUCCESSFUL: 'SUCCESSFUL',
            FJT.Result.INVALID_GOAL: 'INVALID_GOAL',
            FJT.Result.INVALID_JOINTS: 'INVALID_JOINTS',
            FJT.Result.OLD_HEADER_TIMESTAMP: 'OLD_HEADER_TIMESTAMP',
            FJT.Result.PATH_TOLERANCE_VIOLATED: 'PATH_TOLERANCE_VIOLATED',
            FJT.Result.GOAL_TOLERANCE_VIOLATED: 'GOAL_TOLERANCE_VIOLATED'
        }
        
        error_msg = error_messages.get(result.error_code, f'UNKNOWN({result.error_code})')
        
        if result.error_code == FJT.Result.SUCCESSFUL:
            #print(f'Result: {error_msg}')
            pass
        else:
            print(f'Result: {error_msg} (code: {result.error_code})')
        
        self.is_executing = False

    def feedback_callback(self, feedback_msg):
        """Handle feedback (optional)."""
        pass


    def destroy(self) -> None:
        if self._movegroup_follow_joint_trajection_client is not None:
            self._movegroup_follow_joint_trajection_client.destroy()
            self._movegroup_follow_joint_trajection_client = None
        self._current_goal_handle = None