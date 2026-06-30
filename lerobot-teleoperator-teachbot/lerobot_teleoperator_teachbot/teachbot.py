from typing import Any
from lerobot.teleoperators.teleoperator import Teleoperator
from .config_teachbot import TeachbotConfig
from teleop import Teleop as SpesTeleop
import numpy as np
import threading
from .ros_interface_teachbot import ROS2Interface


class Teachbot(Teleoperator):
    config_class = TeachbotConfig
    name = "teachbot"

    def __init__(self, config: TeachbotConfig):
        super().__init__(config)
        self.config = config
        self.ros2_interface = ROS2Interface(config=config)
        self._connected = False
        self._calibrated = True
        self._home = False


        self._mutex = threading.Lock()


    @property
    def action_features(self) -> dict[str, type]:
        return{}


    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._connected

    def connect(self, calibrate: bool = True) -> None:
        self.ros2_interface.connect()
        self._connected = True

    @property
    def is_calibrated(self) -> bool:
        return self._calibrated

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def get_action(self) -> dict[str, Any]:
        # Get the raw joint state from the teachbot interface
        joint_state = self.ros2_interface.joint_state
        if not joint_state or "position" not in joint_state:
            return {}

        # Map teachbot joints to arm joints, apply offsets and scale
        mapped_action = {}
        arm_joint_names = getattr(self.config, "arm_joint_names", [])
        # Try to get offsets and scales from config, fallback to 0/1
        offsets = getattr(self.config, "target_degree_offsets", {})
        scales = getattr(self.config, "joint_scale_factors", {})

        for joint in arm_joint_names:
            pos = joint_state["position"].get(joint)
            if pos is None:
                continue
            offset = offsets.get(joint, 0.0)
            scale = scales.get(joint, 1.0)
            mapped_action[joint] = (pos + offset) * scale

        # Optionally add gripper if present
        if self.config.use_gripper and "gripper" in joint_state:
            mapped_action["gripper"] = joint_state["gripper"]
            #print(f"[Teachbot.get_action] Gripper in action: {joint_state['gripper']:.4f}")
        else:
            mapped_action["gripper"] = 0.0  # Default gripper state if not used
            #print(f"[Teachbot.get_action] No gripper in joint_state, using default 0.0")

        #print(f"[Teachbot.get_action] Full action: {mapped_action}")
        return mapped_action

    def send_feedback(self, feedback: dict[str, float]) -> None:
        pass

    def disconnect(self) -> None: 
        self.ros2_interface.disconnect()
        self._connected = False
        self._calibrated = False
        self._home = False
