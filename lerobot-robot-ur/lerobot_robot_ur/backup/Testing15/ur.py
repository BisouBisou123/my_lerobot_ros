import logging
from pyexpat import features
from typing import Any
import time
import numpy as np


from control_msgs import action
from lerobot.cameras import make_cameras_from_configs
from lerobot.utils.errors import DeviceNotConnectedError, DeviceAlreadyConnectedError
from lerobot.robots.robot import Robot
from lerobot.robots.utils import ensure_safe_goal_position


from .config_ur import ActionType
from .config_ur import CustomConfig
from .config_ur import UrConfig
from .ros_interface_ur import ROS2Interface



logger = logging.getLogger(__name__)


class Ur(Robot):

    config_class = UrConfig
    name = "ur"

    def __init__(self, config: UrConfig):
        super().__init__(config)
        self.config = config
        self.ros2_interface = ROS2Interface(config=config)
        self.cameras = make_cameras_from_configs(config.cameras)
        self.is_connected = False

        

    def connect(self, calibrate: bool = True) -> None:
## print toegevoegd        
        # print("=== ACTION FEATURES ===")
        # print(self.action_features)
        # print("=== OBSERVATION FEATURES ===")
        # print(self.observation_features)

        # self.is_connected = True

        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} is already connected.")
        for cam in self.cameras.values():
            cam.connect()
        self.ros2_interface.connect()
        self.is_connected = True
        #logger.info(f"{self} connected.")

    


    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        """Command arm to move to a target joint configuration.

        The relative action magnitude may be clipped depending on the configuration parameter
        `max_relative_target`. In this case, the action sent differs from original action.
        Thus, this function always returns the action actually sent.

        Args:
            action (dict[str, float]): The goal positions for the motors or pressed_keys dict.

        Raises:
            DeviceNotConnectedError: if robot is not connected.

        Returns:
            dict[str, float]: The action sent to the motors, potentially clipped.
        """
## print toegevoegd     
        # print("RAW ACTION:", action)
        # print("RAW ACTION KEYS:", list(action.keys()))
        # print(f"Received action: {action}" )
        
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        if(self.config.ros2_interface.action_type in (ActionType.JOINT_POSITION, ActionType.MOVEGROUP_FOLLOW_JOINT_TRAJECTION, ActionType.MOVEGROUP_SERVO_JOG)):
            # Use lookup_joint_names_from_telop for mapping
            lookup = getattr(self.config.ros2_interface, "lookup_joint_names_from_telop", {})
            offsets = getattr(self.config.ros2_interface, "target_degree_offsets", {})
            scales = getattr(self.config.ros2_interface, "joint_scale_factors", {})
            arm_joint_names = self.config.ros2_interface.arm_joint_names

            joint_positions = []
            for arm_joint in arm_joint_names:
                # Find the teleop joint name that maps to this arm joint
                teleop_joint = None
                for k, v in lookup.items():
                    if v == arm_joint:
                        teleop_joint = k
                        break
                # Try both .pos and plain key

## .pos weg na action.get({xxx}
                teleop_val = action.get(f"{teleop_joint}", action.get(teleop_joint, 0.0)) if teleop_joint else 0.0
                offset_deg = offsets.get(arm_joint, 0.0)
                offset = offset_deg * 3.141592653589793 / 180.0  # convert degrees to radians
                scale = scales.get(arm_joint, 1.0)
                arm_val = (teleop_val + offset) * scale
                joint_positions.append(arm_val)
            #print(f"Mapped joint positions: {joint_positions}")
            # Prepare action dict with joint positions and gripper
            ros_action = {"joint_positions": joint_positions}


            ##########gripper toevoegen  aan dictonary


            # Extract and add gripper command if present
            gripper_val = action.get("gripper", None)
            if gripper_val is not None:
                ros_action["gripper_joint"] = gripper_val

## print gecommend
                # print(f"Including gripper in action: {gripper_val}")
## print toegevoegd
            # print("joint_positions:", joint_positions)                     
            # print("RAW ACTION:", action)
            # print("JOINT POSITIONS:", joint_positions)
            # print("ROS ACTION:", ros_action)


            self.ros2_interface.send_action(ros_action)
        elif self.config.ros2_interface.action_type == ActionType.MOVEGROUP_SERVO_TWIST:
            self.ros2_interface.send_action(action)
        elif self.config.ros2_interface.action_type == ActionType.MOVEGROUP_SERVO_POSE:
            pass
        else:
            raise ValueError(f"Unsupported action type: {self.config.ros2_interface.action_type}")

        #gripper_pos = action["gripper.pos"]
        #self.ros2_interface.send_gripper_command(gripper_pos)

        # Ensure all required action keys are present for dataset writing
        required_keys = [f"{joint}" for joint in self.config.ros2_interface.arm_joint_names]
        # Optionally add gripper
        if hasattr(self.config.ros2_interface, "gripper_joint_name"):
            required_keys.append(f"{self.config.ros2_interface.gripper_joint_name}")

        for key in required_keys:
            if key not in action:
                action[key] = 0.0
        # print(f"Final action sent: {action}")

## prints toegevoegd        
        # print("ACTION WRITTEN TO DATASET:")
        # print(action)
        
        return action


    def get_observation(self) -> dict[str, Any]:
## print toegevoegd        
        # print("GET_OBSERVATION CALLED")
        
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        obs_dict: dict[str, Any] = {}
        

# ## zelf toegevoegd
        joint_state = self.ros2_interface.joint_state
        

        # if joint_state is not None and "position" in joint_state:
        #     obs_dict["observation.state"] = np.array([
        #         joint_state["position"]["shoulder_pan_joint"],
        #         joint_state["position"]["shoulder_lift_joint"],
        #         joint_state["position"]["elbow_joint"],
        #         joint_state["position"]["wrist_1_joint"],
        #         joint_state["position"]["wrist_2_joint"],
        #         joint_state["position"]["wrist_3_joint"],
        #         # joint_state["position"]["gripper_joint"],
        #     ], dtype=np.float32)
        
        # # print(joint_state["position"]["gripper_joint"])

        if joint_state is None or "position" not in joint_state:
            logger.warning("Joint state 'position' not available yet.")
        else:
            # print("joint_state:", self.ros2_interface.joint_state)
            obs_dict.update({f"{joint}": pos for joint, pos in joint_state["position"].items()})
## print toegevoegd
        # print("STATE:", obs_dict["observation.state"])

        # Capture images from cameras
        for cam_key, cam in self.cameras.items():
            start = time.perf_counter()
            try:
                obs_dict[cam_key] = cam.async_read(timeout_ms=300)
            except Exception as e:
                logger.error(f"Failed to read camera {cam_key}: {e}")
                obs_dict[cam_key] = None
            dt_ms = (time.perf_counter() - start) * 1e3
            logger.debug(f"{self} read {cam_key}: {dt_ms:.1f}ms")

# ## print toegevoegd
#         print("=== OBS KEYS ===")
#         print(obs_dict.keys())
#         # print(obs_dict)       #FUCKED PERFORMANCE
#         print(joint_state["position"])
        
        return obs_dict

    def reset(self):
        pass

    def disconnect(self) -> None:
        if not self.is_connected:
            return

        self.ros2_interface.disconnect()

        
        for cam in self.cameras.values():
            try:
                cam.disconnect()
            except DeviceNotConnectedError:
                logger.warning(f"Tried to disconnect {cam}, but it was not connected.")

        self.is_connected = False
        logger.info(f"{self} disconnected.")

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def is_calibrated(self) -> bool:
        return True#self.is_connected

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @is_connected.setter
    def is_connected(self, value: bool) -> None:
        self._is_connected = value

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3)
            for cam in self.cameras
        }

    @property
    def _motors_ft(self) -> dict[str, type]:
        return {
                "shoulder_pan_joint": float,
                "shoulder_lift_joint": float,
                "elbow_joint": float,
                "wrist_1_joint": float,
                "wrist_2_joint": float,
                "wrist_3_joint": float,
                "gripper_joint": float,
        }



    @property
    def observation_features(self) -> dict[str, Any]:
        features = {**self._cameras_ft, **self._motors_ft}
        return features

## Zelf toegevoegd
    # @property
    # def observation_features(self) -> dict[str, Any]:
    #     features = {**self._motors_ft, **self._cameras_ft}

    #     features["observation.state"] = {
    #         "dtype": "float32",
    #         "shape": (7,),
    #         "names": [
    #             "shoulder_pan_joint",
    #             "shoulder_lift_joint",
    #             "elbow_joint",
    #             "wrist_1_joint",
    #             "wrist_2_joint",
    #             "wrist_3_joint",
    #             "gripper",
    #         ],
    #     }
    #     return features
################################################


    @property
    def cameras(self):
        return self._cameras

    @cameras.setter
    def cameras(self, value):
        self._cameras = value

    @property
    def config(self):
        return self._config

    @config.setter
    def config(self, value):
        self._config = value


##zelf toegevoegd#########
    # @property
    # def action_features(self) -> dict[str, type]:
    #     if self.config.ros2_interface.action_type in (ActionType.MOVEGROUP_SERVO_POSE, ActionType.MOVEGROUP_SERVO_TWIST):
    #         return {
    #             "linear_x.vel": float,
    #             "linear_y.vel": float,
    #             "linear_z.vel": float,
    #             "angular_x.vel": float,
    #             "angular_y.vel": float,
    #             "angular_z.vel": float,
    #             "gripper": float,
    #         }
    #     elif self.config.ros2_interface.action_type in (ActionType.JOINT_POSITION, ActionType.MOVEGROUP_FOLLOW_JOINT_TRAJECTION, ActionType.MOVEGROUP_SERVO_JOG):
    #         return {f"{joint}": float for joint in self.config.ros2_interface.arm_joint_names} | {
    #             "gripper": float
    #         }
    #     else:
    #         raise ValueError(f"Unsupported action type: {self.config.ros2_interface.action_type}")       
#########################


    @property
    def action_features(self) -> dict[str, type]:
        # Provide action features for each arm joint
        features = self._motors_ft #{f"{joint}": float for joint in self.config.ros2_interface.arm_joint_names}
        # Optionally add gripper
        if hasattr(self.config.ros2_interface, "gripper_joint_name"):  ##gripper_joint_name vervangen door gripper
            features[f"{self.config.ros2_interface.gripper_joint_name}"] = float
        return features

    def calibrate(self) -> None:
        # Implement calibration logic if needed
        pass

    def configure(self) -> None:
        # Implement configuration logic if needed
        pass

    def is_calibrated(self) -> bool:
        # Return True if robot is calibrated
        return True


    @property
    def is_connected(self) -> bool:
        return getattr(self, '_is_connected', False)

    @is_connected.setter
    def is_connected(self, value: bool) -> None:
        self._is_connected = value

# class implementing the same interface as Ur, but with a custom config that allows overriding the action type and other parameters.
class Custom(Ur):

    config_class = CustomConfig
    name = "custom"