from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig

from lerobot.robots.config import RobotConfig
from enum import Enum

class ActionType(Enum):
    NONE = "none"
    CARTESIAN_VELOCITY = "cartesian_velocity"
    JOINT_POSITION = "joint_position"
    MOVEGROUP_FOLLOW_JOINT_TRAJECTION = "movegroup_follow_joint_trajectory"
    MOVEGROUP_SERVO_TWIST = "movegroup_servo_twist"
    MOVEGROUP_SERVO_POSE = "movegroup_servo_pose"
    MOVEGROUP_SERVO_JOG = "movegroup_servo_jog"

@dataclass
class ROS2InterfaceConfig:
    namespace: str = ""
    arm_joint_names: list[str] = field(
        default_factory=lambda: [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
    )
    gripper_joint_name: str = "gripper_joint"
    base_link: str = "base_link"
    max_linear_velocity: float = 0.10
    max_angular_velocity: float = 0.25
    # UR5/UR10 joint limits in radians (example: UR5)
    min_joint_positions: list[float] = field(default_factory=lambda: [
        -6.283185307179586,  # shoulder_pan_joint
        -2.356194490192345,  # shoulder_lift_joint
        -3.141592653589793,  # elbow_joint
        -6.283185307179586,  # wrist_1_joint
        -6.283185307179586,  # wrist_2_joint
        -6.283185307179586,  # wrist_3_joint
    ])
    max_joint_positions: list[float] = field(default_factory=lambda: [
        6.283185307179586,   # shoulder_pan_joint
        0.0,                # shoulder_lift_joint
        3.141592653589793,  # elbow_joint
        6.283185307179586,  # wrist_1_joint
        6.283185307179586,  # wrist_2_joint
        6.283185307179586,  # wrist_3_joint
    ])
    gripper_open_position: float = 0.0
    gripper_close_position: float = 1.0
    # Joint degree offsets (in degrees) telop --> arm
    target_degree_offsets: dict[str, float] = field(default_factory=lambda: {
        "shoulder_pan_joint": 0.0,
        "shoulder_lift_joint": -90.0,
        "elbow_joint": -90.0,
        "wrist_1_joint": 0.0,
        "wrist_2_joint": 90.0,
        "wrist_3_joint": 0.0,
    })
    # Joint scale factors (use -1.0 to invert direction) telop --> arm
    joint_scale_factors: dict[str, float] = field(default_factory=lambda: {
        "shoulder_pan_joint": 1.0,
        "shoulder_lift_joint": 1.0,
        "elbow_joint": 1.0,
        "wrist_1_joint": 1.0,
        "wrist_2_joint": 1.0,
        "wrist_3_joint": 1.0,
    })
    # Mapping from teleoperation joint names to actual joint names
    lookup_joint_names_from_telop: dict[str, str] = field(default_factory=lambda: {
        "shoulder_pan_joint": "shoulder_pan_joint",
        "shoulder_lift_joint": "shoulder_lift_joint",
        "elbow_joint": "elbow_joint",
        "wrist_1_joint": "wrist_1_joint",
        "wrist_2_joint": "wrist_2_joint",
        "wrist_3_joint": "wrist_3_joint",
    })
    action_type: ActionType = ActionType.NONE
    trajectory_publisher: str = "/arm_controller/joint_trajectory"
    joint_trajectory_controller_sim: str = "/joint_trajectory_controller/follow_joint_trajectory"
    joint_trajectory_controller: str = "/scaled_joint_trajectory_controller/follow_joint_trajectory"
    joint_position_controller_commands: str = "/forward_position_controller/commands"
    servo_delta_joint_cmds: str = "/servo_node/delta_joint_cmds"
    servo_pose_cmds: str = "/servo_node/pose_target_cmds"
    servo_delta_twist_cmds: str = "/servo_node/delta_twist_cmds"
    servo_pause: str = "/servo_node/pause_servo"
    servo_switch_command_type: str = "/servo_node/switch_command_type"
    sim: bool = False
    min_joint_positions: list[float] = field(
        default_factory=lambda: [-6.283, -2.356, -3.141, -6.283, -6.283, -6.283]
    )
    max_joint_positions: list[float] = field(
        default_factory=lambda: [6.283, 2.356, 3.141, 6.283, 6.283, 6.283]
    )


@dataclass
class UrROS2InterfaceConfig(ROS2InterfaceConfig):
    # Uncomment one of the action types below to select the control mode for the UR robot. Make sure to implement the corresponding logic in the ROS2Interface class.
    #action_type: ActionType = ActionType.CARTESIAN_VELOCITY # Not implemted
    #action_type: ActionType = ActionType.JOINT_POSITION # Tested: Oke, maar haperend
    #action_type: ActionType = ActionType.MOVEGROUP_FOLLOW_JOINT_TRAJECTION # Tested: Oke
    action_type: ActionType = ActionType.MOVEGROUP_SERVO_TWIST # Not Tested
    #action_type: ActionType = ActionType.MOVEGROUP_SERVO_POSE # Not Tested
    #action_type: ActionType = ActionType.MOVEGROUP_SERVO_JOG # Not Tested



@dataclass
class ROS2Config(RobotConfig):
    max_relative_target: int | None = None
    cameras: dict[str, CameraConfig] = field(default_factory=dict)
    ros2_interface: ROS2InterfaceConfig = field(default_factory=ROS2InterfaceConfig)

@RobotConfig.register_subclass("lerobot_robot_ur")
@dataclass
class UrConfig(ROS2Config):
    """Configuration for the Universal Robots UR5 with ROS 2."""
    ros2_interface: UrROS2InterfaceConfig = field(default_factory=UrROS2InterfaceConfig)

    # Compatibility accessors for controllers that use a flat config API.
    @property
    def base_link(self) -> str:
        return self.ros2_interface.base_link

    @property
    def frame_id(self) -> str:
        return self.ros2_interface.base_link

    @property
    def action_type(self) -> ActionType:
        return self.ros2_interface.action_type

    @property
    def servo_delta_joint_cmds(self) -> str:
        return self.ros2_interface.servo_delta_joint_cmds

    @property
    def servo_delta_twist_cmds(self) -> str:
        return self.ros2_interface.servo_delta_twist_cmds

    @property
    def servo_pause(self) -> str:
        return self.ros2_interface.servo_pause

    @property
    def servo_switch_command_type(self) -> str:
        return self.ros2_interface.servo_switch_command_type

    @property
    def pose_cmds(self) -> str:
        return self.ros2_interface.servo_pose_cmds

    @property
    def max_linear_velocity(self) -> float:
        return self.ros2_interface.max_linear_velocity

    @property
    def max_angular_velocity(self) -> float:
        return self.ros2_interface.max_angular_velocity

