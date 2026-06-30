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
    # ROS namespace prefix used for all topics/services (empty string means global namespace).
    namespace: str = ""
    # Ordered list of arm joint names as they appear in JointState and controllers.
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
    # Joint name used for the gripper in ROS interfaces and command messages.
    gripper_joint_name: str = "gripper_joint"
    # Reference frame used for Cartesian commands (typically the robot base frame).
    base_link: str = "base_link"
    # Maximum commanded linear speed in meters/second for Cartesian servo modes.
    max_linear_velocity: float = 0.10
    # Maximum commanded angular speed in radians/second for Cartesian servo modes.
    max_angular_velocity: float = 0.25
    # Maximum commanded joint speed in radians/second for MoveIt Servo joint jog mode.
    max_joint_jog_velocity: float = 5.0
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
    # Gripper command value corresponding to fully open (controller-specific scale).
    gripper_open_position: float = 0.0
    # Gripper command value corresponding to fully closed (controller-specific scale).
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
    # Control mode used by the ROS2 interface to select the command backend.
    action_type: ActionType = ActionType.NONE
    # Topic for publishing full trajectory goals to a trajectory-capable controller.
    trajectory_publisher: str = "/arm_controller/joint_trajectory"
    # Action/service endpoint used for joint trajectory execution in simulation.
    joint_trajectory_controller_sim: str = "/joint_trajectory_controller/follow_joint_trajectory"
    # Action/service endpoint used for scaled trajectory execution on real hardware.
    joint_trajectory_controller: str = "/scaled_joint_trajectory_controller/follow_joint_trajectory"
    # Topic for direct joint-position command streaming.
    joint_position_controller_commands: str = "/forward_position_controller/commands"
    # Topic for MoveIt Servo delta-joint commands.
    servo_delta_joint_cmds: str = "/servo_node/delta_joint_cmds"
    # Topic for MoveIt Servo pose target commands.
    servo_pose_cmds: str = "/servo_node/pose_target_cmds"
    # Topic for MoveIt Servo delta-twist commands.
    servo_delta_twist_cmds: str = "/servo_node/delta_twist_cmds"
    # Service used to pause or unpause MoveIt Servo.
    servo_pause: str = "/servo_node/pause_servo"
    # Service used to switch MoveIt Servo command type (joint/pose/twist).
    servo_switch_command_type: str = "/servo_node/switch_command_type"
    # Whether the stack is connected to simulation instead of physical hardware.
    sim: bool = False
    # Simplified default lower joint limits in radians used by downstream code paths.
    min_joint_positions: list[float] = field(
        default_factory=lambda: [-6.283, -2.356, -3.141, -6.283, -6.283, -6.283]
    )
    # Simplified default upper joint limits in radians used by downstream code paths.
    max_joint_positions: list[float] = field(
        default_factory=lambda: [6.283, 2.356, 3.141, 6.283, 6.283, 6.283]
    )


@dataclass
class UrROS2InterfaceConfig(ROS2InterfaceConfig):
    # Uncomment one of the action types below to select the control mode for the UR robot. Make sure to implement the corresponding logic in the ROS2Interface class.
    #action_type: ActionType = ActionType.CARTESIAN_VELOCITY # Not implemted
    #action_type: ActionType = ActionType.JOINT_POSITION # Tested: Oke, maar haperend
    action_type: ActionType = ActionType.MOVEGROUP_FOLLOW_JOINT_TRAJECTION # Tested: Oke
    #action_type: ActionType = ActionType.MOVEGROUP_SERVO_TWIST # Tested: Oke
    #action_type: ActionType = ActionType.MOVEGROUP_SERVO_POSE # Not Tested
    # Default UR control mode; can be changed for experiments/tests.
    action_type: ActionType = ActionType.MOVEGROUP_SERVO_JOG # Tested: Oke



@dataclass
class ROS2Config(RobotConfig):
    # Optional safety clamp for relative target step size (None disables this clamp).
    max_relative_target: int | None = None
    # Camera configuration map keyed by camera name.
    cameras: dict[str, CameraConfig] = field(default_factory=dict)
    # ROS2 communication and control configuration for the selected robot backend.
    ros2_interface: ROS2InterfaceConfig = field(default_factory=ROS2InterfaceConfig)

@RobotConfig.register_subclass("lerobot_robot_ur")
@dataclass
class UrConfig(ROS2Config):
    """Configuration for the Universal Robots UR5 with ROS 2."""
    # UR-specific ROS2 interface defaults.
    ros2_interface: UrROS2InterfaceConfig = field(default_factory=UrROS2InterfaceConfig)


# Custom robot config that allows overriding the action type and other parameters, while still being compatible with the UR robot implementation
# This can be used for other robots that have a similar ROS 2 interface, but different control modes or parameters. 
# The Custom robot can be used with the same UR implementation, as long as the action features are compatible with the selected action type.
@dataclass
class CustomROS2InterfaceConfig(ROS2InterfaceConfig):

    # Default custom-robot control mode.
    action_type: ActionType = ActionType.MOVEGROUP_SERVO_JOG
    # Custom arm joint naming (example placeholders for 6-DOF manipulators).
    arm_joint_names: list[str] = field(
        default_factory=lambda: [
            "link1",
            "link2",
            "link3",
            "link4",
            "link5",
            "link6",
        ]
    )
    # Target degree offsets for each joint (used for calibration or initial positioning).
    target_degree_offsets: dict[str, float] = field(default_factory=lambda: {
        "link1": 0.0,
        "link2": -90.0,
        "link3": -90.0,
        "link4": 0.0,
        "link5": 90.0,
        "link6": 0.0,
    })
    # Joint scale factors (use -1.0 to invert direction) telop --> arm
    joint_scale_factors: dict[str, float] = field(default_factory=lambda: {
        "link1": 1.0,
        "link2": 1.0,
        "link3": 1.0,
        "link4": 1.0,
        "link5": 1.0,
        "link6": 1.0,
    })
    # Mapping from teleoperation joint names to actual joint names
    lookup_joint_names_from_telop: dict[str, str] = field(default_factory=lambda: {
        "shoulder_pan_joint": "link1",
        "shoulder_lift_joint": "link2",
        "elbow_joint": "link3",
        "wrist_1_joint": "link4",
        "wrist_2_joint": "link5",
        "wrist_3_joint": "link6",
    })
    # Other parameters can be overridden as needed

@RobotConfig.register_subclass("lerobot_robot_custom")
@dataclass
class CustomConfig(ROS2Config):
    """Configuration for the Custom robot with ROS 2."""
    # Custom-robot ROS2 interface defaults.
    ros2_interface: CustomROS2InterfaceConfig = field(default_factory=CustomROS2InterfaceConfig)

