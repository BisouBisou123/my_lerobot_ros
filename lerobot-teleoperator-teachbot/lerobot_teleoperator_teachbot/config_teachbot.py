from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig


from dataclasses import dataclass, field

@TeleoperatorConfig.register_subclass("lerobot_teleoperator_teachbot")
@dataclass
class TeachbotConfig(TeleoperatorConfig):
    use_gripper: bool = True
    vacuum_gripper: bool = False  # Set to True if using a vacuum gripper
    arm_joint_names: list[str] = field(default_factory=lambda: [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint"
    ])
    gripper_joint_name: str = "robotiq_85_left_knuckle_joint"  # Update if you have a gripper joint name
    target_joint_names: list[str] = field(default_factory=lambda: [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint"
    ])
    target_joint_offsets: list[float] = field(default_factory=lambda: [
        0.0,
        -90.0,
        -90.0,
        0.0,
        90.0,
        0.0
    ])
    target_joint_state_topic: str = "/joint_states"
    joint_threshold: float = 30.0  # Set a default threshold value for jointbetween teachbot and robot (in degrees)
