from dataclasses import dataclass
from typing import List, Literal, Optional


@dataclass
class Command:
    """
    Represents a command to be executed by the manipulator.
    """

    command_type: Literal["JointPosition", "ManipulatorPosition"]
    """
    Command type.
    - "JointPosition": defines one joint position.
    - "ManipulatorPosition": defines all 4 joints positions.
    """

    joint_idx: Optional[int] = None
    """ Joint index, only used when command_type == "JointPosition". """

    q1: Optional[float] = None
    """ Joint 1 position. """

    q2: Optional[float] = None
    """ Joint 2 position. """

    q3: Optional[float] = None
    """ Joint 3 position. """

    q4: Optional[float] = None
    """ Joint 4 position. """

    def __post_init__(self):
        # Validation logic
        if self.command_type == "JointPosition" and self.joint_idx is None:
            raise ValueError("joint_idx must be specified for JointPosition commands.")
        if self.command_type == "ManipulatorPosition" and any(q is None for q in [self.q1, self.q2, self.q3, self.q4]):
            raise ValueError("All joint positions (q1–q4) must be specified for ManipulatorPosition.")

    def as_list(self) -> List[float]:
        """Return joint positions as a list."""
        return [self.q1, self.q2, self.q3, self.q4]

    def is_joint_command(self) -> bool:
        return self.command_type == "JointPosition"

    def is_manipulator_command(self) -> bool:
        return self.command_type == "ManipulatorPosition"
    
    def as_string(self) -> str:
        if self.command_type == "JointPosition":
            match self.joint_idx:
                case 1:
                    joint_pose = self.q1
                case 2:
                    joint_pose = self.q2
                case 3:
                    joint_pose = self.q3
                case 4:
                    joint_pose = self.q4
                case _:
                    raise ValueError("joint_idx out of range (1 to 4)")
            return f"SETQ{self.joint_idx} {joint_pose}"
        
        if self.command_type == "ManipulatorPosition":
            return f"SETALLQ {self.q1} {self.q2} {self.q3} {self.q4}"