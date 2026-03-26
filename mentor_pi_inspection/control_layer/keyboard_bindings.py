"""Pure-Python helpers for keyboard teleoperation bindings."""
from dataclasses import dataclass
from typing import Optional


HELP_KEYS = {'h', '?'}
QUIT_KEYS = {'\x03'}
STOP_KEYS = {' ', 'k'}


@dataclass(frozen=True)
class TeleopCommand:
    """A velocity or stop command derived from a single keypress."""

    linear_x: float = 0.0
    linear_y: float = 0.0
    angular_z: float = 0.0
    emergency_stop: bool = False
    description: str = 'stop'

    @property
    def is_motion(self) -> bool:
        return any((self.linear_x, self.linear_y, self.angular_z))


def command_for_key(
    key: str,
    linear_speed: float,
    lateral_speed: float,
    angular_speed: float,
) -> Optional[TeleopCommand]:
    """Translate a keypress into a teleop command."""
    if not key:
        return None

    normalized = key.lower()
    if normalized == 'w':
        return TeleopCommand(linear_x=linear_speed, description='forward')
    if normalized == 's':
        return TeleopCommand(linear_x=-linear_speed, description='backward')
    if normalized == 'a':
        return TeleopCommand(linear_y=lateral_speed, description='strafe left')
    if normalized == 'd':
        return TeleopCommand(linear_y=-lateral_speed, description='strafe right')
    if normalized == 'q':
        return TeleopCommand(angular_z=angular_speed, description='rotate left')
    if normalized == 'e':
        return TeleopCommand(angular_z=-angular_speed, description='rotate right')
    if normalized in STOP_KEYS:
        return TeleopCommand(emergency_stop=True, description='emergency stop')
    return None
