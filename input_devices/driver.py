import wpilib


def calculate_tank_drive_speeds(forward, rotation):
    """
    Calculate motor speeds for a tank drive robot.

    Args:
        forward (float): Forward/backward joystick input (-1 (back) to 1 (forward)).
        rotation (float): Rotation joystick input (1 (right) to -1 (left)).

    Returns:
        tuple: (left_speed, right_speed)
    """
    # Calculate raw motor speeds
    left_speed = forward - rotation
    right_speed = forward + rotation

    # Normalize speeds to ensure they're within the range [-1, 1]
    max_magnitude = max(1, abs(left_speed), abs(right_speed))
    left_speed /= max_magnitude
    right_speed /= max_magnitude

    return left_speed, right_speed

class DriverController:
    xbox = False
    logitech = False
    logitech_flight = False

    def __init__(self, port: int) -> None:
        jsname = wpilib.DriverStation.getJoystickName(port)
        if jsname.startswith('Microsoft Xbox'):
            self.xbox = True
        elif jsname == 'Logitech Logitech Extreme 3D':  # Flight stick
            self.logitech_flight = True
        elif jsname.startswith('Logitech'):  # gamepad
            self.logitech = True
        else:
            raise ValueError(f'Joystick port {port} must have either an XBox or Logitech controller assigned')
        self.joystick = wpilib.Joystick(port)


    def getLeftThrottle(self) -> float:
        if self.xbox or self.logitech:
            return self.joystick.getRawAxis(1)
        if self.logitech_flight:
            fwd = self.joystick.getRawAxis(1)
            rot = self.joystick.getRawAxis(2)
            left, right = calculate_tank_drive_speeds(fwd, rot)
            return left

    def getRightThrottle(self) -> float:
        if self.xbox:
            return self.joystick.getRawAxis(4)
        if self.logitech:
            return self.joystick.getRawAxis(5)
        if self.logitech_flight:
            fwd = self.joystick.getRawAxis(1)
            rot = self.joystick.getRawAxis(2)
            left, right = calculate_tank_drive_speeds(fwd, rot)
            return right

