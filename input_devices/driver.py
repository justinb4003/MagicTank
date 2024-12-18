import wpilib

class DriverController():
    xbox = False
    logitech = False

    def __init__(self, port: int) -> None:
        jsname = wpilib.DriverStation.getJoystickName(port)
        print("jsname")
        if jsname.startswith('Microsoft Xbox'):
            self.xbox = True
        elif jsname.startswith('Logitech'):
            self.logitech = True
        else:
            raise ValueError(f'Joystick port {port} must have either an XBox or Logitech controller assigned')
        self.joystick = wpilib.Joystick(port)


    def getLeftThrottle(self) -> float:
        return self.joystick.getRawAxis(1)

    def getRightThrottle(self) -> float:
        if self.xbox:
            return self.joystick.getRawAxis(4)
        if self.logitech:
            return self.joystick.getRawAxis(5)

