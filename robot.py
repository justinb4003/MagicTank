import magicbot
import wpilib

from components.drivetrain import DrivetrainComponent
from input_devices.driver import DriverController


class MyRobot(magicbot.MagicRobot):

    # Here we declare our components which are like subsystems
    # these are set up for dependency injection into other components
    # and controls
    drivetrain: DrivetrainComponent

   
    def createObjects(self) -> None:
        # Unlike most robot projects we don't actually create our subsystems
        # here. The declearation above takes care of that. Here we only
        # need to "special" things that aren't our robot parts.
        print('creating objects')
        self.data_log = wpilib.DataLogManager.getLog()
        self.field = wpilib.Field2d()
        wpilib.SmartDashboard.putData(self.field)

    def teleopInit(self):
        # We instantiate the controllers here instead of createObjects()
        # because an auton doesn't need a controller and no point in crashing
        # unless we have an actual problem.
        self.driver_controller = DriverController(0)

    def teleopPeriodic(self) -> None:
        # A very basic drive control mech
        left_power = -self.driver_controller.getLeftThrottle()
        right_power = -self.driver_controller.getRightThrottle()
        pn = wpilib.SmartDashboard.putNumber
        pn('left_power', left_power)
        pn('right_power', right_power)
        self.drivetrain.drive_tank(left_power, right_power)
