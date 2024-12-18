import magicbot
import wpilib

from components.drivetrain import DrivetrainComponent
from input_devices.driver import DriverController


class MyRobot(magicbot.MagicRobot):

    drivetrain: DrivetrainComponent

    def createObjects(self) -> None:
        print('creating objects')
        self.data_log = wpilib.DataLogManager.getLog()
        self.field = wpilib.Field2d()
        wpilib.SmartDashboard.putData(self.field)

    def teleopInit(self):
        self.driver_controller = DriverController(0)

    def teleopPeriodic(self) -> None:
        left_power = -self.driver_controller.getLeftThrottle()
        right_power = -self.driver_controller.getRightThrottle()
        pn = wpilib.SmartDashboard.putNumber
        pn('left_power', left_power)
        pn('right_power', right_power)
        self.drivetrain.drive_tank(left_power, right_power)
