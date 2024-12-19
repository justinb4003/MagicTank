from phoenix6.configs import (
    MotorOutputConfigs,
)
from phoenix6.hardware import TalonFX
from phoenix6.signals import InvertedValue, NeutralModeValue


class DrivetrainComponent:

    def __init__(self) -> None:
        print('drivetrain init')
        self.left_motor = TalonFX(11)
        self.right_motor = TalonFX(21)

        lconf = self.left_motor.configurator
        rconf = self.right_motor.configurator

        left_config = MotorOutputConfigs()
        left_config.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        left_config.neutral_mode = NeutralModeValue.BRAKE
        
        right_config = MotorOutputConfigs()
        right_config.inverted = InvertedValue.CLOCKWISE_POSITIVE
        right_config.neutral_mode = NeutralModeValue.BRAKE

        lconf.apply(left_config)
        rconf.apply(right_config)
        
        self.left_power = 0
        self.right_power = 0

    def setup(self) -> None:
        print('drivetrain setup')

    def on_enable(self) -> None:
        print('drivetrain enabled')
    
    def drive_tank(self, left_power: float, right_power: float) -> None:
        self.left_power = left_power
        self.right_power = right_power
    
    def execute(self) -> None:
        # print('drivetrain execute')
        self.left_motor.set(self.left_power)
        self.right_motor.set(self.right_power)

