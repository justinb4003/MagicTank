import math
import wpilib
import phoenix6

from pyfrc.physics import drivetrains
from pyfrc.physics.core import PhysicsInterface
from wpilib.simulation import DCMotorSim, SimDeviceSim
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.system.plant import DCMotor, LinearSystemId
from wpimath.units import kilogram_square_meters

from components.drivetrain import DrivetrainComponent

from robot import MyRobot


class SimpleTalonFXMotorSim:
    def __init__(
        self, motor: phoenix6.hardware.TalonFX, units_per_rev: float, kV: float
    ) -> None:
        self.sim_state = motor.sim_state
        self.sim_state.set_supply_voltage(12.0)
        self.kV = kV  # volt seconds per unit
        self.units_per_rev = units_per_rev

    def update(self, dt: float) -> None:
        voltage = self.sim_state.motor_voltage
        velocity = voltage / self.kV  # units per second
        velocity_rps = velocity * self.units_per_rev
        self.sim_state.set_rotor_velocity(velocity_rps)
        self.sim_state.add_rotor_position(velocity_rps * dt)


class Falcon500MotorSim:
    def __init__(
        self,
        *motors: phoenix6.hardware.TalonFX,
        # Reduction between motor and encoder readings, as output over input.
        # If the mechanism spins slower than the motor, this number should be greater than one.
        gearing: float,
        moi: kilogram_square_meters,
    ):
        self.falcon = DCMotor.falcon500(len(motors))
        self.plant = LinearSystemId.DCMotorSystem(self.falcon, moi, gearing)
        self.gearing = gearing
        self.sim_states = [motor.sim_state for motor in motors]
        for sim_state in self.sim_states:
            sim_state.set_supply_voltage(12.0)
        self.motor_sim = DCMotorSim(self.plant, self.falcon)

    def update(self, dt: float) -> None:
        voltage = self.sim_states[0].motor_voltage
        self.motor_sim.setInputVoltage(voltage)
        self.motor_sim.update(dt)
        motor_rev_per_mechanism_rad = self.gearing / math.tau
        for sim_state in self.sim_states:
            sim_state.set_raw_rotor_position(
                self.motor_sim.getAngularPosition() * motor_rev_per_mechanism_rad
            )
            sim_state.set_rotor_velocity(
                self.motor_sim.getAngularVelocity() * motor_rev_per_mechanism_rad
            )


class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: MyRobot):
        self.physics_controller = physics_controller
        self.sim_drivetrain = drivetrains.TwoMotorDrivetrain()
        self.robot = robot

        gear_ratio = 7.43
        moi = 0.0009972
        # Motors
        self.left_drive = Falcon500MotorSim(
                robot.drivetrain.left_motor,
                gearing=gear_ratio,
                moi=moi,
            )
        self.right_drive = Falcon500MotorSim(
                robot.drivetrain.right_motor,
                gearing=gear_ratio,
                moi=moi,
            )


    def update_sim(self, now: float, tm_diff: float) -> None:
        # Enable the Phoenix6 simulated devices
        # TODO: delete when phoenix6 integrates with wpilib
        if wpilib.DriverStation.isEnabled():
            phoenix6.unmanaged.feed_enable(0.1)

        drive_motors = [self.left_drive, self.right_drive]
        for dm in drive_motors:
            dm.update(tm_diff)

        pn = wpilib.SmartDashboard.putNumber
        # TODO: Calculate movement with sim
        lspeed = self.left_drive.motor_sim.getAngularVelocity()/100
        rspeed = self.right_drive.motor_sim.getAngularVelocity()/100
        pn('sim_left_speed', lspeed)
        pn('sim_right_speed', rspeed)
        speeds = self.sim_drivetrain.calculate(lspeed, rspeed)
        self.physics_controller.drive(speeds, tm_diff)
        pass

