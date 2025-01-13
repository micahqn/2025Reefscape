import commands2
import commands2.button
import commands2.cmd
from commands2.button import CommandXboxController
from pathplannerlib.auto import AutoBuilder
from phoenix6 import SignalLogger
from wpilib import SmartDashboard
from wpimath.units import rotationsToRadians

from generated.tuner_constants import TunerConstants
from robot_state import RobotState
from subsystems.superstructure import Superstructure
from subsystems.swerve import SwerveSubsystem


class RobotContainer:

    def __init__(self) -> None:
        self._max_speed = (
            TunerConstants.speed_at_12_volts
        )  # speed_at_12_volts desired top speed
        self._max_angular_rate = rotationsToRadians(
            1
        )  # 3/4 of a rotation per second max angular velocity

        self._driver_controller = commands2.button.CommandXboxController(0)

        self.drivetrain = TunerConstants.create_drivetrain(controller=self._driver_controller)
        self.superstructure = Superstructure(self.drivetrain)
        self._robot_state = RobotState(self.drivetrain)

        # Setting up bindings for necessary control of the swerve drive platform


        # Path follower
        self._auto_chooser = AutoBuilder.buildAutoChooser("Auto Chooser")
        SmartDashboard.putData("Auto Mode", self._auto_chooser)

        # Configure the button bindings
        self.configure_button_bindings()

    def configure_button_bindings(self) -> None:

        self._driver_controller.a().whileTrue(self.drivetrain.set_desired_state_command(SwerveSubsystem.SubsystemState.BRAKE))
        self._driver_controller.b().whileTrue(self.drivetrain.set_desired_state_command(SwerveSubsystem.SubsystemState.POINT))

        # SysId commands don't end automatically, so we specify to switch to field centric once False
        (self._driver_controller.back() & self._driver_controller.y()).onTrue(commands2.InstantCommand(lambda: SignalLogger.start())).whileTrue(
            self.drivetrain.set_desired_state_command(SwerveSubsystem.SubsystemState.SYS_ID_DYNAMIC_FORWARD)
        ).onFalse(self.drivetrain.set_desired_state_command(SwerveSubsystem.SubsystemState.FIELD_CENTRIC))
        (self._driver_controller.back() & self._driver_controller.x()).onTrue(commands2.InstantCommand(lambda: SignalLogger.start())).whileTrue(
            self.drivetrain.set_desired_state_command(SwerveSubsystem.SubsystemState.SYS_ID_DYNAMIC_REVERSE)
        ).onFalse(self.drivetrain.set_desired_state_command(SwerveSubsystem.SubsystemState.FIELD_CENTRIC))
        (self._driver_controller.start() & self._driver_controller.y()).onTrue(commands2.InstantCommand(lambda: SignalLogger.start())).whileTrue(
            self.drivetrain.set_desired_state_command(SwerveSubsystem.SubsystemState.SYS_ID_QUASI_FORWARD)
        ).onFalse(self.drivetrain.set_desired_state_command(SwerveSubsystem.SubsystemState.FIELD_CENTRIC))
        (self._driver_controller.start() & self._driver_controller.x()).onTrue(commands2.InstantCommand(lambda: SignalLogger.start())).whileTrue(
            self.drivetrain.set_desired_state_command(SwerveSubsystem.SubsystemState.SYS_ID_QUASI_REVERSE)
        ).onFalse(self.drivetrain.set_desired_state_command(SwerveSubsystem.SubsystemState.FIELD_CENTRIC))

        self._driver_controller.leftBumper().onTrue(
            self.drivetrain.runOnce(lambda: self.drivetrain.seed_field_centric())
        )

        self._driver_controller.rightBumper().whileTrue(self.drivetrain.set_desired_state_command(SwerveSubsystem.SubsystemState.ROBOT_CENTRIC))

        self.drivetrain.register_telemetry(lambda state: self._robot_state.log_swerve_state(state))

    def get_driver_controller(self) -> CommandXboxController:
        return self._driver_controller

    def get_autonomous_command(self) -> commands2.Command:
        return self._auto_chooser.getSelected()
