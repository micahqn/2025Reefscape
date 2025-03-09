from phoenix6.configs.config_groups import Slot0Configs
from phoenix6.signals import GravityTypeValue
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout


class Constants:

    apriltag_layout = AprilTagFieldLayout.loadField(AprilTagField.k2025ReefscapeWelded)

    class CanIDs:
        LEFT_ELEVATOR_TALON = 10
        RIGHT_ELEVATOR_TALON = 11
        INTAKE_TALON = 12
        LEFT_PIVOT_TALON = 13
        RIGHT_PIVOT_TALON = 14
        CLIMB_TALON = 15
        FUNNEL_TALON = 22

        ELEVATOR_CANDI = 20
        PIVOT_CANCODER = 21

    class ClimberConstants:
        GEAR_RATIO = 61504/189
        GAINS = (Slot0Configs()
            .with_k_p(1.0)
            .with_k_i(0.0)
            .with_k_d(0.0)
            .with_k_s(0.0)
            .with_k_v(0.0)
            .with_k_a(0.0)
        )

        VOLTAGE_INWARDS = 16
        VOLTAGE_OUTWARDS = -4

        SERVO_PORT = 0
        SERVO_ENGAGED_ANGLE = 0
        SERVO_DISENGAGED_ANGLE = 90

    class ElevatorConstants:
        L1_SCORE_POSITION = 0.690674
        L2_SCORE_POSITION = 1.841
        L3_SCORE_POSITION = 3.576
        L4_SCORE_POSITION = 6.087158
        L2_ALGAE_POSITION = 3.198
        L3_ALGAE_POSITION = 5
        NET_SCORE_POSITION = 6.052246
        ELEVATOR_MAX = 6.096924

        DEFAULT_POSITION = 0

        CRUISE_VELOCITY = 8
        MM_UPWARD_ACCELERATION = 48
        MM_BRAKE_ACCELERATION = 24
        MM_DOWNWARD_ACCELERATION = 12
        EXPO_K_V = 10
        EXPO_K_A = 4

        GEAR_RATIO = 31/4
        GAINS = (Slot0Configs()
            .with_k_g(0.36)
            .with_k_p(40)
            .with_k_i(0.0)
            .with_k_d(0.0)
            .with_k_s(0.11)
            .with_k_v(0.0)
            .with_k_a(0.0)
            .with_gravity_type(GravityTypeValue.ELEVATOR_STATIC)
        )

        SETPOINT_TOLERANCE = 0.1

    class PivotConstants:
        INSIDE_ELEVATOR_ANGLE = 0.262207 # Used for subsystem collision checking
        ELEVATOR_PRIORITY_ANGLE = 0.201943 # We move the pivot to this position until the elevator has reached its setpoint.
        STOW_ANGLE = 0.2854
        GROUND_INTAKE_ANGLE = -0.081543
        FUNNEL_INTAKE_ANGLE = 0.322
        ALGAE_INTAKE_ANGLE = -0.033
        HIGH_SCORING_ANGLE =  0.262
        MID_SCORING_ANGLE = 0.262
        LOW_SCORING_ANGLE = 0.338379
        NET_SCORING_ANGLE = 0.123535
        PROCESSOR_SCORING_ANGLE = 0.004639
        CLIMBER_PRIORITY_ANGLE = 0.201943

        MINIMUM_ANGLE = -0.091
        MAXIMUM_ANGLE = 0.392822

        CRUISE_VELOCITY = 3
        MM_ACCELERATION = 2

        GEAR_RATIO = 961/36
        GAINS = (Slot0Configs()
                 .with_k_g(0.0)
                 .with_k_p(60)
                 .with_k_i(0.0)
                 .with_k_d(0.0)
                 .with_k_s(0.0)
                 .with_k_v(0.0)
                 .with_k_a(0.0)
                 .with_gravity_type(GravityTypeValue.ARM_COSINE)
        )

        CANCODER_DISCONTINUITY = 0.8
        CANCODER_OFFSET = 0.4751769375

        SETPOINT_TOLERANCE = 0.03125

    class IntakeConstants:

        CORAL_INTAKE_SPEED = 0.5
        FUNNEL_INTAKE_SPEED = 0.4
        CORAL_OUTPUT_SPEED = 0.425

        ALGAE_INTAKE_SPEED = 1
        ALGAE_OUTPUT_SPEED = -1

        GEAR_RATIO = 4
        GAINS = (Slot0Configs()
            .with_k_p(1.0)
            .with_k_i(0.0)
            .with_k_d(0.0)
            .with_k_s(0.0)
            .with_k_v(0.0)
            .with_k_a(0.0)
        )

    class VisionConstants:
        FRONT_LEFT = "limelight-fl"
        FRONT_RIGHT = "limelight-fr"
        FRONT_CENTER = "limelight-front"
        BACK_CENTER = "limelight-back"

    class FunnelConstants:

        CORAL_STATION_POSITION = 0.128
        STOWED_POSITION = 0

        GEAR_RATIO = 192/7

        CRUISE_VELOCITY = 1 

        SETPOINT_TOLERANCE = 0.01

        MM_ACCELERATION = 1

        GAINS = (Slot0Configs()
            .with_k_p(35)
            .with_k_i(0.0)
            .with_k_d(0.0)
            .with_k_s(0.0)
            .with_k_v(0.0)
            .with_k_a(0.0)
            .with_k_g(0.2811) # Shoutout StormBots
            .with_gravity_type(GravityTypeValue.ARM_COSINE)
        )

        SUPPLY_LIMIT = 20
        STATOR_LIMIT = 50
