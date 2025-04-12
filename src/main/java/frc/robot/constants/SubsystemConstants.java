package frc.robot.constants;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.utils.limelight.Limelight;

import static frc.robot.constants.TunerConstants.kSpeedAt12Volts;

public class SubsystemConstants {
    public static class DrivetrainConstants {
        public static  class AutoConstants {
            public static final double TRANSLATION_MAX = 4.35;
            public static final double TRANSLATION_MAX_A = 4.25;//3.05 3.6;
            public static final Rotation2d ROTATION_MAX = Rotation2d.fromDegrees(360.0);
            public static final Rotation2d ROTATION_MAX_A = Rotation2d.fromDegrees(560.0);

            public static final double LIMITED_TRANSLATION = 4.0;
            public static final double LIMITED_TRANSLATION_A = 3.5;

            public static final double TRANSLATION_P = 10.0;
            public static final double TRANSLATION_I = 0.00005; // This has to be greater that 0 I think
            public static final double TRANSLATION_D = 0.0;

            public static final double ROTATION_P = 10.0;
            public static final double ROTATION_I = 0.0;
            public static final double ROTATION_D = 0.0;
        }

        public static class TeleConstants {
            public static final double MAX_TRANSLATION = kSpeedAt12Volts.in(MetersPerSecond);
            public static final double MAX_ANGULAR = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
        }
    }

    public static class LimeLightConstants {
        public static Pose3d FRONT_LIMELIGHT_POSE = new Pose3d(
            0.368878,
            -0.071628,
            0.2425,
            new Rotation3d(
                Rotation2d.fromDegrees(90).getRadians(), 
                Rotation2d.fromDegrees(26.471).getRadians(),
                0
            )
        );
        public static Pose3d FRONT_LIMELIGHT_2_POSE = new Pose3d(
            -0.034556,
            0.280792,
            0.211222,
            new Rotation3d(
                Rotation2d.fromDegrees(90).getRadians(),
                0, 

                0
            )
        );
        public static Pose3d ELEVATOR_LIMELIGHT_POSE = new Pose3d(
            0.308471, //Forward to back
            -0.107326, //Right to left
            0.984077, // down to up
            new Rotation3d(
                Rotation2d.fromDegrees(0).getRadians(),
                Rotation2d.fromDegrees(50).getRadians(),//?
                Rotation2d.fromDegrees(180).getRadians()
            )
        );
        public static Limelight FRONT_LIMELIGHT = new Limelight("limelight-front");
        public static Limelight ELEVATOR_LIMELIGHT = new Limelight("limelight-elevate");
        public static Limelight FRONT_2_LIMELIGHT = new Limelight("limelight-mid");
    }

    public static class ArmevatorConstants {
        public static final int CURRENT_LIMIT_ELEVATOR_MOTORS = 60;
        public static final int CURRENT_LIMIT_ARM_MOTOR = 40;
        public static final int NOMINAL_VOLTAGE = 12;

        public static final double ELEVATOR_GEAR_RATIO = Meters.convertFrom(1.055544, Inches);
        public static final double ARM_GEAR_RATIO = 1.0 / 58.7755;

        public static final double MAX_ELEVATOR_HEIGHT = Meters.convertFrom(5, Feet);

        public static final Rotation2d SAFE_ANGLE = Rotation2d.fromDegrees(5);
        public static final double SAFE_ELEVATOR_HEIGHT = Meters.convertFrom(23, Inches);
        
        public static final double MAVCODER_OFFSET = 360.0 - 115.5;

        public static final double ELEVATOR_P = 4.0;
        public static final double ELEVATOR_I = 0.0;
        public static final double ELEVATOR_D = 0.0;
        public static final double ELEVATOR_FF = 0.025;

        public static final double ARM_P = 8.0;
        public static final double ARM_I = 0.0;
        public static final double ARM_D = 1.0;
        public static final double ARM_FF = 0.02;
    }

    public static class ClimberConstants {
        public static final int CLIMBER_CURRENT_LIMIT = 60;
    }

    public static class FeederConstants {
        public static final int FEEDER_CAN_FRONT_TRIGGER_DISTANCE = 200;
        public static final int FEEDER_CAN_BACK_TRIGGER_DISTANCE = 135;
    }

    public static class CoralManipulatorConstants {
        public static final int CURRENT_LIMIT_CORAL = 60;

        public static final double CORAL_MANIPULATOR_P = 5;
        public static final double CORAL_MANIPULATOR_I = 0.00;
        public static final double CORAL_MANIPULATOR_D = 0.0;

        public static final double POSITION_CONVERSION_FACTOR = 11.0 / 38.0;
        public static final double CORAL_MOTOR_DISTANCE_FACTOR = POSITION_CONVERSION_FACTOR * 3.26262626262626 * 1/7168;
    }
}
