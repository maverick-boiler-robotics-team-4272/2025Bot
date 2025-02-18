package frc.robot.subsystems.coralManipulator;

// Hardware
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.constants.HardwareMap.*;
import static frc.robot.constants.SubsystemConstants.ArmevatorConstants.MAV_POSITION_FACTOR;
import static frc.robot.constants.SubsystemConstants.CoralManipulatorConstants.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.config.AnalogSensorConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.utils.hardware.Vortex;
import frc.robot.utils.hardware.VortexBuilder;

public class CoralManipulator extends SubsystemBase {
    private Vortex coralControllerMotor;

    public CoralManipulator() {
        coralControllerMotor = VortexBuilder.create(CORAL_MOTOR_ID)
            .withInversion(false)
            .withCurrentLimit(40)
            .withIdleMode(IdleMode.kBrake)
            .withAnalogConfig(
                new AnalogSensorConfig()
                    .inverted(true)
                    .positionConversionFactor(MAV_POSITION_FACTOR)
            )
            // .asFollower(ARM_MOTOR_1, true)
            .withPIDParams(CORAL_MANIPULATOR_P, CORAL_MANIPULATOR_I, CORAL_MANIPULATOR_D)
            .build();
    }

    public void setCoralPower(double power) {
        if(power != 0.0) {
            coralControllerMotor.pauseFollowerMode();
        } else {
            coralControllerMotor.resumeFollowerMode();
        }

        coralControllerMotor.set(-power);
    }

    public void getEncoderRotation() {
        AbsoluteEncoder encoder = coralControllerMotor.getAbsoluteEncoder();
        encoder.getPosition();
    }

    public void setEncoderRotation(double meters) {
        coralControllerMotor.setReference(CORAL_MOTOR_DISTANCE_FACTOR);
    }

    public SparkAnalogSensor getArmEncoder() {
        return coralControllerMotor.getAnalog();
    }
}

