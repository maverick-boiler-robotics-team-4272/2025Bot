package frc.robot.subsystems.coralManipulator;

import edu.wpi.first.math.geometry.Rotation2d;
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
            .withInversion(true)
            .withCurrentLimit(CURRENT_LIMIT_CORAL)
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

    public void setWheelRotation(Rotation2d rot) {
        coralControllerMotor.setReference(rot.getRotations());
    }

    public Rotation2d getWheelRotation() {
        return Rotation2d.fromRotations(coralControllerMotor.getEncoder().getPosition());
    }

    public SparkAnalogSensor getArmEncoder() {
        return coralControllerMotor.getAnalog();
    }
}

