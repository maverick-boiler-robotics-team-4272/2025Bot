package frc.robot.subsystems.coralManipulator;

import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.constants.FieldConstants.LOG_COUNTER;
import static frc.robot.constants.HardwareMap.*;
import static frc.robot.constants.SubsystemConstants.ArmevatorConstants.MAVCODER_OFFSET;
import static frc.robot.constants.SubsystemConstants.CoralManipulatorConstants.*;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.utils.hardware.Vortex;
import frc.robot.utils.hardware.VortexBuilder;
import frc.robot.utils.logging.Loggable;

public class CoralManipulator extends SubsystemBase implements Loggable {
    @AutoLog
    public static class CoralManipulatorInputs {
        Rotation2d currRotation;
        Rotation2d desiredRotation;
        Rotation2d rotationError;
    }

    CoralManipulatorInputsAutoLogged inputs = new CoralManipulatorInputsAutoLogged();

    private void initInputs() {
        inputs.currRotation = new Rotation2d();
        inputs.desiredRotation = new Rotation2d();
        inputs.rotationError = new Rotation2d();
    }
 
    private Vortex coralControllerMotor;

    public CoralManipulator() {
        coralControllerMotor = VortexBuilder.create(CORAL_MOTOR_ID)
            .withInversion(false)
            .withCurrentLimit(CURRENT_LIMIT_CORAL)
            .withIdleMode(IdleMode.kBrake)
            .withAbsoluteEncoderConfig(
                new AbsoluteEncoderConfig()
                    .zeroOffset(MAVCODER_OFFSET / 360)
                    .inverted(true)
            )
            .withPositionConversionFactor(POSITION_CONVERSION_FACTOR)
            .withPIDParams(CORAL_MANIPULATOR_P, CORAL_MANIPULATOR_I, CORAL_MANIPULATOR_D)
            .positionFrameMs(1)
            .build();

        initInputs();

        System.out.println("Unoffset pose: %.2f".formatted(coralControllerMotor.getAbsoluteEncoder().getPosition() * 360.0 - MAVCODER_OFFSET));
    }

    public void setCoralPower(double power) {
        coralControllerMotor.set(power);
    }

    public void getEncoderRotation() {
        AbsoluteEncoder encoder = coralControllerMotor.getAbsoluteEncoder();
        encoder.getPosition();
    }

    public void setWheelRotation(Rotation2d rot) {
        coralControllerMotor.setReference(rot.getRotations());
        inputs.desiredRotation = rot;
    }

    public void addWheelRotations(Rotation2d rot) {
        setWheelRotation(Rotation2d.fromRotations(rot.getRotations() + getWheelRotation().getRotations()));
    }

    public Rotation2d getWheelRotation() {
        return Rotation2d.fromRotations(coralControllerMotor.getEncoder().getPosition());
    }

    public SparkAbsoluteEncoder getArmEncoder() {
        return coralControllerMotor.getAbsoluteEncoder();
    }

    public Rotation2d getWheelError() {
        return Rotation2d.fromRotations(inputs.desiredRotation.getRotations() - inputs.currRotation.getRotations());
    }

    @Override
    public void log(String subdirectory, String humanReadableName) {
        Logger.processInputs(subdirectory + "/" + humanReadableName, inputs);

        coralControllerMotor.log(subdirectory + "/" + humanReadableName, "CoralMotor");
    }

    @Override
    public void periodic() {
        if(LOG_COUNTER % 20 == 0) {
            log("Subsystems", "CoralManipulator");
        }
        inputs.currRotation = getWheelRotation();
        inputs.rotationError = inputs.desiredRotation.minus(inputs.currRotation);
    }
}

