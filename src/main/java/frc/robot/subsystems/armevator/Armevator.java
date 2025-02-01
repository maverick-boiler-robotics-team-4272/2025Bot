package frc.robot.subsystems.armevator;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.hardware.Vortex;
import frc.robot.utils.hardware.VortexBuilder;
import frc.robot.utils.logging.Loggable;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import frc.robot.constants.positions.ArmevatorPosition;

import static frc.robot.constants.HardwareMap.*;
import static frc.robot.constants.SubsystemConstants.ArmevatorConstants.*;

public class Armevator extends SubsystemBase implements Loggable {
    @AutoLog
    public static class ArmevatorInputs {
        public double desiredElevatorHeight;
        public Rotation2d desiredArmRotation;
        public double setElevatorHeight;
        public Rotation2d setArmRotation;

        // public LoggedMechanism2d armevatorMechanism;
    }

    ArmevatorInputsAutoLogged inputs = new ArmevatorInputsAutoLogged();

    private Vortex elevatorMotor1; 
    @SuppressWarnings("unused")
    private Vortex elevatorMotor2;
    private Vortex armMotor1;
    @SuppressWarnings("unused")
    private Vortex armMotor2;

   //TODO: private MAVCoder2 armMAVCoder;

    public Armevator() {
        elevatorMotor1 = VortexBuilder.create(BASE_ARMEVATOR_MOTOR_1)
            .withVoltageCompensation(NOMINAL_VOLTAGE)
            .withPosition(0)
            .withPositionConversionFactor(ELEVATOR_GEAR_RATIO)
            .withIdleMode(IdleMode.kBrake)
            .withInversion(false)
            .withCurrentLimit(CURRENT_LIMIT_ELEVATOR_MOTORS)
            .build();

        elevatorMotor2 = VortexBuilder.create(BASE_ARMEVATOR_MOTOR_2)
            .withVoltageCompensation(NOMINAL_VOLTAGE)
            .withPosition(0)
            .withIdleMode(IdleMode.kBrake)
            .asFollower(elevatorMotor1, true)
            .withCurrentLimit(CURRENT_LIMIT_ELEVATOR_MOTORS)
            .build();

        armMotor1 = VortexBuilder.create(ARM_MOTOR_1)
            .withCurrentLimit(CURRENT_LIMIT_ARM_MOTOR)
            .withIdleMode(IdleMode.kBrake)
            .withInversion(true)
            .build();

        armMotor2 = VortexBuilder.create(ARM_MOTOR_2)
            .withCurrentLimit(CURRENT_LIMIT_ARM_MOTOR)
            .withIdleMode(IdleMode.kBrake)
            .asFollower(armMotor1, true)
            .build();

            // inputs.armevatorMechanism = new LoggedMechanism2d(5, 5);
    }

    public void goToPos(ArmevatorPosition position) {
        inputs.desiredElevatorHeight = position.getElevatorHeight();
        inputs.desiredArmRotation = position.getArmAngle();
    }

    public void setElevtorHeight(double height){
        elevatorMotor1.setReference(height);

        inputs.setElevatorHeight = height;
        // inputs.armevatorMechanism
        //     .getRoot("Armevator", 0, 0)
        //     .append(new LoggedMechanismLigament2d("Elevator", height, 90));
    }

    public void setArmRotation(Rotation2d rotation){
        armMotor1.setReference(rotation.getRadians());

        inputs.setArmRotation = rotation;
        // inputs.armevatorMechanism
        //     .getRoot("Armevator", 0, 0)
        //     .append(new LoggedMechanismLigament2d("Arm", 1.0, rotation.getDegrees() - 90));
    }

    public void safetyLogic() {
        // double x = ARM_LENGTH * Math.sin(inputs.desiredArmRotation.minus(POINT_TO_PIVOT_OFFSET).getRadians());
        // double y = inputs.desiredElevatorHeight + BASE_OF_STAGE_TO_PIVOT - ARM_LENGTH * Math.cos(inputs.desiredArmRotation.minus(POINT_TO_PIVOT_OFFSET).getRadians());

        // if(
        //     y < BoundingBox.Y + 0.5 * BoundingBox.HEIGHT &&
        //     y > BoundingBox.Y - 0.5 * BoundingBox.HEIGHT &&
        //     x < BoundingBox.X + 0.5 * BoundingBox.WIDTH &&
        //     x > BoundingBox.X - 0.5 * BoundingBox.WIDTH
        // ) {
        //     // TODO: find safe angle and height
        // } // Maybe math

        if(!(elevatorMotor1.getEncoder().getPosition() > 2 && elevatorMotor1.getEncoder().getPosition() < 20)) {
            setArmRotation(inputs.desiredArmRotation);
        } else if(inputs.desiredArmRotation.getDegrees() < SAFE_ANGLE.getDegrees()) {
            setArmRotation(SAFE_ANGLE);
        } else {
            setArmRotation(inputs.desiredArmRotation);
        }

        setElevtorHeight(inputs.desiredElevatorHeight);
        setArmRotation(inputs.desiredArmRotation);
    }

    @Override
    public void log(String subdirectory, String humanReadableName) {
        Logger.processInputs(subdirectory + "/" + humanReadableName, inputs);
    }

    @Override
    public void periodic(){
        safetyLogic();
    }
}
