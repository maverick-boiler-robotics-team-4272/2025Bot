package frc.robot.subsystems.armevator;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.hardware.Vortex;
import frc.robot.utils.hardware.VortexBuilder;
import frc.robot.utils.logging.Loggable;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import frc.robot.constants.positions.ArmevatorPosition;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.constants.HardwareMap.*;
import static frc.robot.constants.SubsystemConstants.ArmevatorConstants.*;

public class Armevator extends SubsystemBase implements Loggable {
    @AutoLog
    public static class ArmevatorInputs {
        public double desiredElevatorHeight;
        public Rotation2d desiredArmRotation;
        public double setElevatorHeight;
        public Rotation2d setArmRotation;
        public double currentElevatorHeight;
        public Rotation2d currentArmRotation;
        public Rotation2d armEncoderRotation;
        public boolean isSafe;
    }
    
    @AutoLogOutput
    private LoggedMechanism2d armevatorMechanism;

    private LoggedMechanismLigament2d armLigament;
    private LoggedMechanismLigament2d elevatorLigament;

    private ArmevatorInputsAutoLogged inputs = new ArmevatorInputsAutoLogged();

    private void initInputs() {
        inputs.desiredArmRotation = SAFE_ANGLE;
        inputs.desiredElevatorHeight = 0.0;
        inputs.setArmRotation = new Rotation2d();
        inputs.setElevatorHeight = 0.0;
        inputs.currentArmRotation = new Rotation2d();
        inputs.currentElevatorHeight = 0.0;
        inputs.armEncoderRotation = new Rotation2d();

        inputs.isSafe = false;

        armevatorMechanism = new LoggedMechanism2d(1, 1);
        armLigament = new LoggedMechanismLigament2d("Arm", 0.5, 0);
        elevatorLigament = new LoggedMechanismLigament2d("Elevator", 1.05344, 90);

        armevatorMechanism.getRoot("armevator", 0.5, 0)
            .append(elevatorLigament)
            .append(armLigament);
    }

    private Vortex elevatorMotor1; 
    @SuppressWarnings("unused")
    private Vortex elevatorMotor2;
    private Vortex armMotor1;
    @SuppressWarnings("unused")
    private Vortex armMotor2;

    private SparkAnalogSensor armEncoder;

    private ArmFeedforward armFeedforward = new ArmFeedforward(0, ARM_FF, 0, 0);

    public Armevator(SparkAnalogSensor armEncoder) {
        elevatorMotor1 = VortexBuilder.create(BASE_ARMEVATOR_MOTOR_1)
            .withVoltageCompensation(NOMINAL_VOLTAGE)
            .withPosition(0)
            .withPositionConversionFactor(ELEVATOR_GEAR_RATIO)
            .withIdleMode(IdleMode.kBrake)
            .withInversion(false)
            .withSoftLimits(MAX_ELEVATOR_HEIGHT, 0)
            .withCurrentLimit(CURRENT_LIMIT_ELEVATOR_MOTORS)
            .withPIDParams(ELEVATOR_P, ELEVATOR_I, ELEVATOR_D)
            .withOutputRange(-0.5, 1.0)
            .maxSpeeds(0.01, 0.001)
            .build();

        elevatorMotor2 = VortexBuilder.create(BASE_ARMEVATOR_MOTOR_2)
            .withVoltageCompensation(NOMINAL_VOLTAGE)
            .withPosition(0)
            .withIdleMode(IdleMode.kBrake)
            .asFollower(elevatorMotor1, false)
            .withCurrentLimit(CURRENT_LIMIT_ELEVATOR_MOTORS)
            .withLimitSwitch()
            .build();

        armMotor1 = VortexBuilder.create(ARM_MOTOR_1)
            .withCurrentLimit(CURRENT_LIMIT_ARM_MOTOR)
            .withIdleMode(IdleMode.kBrake)
            .withInversion(false)
            .withPIDParams(ARM_P, ARM_I, ARM_D)
            .withPositionConversionFactor(ARM_GEAR_RATIO)
            .build();

        armMotor2 = VortexBuilder.create(ARM_MOTOR_2)
            .withCurrentLimit(CURRENT_LIMIT_ARM_MOTOR)
            .withIdleMode(IdleMode.kBrake)
            .asFollower(armMotor1, false)
            .build();

        initInputs();

        this.armEncoder = armEncoder;
    }

    public void goToPos(ArmevatorPosition position) {
        inputs.desiredElevatorHeight = position.getElevatorHeight();
        inputs.desiredArmRotation = position.getArmAngle();
    }

    public void setElevtorHeight(double height){
        elevatorMotor1.setReference(height, ControlType.kPosition, ClosedLoopSlot.kSlot0, ELEVATOR_FF);

        inputs.setElevatorHeight = height;
        elevatorLigament.setLength(height + 1.05344);
    }

    public void setArmRotation(Rotation2d rotation){
        armMotor1.setReference(
            rotation.getRotations(), 
            ControlType.kPosition, 
            ClosedLoopSlot.kSlot0,
            armFeedforward.calculate(
                Rotation2d.fromRotations(armMotor1.getEncoder().getPosition() - 0.25).getRadians(), 
                0
            )
        );

        inputs.setArmRotation = rotation;
        armLigament.setAngle(rotation.getDegrees() - 90);
    }

    public Rotation2d getArmEncoderRotation() {
        return Rotation2d.fromDegrees(armEncoder.getPosition()); //TODO: fix this
    }

    public boolean limitSwitch() {
        return elevatorMotor2.getReverseLimitSwitch().isPressed();
    }

    public void safetyLogic() {
        if(!(elevatorMotor1.getEncoder().getPosition() > Meters.convertFrom(2, Inches) && elevatorMotor1.getEncoder().getPosition() < Meters.convertFrom(20, Inches))) {
            inputs.isSafe = true;
            setArmRotation(inputs.desiredArmRotation);
        } else if(inputs.desiredArmRotation.getDegrees() < SAFE_ANGLE.getDegrees()) {
            inputs.isSafe = false;
            setArmRotation(SAFE_ANGLE);
        } else {
            setArmRotation(inputs.desiredArmRotation);
            inputs.isSafe = true;
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
        inputs.currentArmRotation = Rotation2d.fromRotations(armMotor1.getEncoder().getPosition());
        inputs.currentElevatorHeight = elevatorMotor1.getEncoder().getPosition();
        inputs.armEncoderRotation = getArmEncoderRotation();

        safetyLogic();

        log("Subsystems", "Armevator");
    }
}
