package frc.robot.subsystems.armevator;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
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

import frc.robot.constants.positions.ArmevatorPositions.ArmevatorPosition;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.constants.HardwareMap.*;
import static frc.robot.constants.SubsystemConstants.ArmevatorConstants.*;
import static frc.robot.constants.positions.ArmevatorPositions.L2_ARMEVATOR_POSITION;
import static frc.robot.constants.positions.ArmevatorPositions.L4_ARMEVATOR_POSITION;

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
        public boolean limitSwitch;
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
        inputs.limitSwitch = false;

        armevatorMechanism = new LoggedMechanism2d(1, 1);
        armLigament = new LoggedMechanismLigament2d("Arm", 0.5, 0);
        elevatorLigament = new LoggedMechanismLigament2d("Elevator", 1.05344, 90);

        armevatorMechanism.getRoot("armevator", 0.5, 0)
            .append(elevatorLigament)
            .append(armLigament);
    }

    private Vortex elevatorMotor1; 
    private Vortex elevatorMotor2;
    private Vortex armMotor1;
    private Vortex armMotor2;

    private boolean disableSaftey = false;

    private SparkAbsoluteEncoder armEncoder;

    private ArmFeedforward armFeedforward = new ArmFeedforward(0, ARM_FF, 0, 0);

    private ArmevatorPosition nextPose = L2_ARMEVATOR_POSITION;

    public Armevator(SparkAbsoluteEncoder armEncoder) {
        this.armEncoder = armEncoder;

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
            .withLimitSwitch(
                new LimitSwitchConfig()
                    .reverseLimitSwitchType(Type.kNormallyOpen)
                    .reverseLimitSwitchEnabled(true)
            )
            .build();

        elevatorMotor2 = VortexBuilder.create(BASE_ARMEVATOR_MOTOR_2)
            .withVoltageCompensation(NOMINAL_VOLTAGE)
            .withPosition(0)
            .withIdleMode(IdleMode.kBrake)
            .asFollower(elevatorMotor1, false)
            .withCurrentLimit(CURRENT_LIMIT_ELEVATOR_MOTORS)
            .build();

        armMotor1 = VortexBuilder.create(ARM_MOTOR_1)
            .withCurrentLimit(CURRENT_LIMIT_ARM_MOTOR)
            .withIdleMode(IdleMode.kBrake)
            .withInversion(false)
            .withPIDParams(ARM_P, ARM_I, ARM_D)
            .withPositionConversionFactor(ARM_GEAR_RATIO)
            .withOutputRange(-0.6, 0.75)
            .withPosition(getArmEncoderRotation().getRotations())
            .build();

        armMotor2 = VortexBuilder.create(ARM_MOTOR_2)
            .withCurrentLimit(CURRENT_LIMIT_ARM_MOTOR)
            .withIdleMode(IdleMode.kBrake)
            .asFollower(armMotor1, false)
            .build();

        initInputs();
    }

    public void goToPos(ArmevatorPosition position) {
        inputs.desiredElevatorHeight = position.getElevatorHeight();
        inputs.desiredArmRotation = position.getArmAngle();
    }

    public void goToPosNext(ArmevatorPosition position) {
        nextPose = position;
    }

    public void goToNextPos() {
        goToPos(nextPose);
    }

    public boolean nextIsL4() {
        return nextPose == L4_ARMEVATOR_POSITION;
    }

    private void setElevtorHeight(double height){
        elevatorMotor1.setReference(height, ControlType.kPosition, ClosedLoopSlot.kSlot0, ELEVATOR_FF);

        inputs.setElevatorHeight = height;
        elevatorLigament.setLength(height + 1.05344);
    }
    
    public void setElevatorPower(double speed) {
        elevatorMotor1.set(speed);
        elevatorMotor2.set(speed);
    }

    public void resetElevator(double position) {
        elevatorMotor1.getEncoder().setPosition(0);
    }

    private void setArmRotation(Rotation2d rotation){
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
        return Rotation2d.fromRotations(armEncoder.getPosition() - 0.25);
    }

    public Rotation2d getArmRotation() {
        return Rotation2d.fromRotations(armMotor1.getEncoder().getPosition());
    }

    public double getElevatorHeight() {
        return elevatorMotor1.getEncoder().getPosition();
    }

    public boolean isLimitSwitchHit() {
        return elevatorMotor2.getReverseLimitSwitch().isPressed();
    }

    public void disableSaftey() {
        disableSaftey = true;
    }

    public void enableSaftey() {
        disableSaftey = false;
    }

    public boolean atDesiredPosition() {
        if(
            Math.abs(getElevatorHeight() - inputs.desiredElevatorHeight) < 0.05 &&
            Math.abs(getArmRotation().getDegrees() - inputs.desiredArmRotation.getDegrees()) < 5.0
        ) {
            return true;
        }

        return false;
    }

    public void safetyLogic() {
        if(disableSaftey) {
            return;
        }

        if(getArmRotation().getDegrees() < SAFE_ANGLE.getDegrees() - 5.0 && inputs.desiredElevatorHeight <= SAFE_ELEVATOR_HEIGHT && inputs.desiredElevatorHeight < getElevatorHeight() - 0.01) {
            inputs.isSafe = false;
            setElevtorHeight(SAFE_ELEVATOR_HEIGHT);
            setArmRotation(SAFE_ANGLE);
            return;
        }
        
        if(!(getElevatorHeight() > Meters.convertFrom(0.1, Inches) && getElevatorHeight() < SAFE_ELEVATOR_HEIGHT)) {
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
    }

    @Override
    public void log(String subdirectory, String humanReadableName) {
        Logger.processInputs(subdirectory + "/" + humanReadableName, inputs);

        elevatorMotor1.log(subdirectory + "/" + humanReadableName, "elevatorMotor1");
        elevatorMotor2.log(subdirectory + "/" + humanReadableName, "elevatorMotor2");
        armMotor1.log(subdirectory + "/" + humanReadableName, "armMotor1");
        armMotor2.log(subdirectory + "/" + humanReadableName, "armMotor2");
    }

    @Override
    public void periodic(){
        inputs.currentArmRotation = Rotation2d.fromRotations(armMotor1.getEncoder().getPosition());
        inputs.currentElevatorHeight = elevatorMotor1.getEncoder().getPosition();
        inputs.armEncoderRotation = getArmEncoderRotation();
        inputs.limitSwitch = isLimitSwitchHit();

        safetyLogic();

        log("Subsystems", "Armevator");
    }
}
