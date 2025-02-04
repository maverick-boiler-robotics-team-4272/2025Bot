package frc.robot.subsystems.armevator;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
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
import static frc.robot.constants.HardwareMap.*;
import static frc.robot.constants.SubsystemConstants.ArmevatorConstants.*;

public class Armevator extends SubsystemBase implements Loggable {
    @AutoLog
    public static class ArmevatorInputs {
        public double desiredElevatorHeight;
        public Rotation2d desiredArmRotation;
        public double setElevatorHeight;
        public Rotation2d setArmRotation;
    }
    
    @AutoLogOutput
    private LoggedMechanism2d armevatorMechanism;

    private LoggedMechanismLigament2d armLigament;
    private LoggedMechanismLigament2d elevatorLigament;

    private ArmevatorInputsAutoLogged inputs = new ArmevatorInputsAutoLogged();

    private void initInputs() {
        inputs.desiredArmRotation = new Rotation2d();
        inputs.desiredElevatorHeight = 0.0;
        inputs.setArmRotation = new Rotation2d();
        inputs.setElevatorHeight = 0.0;

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

    @SuppressWarnings("unused")
    private AbsoluteEncoder armEncoder;

    public Armevator(AbsoluteEncoder armEncoder) {
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

        initInputs();

        this.armEncoder = armEncoder;
    }

    public void goToPos(ArmevatorPosition position) {
        inputs.desiredElevatorHeight = position.getElevatorHeight();
        inputs.desiredArmRotation = position.getArmAngle();
    }

    public void setElevtorHeight(double height){
        elevatorMotor1.setReference(height);

        inputs.setElevatorHeight = height;
        elevatorLigament.setLength(height + 1.05344);
    }

    public void setArmRotation(Rotation2d rotation){
        armMotor1.setReference(rotation.getRadians());

        inputs.setArmRotation = rotation;
        armLigament.setAngle(rotation.getDegrees() - 90);
    }

    public void safetyLogic() {
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

        log("Subsystems", "Armevator");
    }
}
