package frc.robot.subsystems.armevator;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.hardware.Vortex;
import frc.robot.utils.hardware.VortexBuilder;

import static frc.robot.constants.HardwareMap.*;
import static frc.robot.constants.SubsystemConstants.ArmevatorConstants.*;
import frc.robot.constants.positions.ArmevatorPosition;

public class Armevator extends SubsystemBase{
    private Vortex elevatorMotor1;    
    private Vortex elevatorMotor2;
    private Vortex armMotor1;
    private Vortex armMotor2;

    private double desiredElevatorHeight;
    private Rotation2d desiredArmRotation;

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
    }

    //TODO: implement method
    public void goToPos(ArmevatorPosition position) {
        desiredElevatorHeight = position.getElevatorHeight();
        desiredArmRotation = position.getArmAngle();
    }

    public void setElevtorHeight(double height){
        elevatorMotor1.setReference(height);
    }

    public void setArmRotation(Rotation2d rotation){
        armMotor1.setReference(rotation.getRadians());
    }

    public void safetyLogic() {
        setElevtorHeight(desiredElevatorHeight);
        setArmRotation(desiredArmRotation);
    }

    @Override
    public void periodic(){
        safetyLogic();
    }
}
