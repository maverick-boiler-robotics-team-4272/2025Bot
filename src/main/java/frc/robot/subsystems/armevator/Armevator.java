package frc.robot.subsystems.armevator;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.hardware.Vortex;
import frc.robot.utils.hardware.VortexBuilder;

import static frc.robot.constants.HardwareMap.*;
import static frc.robot.constants.SubsystemConstants.ArmevatorConstants.*;

public class Armevator extends SubsystemBase{
    private Vortex baseArmevatorMotor1;    
    private Vortex baseArmevtorMotor2;
    private Vortex armMotor;

    public Armevator() {
        baseArmevatorMotor1 = VortexBuilder.create(BASE_ARMEVATOR_MOTOR_1)
        .withVoltageCompensation(NOMINAL_VOLTAGE)
        .withPosition(0)
        .withIdleMode(IdleMode.kBrake)
        .withInversion(false)
        .withCurrentLimit(CURRENT_LIMIT_ELEVATOR_MOTORS)
        .build();

        baseArmevtorMotor2 = VortexBuilder.create(BASE_ARMEVATOR_MOTOR_2)
            .withVoltageCompensation(NOMINAL_VOLTAGE)
            .withPosition(0)
            .withIdleMode(IdleMode.kBrake)
            .withInversion(false)
            .withCurrentLimit(CURRENT_LIMIT_ELEVATOR_MOTORS)
            .build();

        armMotor = VortexBuilder.create(ARM_MOTOR)
            .withCurrentLimit(CURRENT_LIMIT_ARM_MOTOR)
            .withIdleMode(IdleMode.kBrake)
            .withInversion(true)
            .withCurrentLimit(CURRENT_LIMIT_ARM_MOTOR)
            .build();

    }
    public void up(double height){
        baseArmevatorMotor1.set(height);
        baseArmevtorMotor2.set(height);
    }

    public void arm(double speed){
        armMotor.set(speed);
    }
    @Override
    public void periodic(){

    }
}
