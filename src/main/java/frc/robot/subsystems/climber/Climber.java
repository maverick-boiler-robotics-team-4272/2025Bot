package frc.robot.subsystems.climber;

import static frc.robot.constants.HardwareMap.CLIMBER_MOTOR_ID;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.hardware.Vortex;
import frc.robot.utils.hardware.VortexBuilder;

import static frc.robot.constants.SubsystemConstants.ClimberConstants.*;

public class Climber extends SubsystemBase {
    private Vortex climberControllerMotor;

    public Climber() {
        climberControllerMotor = VortexBuilder.create(CLIMBER_MOTOR_ID)
            .withCurrentLimit(CLIMBER_CURRENT_LIMIT)
            .withIdleMode(IdleMode.kBrake)
            .build();
    }

    public void setClimberPower(double power) {
        climberControllerMotor.set(power);
    }
}
