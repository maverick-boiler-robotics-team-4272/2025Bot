package frc.robot.subsystems.climber;

import frc.robot.constants.HardwareMap;
import frc.robot.utils.hardware.Vortex;
import frc.robot.utils.hardware.VortexBuilder;

import static frc.robot.constants.HardwareMap.CLIMBER_MOTOR_ID;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
    private Vortex climberControllerMotor;
    private AbsoluteEncoder climberEncoder;

    public ClimberSubsystem() {
        climberControllerMotor = VortexBuilder.create(CLIMBER_MOTOR_ID)
            .withInversion(false)
            .withCurrentLimit(80)
            .withIdleMode(IdleMode.kBrake)
            .build();

        climberEncoder = climberControllerMotor.getAbsoluteEncoder();
    }

    public void setClimberPower(double power) {
        climberControllerMotor.set(power);
    }
}
