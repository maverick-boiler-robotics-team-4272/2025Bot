package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static frc.robot.constants.HardwareMap.CLIMBER_MOTOR_ID;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.constants.SubsystemConstants.ClimberConstants.*;

public class Climber extends SubsystemBase {
    private TalonFX climberControllerMotor;

    public Climber() {
        TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
        climberControllerMotor = new TalonFX(CLIMBER_MOTOR_ID);
        climberControllerMotor.getConfigurator().apply(motorConfiguration
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(CLIMBER_CURRENT_LIMIT))
                .withStatorCurrentLimitEnable(true)
            )
        );

        climberControllerMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void setClimberPower(double power) {
        climberControllerMotor.set(power);
    }
}
