package frc.robot.subsystems.climber;

import static frc.robot.constants.HardwareMap.CLIMBER_MOTOR_ID;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.hardware.Vortex;
import frc.robot.utils.hardware.VortexBuilder;
import frc.robot.utils.logging.Loggable;

import static frc.robot.constants.SubsystemConstants.ClimberConstants.*;

public class Climber extends SubsystemBase implements Loggable {
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

    @Override
    public void log(String subdirectory, String humanReadableName) {
        climberControllerMotor.log(subdirectory + "/" + humanReadableName, "CLimberMotor");
    }

    @Override
    public void periodic() {
        log("Subsystems", "Climber");
    }
}
