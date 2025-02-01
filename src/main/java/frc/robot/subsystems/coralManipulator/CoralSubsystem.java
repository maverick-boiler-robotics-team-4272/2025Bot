package frc.robot.subsystems.coralManipulator;

// Hardware
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.constants.HardwareMap.*;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.utils.hardware.Vortex;
import frc.robot.utils.hardware.VortexBuilder;

public class CoralSubsystem extends SubsystemBase {
    private Vortex coralControllerMotor;

    public CoralSubsystem() {
        coralControllerMotor = VortexBuilder.create(CORAL_MOTOR_ID)
            .withInversion(false)
            .withCurrentLimit(80)
            .withIdleMode(IdleMode.kBrake)
            .build();
    }

    public void setCoralPower(double power) {
        coralControllerMotor.set(power);
    }
}

