package frc.robot.subsystems.algaeManipulator;

// Hardware
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.constants.HardwareMap.*;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.utils.hardware.Vortex;
import frc.robot.utils.hardware.VortexBuilder;

public class AlgaeManipulator extends SubsystemBase {
    private Vortex algaeControllerMotor;

    public AlgaeManipulator() {
        algaeControllerMotor = VortexBuilder.create(ALGAE_MOTOR_ID)
            .withInversion(true)
            .withCurrentLimit(40)
            .withIdleMode(IdleMode.kBrake)
            .build();
    }

    public void setAlgaePower(double power) {
        algaeControllerMotor.set(power);
    }
}

