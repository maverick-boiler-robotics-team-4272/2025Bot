package frc.robot.subsystems.algaeManipulator;

// Hardware
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.constants.HardwareMap.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.utils.hardware.Vortex;
import frc.robot.utils.hardware.VortexBuilder;

public class AlgaeSubsystem extends SubsystemBase {
    private Vortex algaeControllerMotor;

    public AlgaeSubsystem() {
        algaeControllerMotor = VortexBuilder.create(ALGAE_MOTOR_ID)
            .withInversion(false)
            .withCurrentLimit(80)
            .withIdleMode(IdleMode.kBrake)
            .build();
    }

    public void setAlgaePower(double power) {
        algaeControllerMotor.set(power);
    }

    public AbsoluteEncoder getArmEncoder() {
        return algaeControllerMotor.getAbsoluteEncoder();
    }
}

