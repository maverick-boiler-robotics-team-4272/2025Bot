package frc.robot.subsystems.algaeManipulator;

// Hardware
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.constants.HardwareMap.*;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.utils.hardware.Vortex;
import frc.robot.utils.hardware.VortexBuilder;
import frc.robot.utils.logging.Loggable;

public class AlgaeManipulator extends SubsystemBase implements Loggable {
    private Vortex algaeControllerMotor;

    public AlgaeManipulator() {
        algaeControllerMotor = VortexBuilder.create(ALGAE_MOTOR_ID)
            .withInversion(true)
            .withCurrentLimit(30)
            .withIdleMode(IdleMode.kBrake)
            .build();
    }

    public void setAlgaePower(double power) {
        algaeControllerMotor.set(power);
    }

    public double getCurrent() {
        return algaeControllerMotor.getOutputCurrent();
    }

    public boolean hasAlgae() {
        // return getCurrent() > 40;
        return false;
    }

    @Override
    public void log(String subdirectory, String humanReadableName) {
        algaeControllerMotor.log(subdirectory + "/" + humanReadableName, "AlgaeMotor");
    }

    @Override
    public void periodic() {
        log("Subsystems", "AlgaeManipulator");
    }
}

