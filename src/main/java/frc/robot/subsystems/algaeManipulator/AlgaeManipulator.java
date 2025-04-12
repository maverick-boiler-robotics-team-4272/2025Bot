package frc.robot.subsystems.algaeManipulator;

import edu.wpi.first.math.filter.MedianFilter;
// Hardware
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.constants.HardwareMap.*;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.utils.hardware.Vortex;
import frc.robot.utils.hardware.VortexBuilder;
import frc.robot.utils.logging.Loggable;

public class AlgaeManipulator extends SubsystemBase implements Loggable {
    @AutoLog
    public static class AlgaeManipulatorInputs {
        public boolean hasAlgae;
    }

    AlgaeManipulatorInputsAutoLogged inputs = new AlgaeManipulatorInputsAutoLogged();

    private void initInputs() {
        inputs.hasAlgae = false;
    }

    private Vortex algaeControllerMotor;

    private MedianFilter algMedianFilter = new MedianFilter(11);

    public AlgaeManipulator() {
        algaeControllerMotor = VortexBuilder.create(ALGAE_MOTOR_ID)
            .withInversion(true)
            .withCurrentLimit(40)
            .withIdleMode(IdleMode.kBrake)
            .build();

        initInputs();
    }

    public void setAlgaePower(double power) {
        algaeControllerMotor.set(power);
    }

    public double getCurrent() {
        return algaeControllerMotor.getOutputCurrent();
    }

    public boolean hasAlgae() {
        return algMedianFilter.calculate(getCurrent()) > 29;
    }

    @Override
    public void log(String subdirectory, String humanReadableName) {
        algaeControllerMotor.log(subdirectory + "/" + humanReadableName, "AlgaeMotor");
    }

    @Override
    public void periodic() {
        inputs.hasAlgae = hasAlgae();

        log("Subsystems", "AlgaeManipulator");
    }
}

