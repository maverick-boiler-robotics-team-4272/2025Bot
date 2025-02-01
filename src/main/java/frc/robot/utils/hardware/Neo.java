package frc.robot.utils.hardware;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkMax;

import frc.robot.utils.logging.Loggable;

public class Neo extends SparkMax implements Loggable {
    @AutoLog
    public static class NeoInputs {
        public double current;
        public double temperature;
        public double rotations;
        public double volts;
    }

    private NeoInputsAutoLogged inputs = new NeoInputsAutoLogged();

    public Neo(int id) {
        super(id, MotorType.kBrushless);
    }

    public void setReference(double reference, ControlType controlType) {
        getClosedLoopController().setReference(reference, controlType);
    }

    public void setReference(double reference) {
        setReference(reference, ControlType.kPosition);
    }

    @Override
    public void log(String subdirectory, String humanReadableName) {
        Logger.processInputs(subdirectory + "/" + humanReadableName, inputs);
    }
}
