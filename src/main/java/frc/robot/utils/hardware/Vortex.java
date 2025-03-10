package frc.robot.utils.hardware;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;

import frc.robot.utils.logging.Loggable;

public class Vortex extends SparkFlex implements Loggable {
    @AutoLog
    public static class VortexInputs {
        public double current;
        public double temperature;
        public double rotations;
        public double volts;
    }

    VortexInputsAutoLogged inputs = new VortexInputsAutoLogged();

    public Vortex(int id) {
        super(id, MotorType.kBrushless);
    }

    public void setReference(double reference, ControlType controlType, ClosedLoopSlot slot, double ff) {
        getClosedLoopController().setReference(reference, controlType, slot, ff, ArbFFUnits.kPercentOut);
    }

    public void setReference(double reference, ControlType controlType) {
        getClosedLoopController().setReference(reference, controlType);
    }

    public void setReference(double reference) {
        setReference(reference, ControlType.kPosition);
    }

    public void log(String subdirectory, String humanReadableName) {
        inputs.current = getOutputCurrent();
        inputs.rotations = getEncoder().getPosition();
        inputs.temperature = getMotorTemperature();
        inputs.volts = getBusVoltage();

        Logger.processInputs(subdirectory + "/" + humanReadableName, inputs);
    }
}
