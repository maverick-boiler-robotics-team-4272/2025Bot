package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.hardware.Vortex;

public class Manipulator extends SubsystemBase {
     private final Vortex motor;

    public Manipulator() {

    private Inputs inputs = new Inputs();

    private ManiMotor maniMotor = new DropperMotor();

    public void setPower(double power) {
        inputs.maniPower = power;
        maniMotor.set(power);
    }

    private class Inputs {
        public double maniMotor;
    }

    private class DropperMotor {
        public void set(double power) {
        }
        
    }
}
