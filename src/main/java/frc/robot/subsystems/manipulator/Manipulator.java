package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.hardware.Vortex;

public class Manipulator extends SubsystemBase {
    private final Vortex motor;

    public Manipulator() {
        motor = new Vortex(0);

    }

    public void setSpeed(double speed) {
        motor.set(speed);
    }

    public void stop() {
        motor.stopMotor();
    }
    
}
