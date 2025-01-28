package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.hardware.Vortex;

public class Manipulator extends SubsystemBase {
    public final Vortex motor1;
    public final Vortex motor2;

    public Manipulator() {
        motor1 = new Vortex(1);
        motor2 = new Vortex(2);
    }

    public void setPower(double power) {
        motor1.set(power);
        motor2.set(power);
    }


}

