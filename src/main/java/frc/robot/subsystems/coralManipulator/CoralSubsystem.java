package frc.robot.subsystems.coralManipulator;

// Hardware
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.constants.HardwareMap.*;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.utils.hardware.Vortex;
import frc.robot.utils.hardware.VortexBuilder;

public class CoralSubsystem extends SubsystemBase {
    private SparkFlex coralControllerMotor;
    final int CORAL_MOTOR_ID = 2;
    final int NOMINAL_VOLTAGE = 12;

    public CoralSubsystem() {
        coralControllerMotor = VortexBuilder.create(CORAL_MOTOR_ID)
            .withVoltageCompensation(NOMINAL_VOLTAGE)
            .withInversion(false)
            .withCurrentLimit(80)
            .withIdleMode(IdleMode.kBrake)
            .build();
    }

    public void setCoralPower(double power) {
        coralControllerMotor.set(power);
    }
}

