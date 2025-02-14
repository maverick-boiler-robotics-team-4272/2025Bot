package frc.robot.subsystems.coralManipulator;

// Hardware
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.constants.HardwareMap.*;
import static frc.robot.constants.SubsystemConstants.ArmevatorConstants.MAV_POSITION_FACTOR;

import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.config.AnalogSensorConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.utils.hardware.Vortex;
import frc.robot.utils.hardware.VortexBuilder;
import com.revrobotics.*;

public class CoralManipulator extends SubsystemBase {
    private Vortex coralControllerMotor;

    public CoralManipulator() {
        coralControllerMotor = VortexBuilder.create(CORAL_MOTOR_ID)
            .withInversion(false)
            .withCurrentLimit(40)
            .withIdleMode(IdleMode.kBrake)
            .withAnalogConfig(
                new AnalogSensorConfig()
                    .inverted(true)
                    .positionConversionFactor(MAV_POSITION_FACTOR)
            )
            .withPIDParams(0, 0, 0)
            .build();
    }

    public void setCoralPower(double power) {
        coralControllerMotor.set(power);
    }
    public void setCoralRotation(double rotation) {
        coralControllerMotor.setReference(rotation);
    }

    public SparkAnalogSensor getArmEncoder() {
        return coralControllerMotor.getAnalog();
    }
}

