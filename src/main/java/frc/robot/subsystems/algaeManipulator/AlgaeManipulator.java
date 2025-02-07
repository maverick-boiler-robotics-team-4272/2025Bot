package frc.robot.subsystems.algaeManipulator;

// Hardware
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.constants.HardwareMap.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.utils.hardware.Vortex;
import frc.robot.utils.hardware.VortexBuilder;

import static frc.robot.constants.SubsystemConstants.ArmevatorConstants.*;

public class AlgaeManipulator extends SubsystemBase {
    private Vortex algaeControllerMotor;

    public AlgaeManipulator() {
        AbsoluteEncoderConfig encoderConfig = new AbsoluteEncoderConfig()
            .positionConversionFactor(MAV_2_POSITION_FACTOR);


        algaeControllerMotor = VortexBuilder.create(ALGAE_MOTOR_ID)
            .withInversion(false)
            .withCurrentLimit(40)
            .withIdleMode(IdleMode.kBrake)
            .withAbsoluteEncoderConfig(encoderConfig)
            .build();
    }

    public void setAlgaePower(double power) {
        algaeControllerMotor.set(power);
    }

    public AbsoluteEncoder getArmEncoder() {
        return algaeControllerMotor.getAbsoluteEncoder();
    }
}

