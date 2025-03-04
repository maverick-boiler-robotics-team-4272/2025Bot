package frc.robot.subsystems.climber;

import static frc.robot.constants.HardwareMap.CLIMBER_LIDAR_ID;
import static frc.robot.constants.HardwareMap.CLIMBER_MOTOR_ID;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.feeder.FeederInputsAutoLogged;
import frc.robot.utils.hardware.Vortex;
import frc.robot.utils.hardware.VortexBuilder;
import frc.robot.utils.logging.Loggable;

import static frc.robot.constants.SubsystemConstants.ClimberConstants.*;

import org.littletonrobotics.junction.AutoLog;

public class Climber extends SubsystemBase implements Loggable {
    @AutoLog
    public static class ClimberInputs {
        public double climberLidarDistance;
    }
    private Vortex climberControllerMotor;

    private LaserCan climberCan;

    public Climber() {
        climberControllerMotor = VortexBuilder.create(CLIMBER_MOTOR_ID)
            .withCurrentLimit(CLIMBER_CURRENT_LIMIT)
            .withIdleMode(IdleMode.kBrake)
            .build();
    }
    
    ClimberInputsAutoLogged inputs = new ClimberInputsAutoLogged();

    public void configClimberLidar() {
        climberCan = configClimberCan(CLIMBER_LIDAR_ID);
      }

      public LaserCan configClimberCan(int canID) {
        LaserCan laserCan = new LaserCan(canID);
        try {
          laserCan.setRangingMode(LaserCan.RangingMode.SHORT);
          laserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
          laserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
          System.out.println("Configuration failed! " + e);
        }
    
        return laserCan;
      }
    
      public boolean climberLidarTripped() {
        LaserCan.Measurement climberMeasurement = climberCan.getMeasurement();
        if (climberMeasurement.distance_mm <= CLIMBER_LIDAR_TRIGGER_DISTANCE) {
          return true;
        }
        return false;
      }

    public void setClimberPower(double power) {
        climberControllerMotor.set(power);
    }

    @Override
    public void log(String subdirectory, String humanReadableName) {
        climberControllerMotor.log(subdirectory + "/" + humanReadableName, "CLimberMotor");
    }

    @Override
    public void periodic() {
        log("Subsystems", "Climber");

        inputs.climberLidarDistance = climberCan.getMeasurement().distance_mm;
    }
}
