// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Check if this version of LaserCan is correct
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;

import static edu.wpi.first.units.Units.Amps;
import static frc.robot.constants.HardwareMap.*;
import static frc.robot.constants.SubsystemConstants.FeederConstants.*;


import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import frc.robot.utils.logging.Loggable;

public class Feeder extends SubsystemBase implements Loggable {
  @AutoLog
  public static class FeederInputs {
    public double frontLidarDistance;
    public double backLidarDistance;
    public double averageBackLidarDistance;
    public double averageFrontLidarDistance;
    public boolean frontLidarIsTripped;
    public boolean backLidarIsTripped;
  }

  FeederInputsAutoLogged inputs = new FeederInputsAutoLogged();

  private void initInputs() {
    inputs.frontLidarDistance = 0.0;
    inputs.backLidarDistance = 0.0;
    inputs.frontLidarIsTripped = false;
    inputs.backLidarIsTripped = false;
  }

  private LaserCan feederCanFront;
  private LaserCan feederCanBack;

  private TalonFX feederControllerMotor;

  private MedianFilter backLidarFilter = new MedianFilter(11);
  private MedianFilter frontLidarFilter = new MedianFilter(5);

  public Feeder() {
    TalonFXConfiguration motorConfiguration = new TalonFXConfiguration()
      .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(60))
                .withStatorCurrentLimitEnable(true)
      )
      .withMotorOutput(
        new MotorOutputConfigs()
          .withNeutralMode(NeutralModeValue.Brake)
          .withInverted(InvertedValue.Clockwise_Positive)
      );

    feederControllerMotor = new TalonFX(FEEDER_MOTOR_ID);
    feederControllerMotor.getConfigurator().apply(motorConfiguration);

    initInputs();

    configLidar();
  }

  public void setFeederPower(double power) {
    feederControllerMotor.set(power);
  }

  public void configLidar() {
    feederCanBack = configFeederCan(FEEDER_CAN_BACK_ID);
    feederCanFront = configFeederCan(FEEDER_CAN_FRONT_ID);
  }

  public LaserCan configFeederCan(int canID) {
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

  public boolean lidarFrontTripped() {
    // TODO: make sure it doesnt die if no lidar
    if (inputs.averageFrontLidarDistance <= FEEDER_CAN_FRONT_TRIGGER_DISTANCE) {
      return true;
    }
    return false;
  }

  public boolean lidarFrontNotTripped() {
    return !lidarFrontTripped();
  }

  public boolean lidarBackTripped() {
    if (inputs.averageBackLidarDistance <= FEEDER_CAN_BACK_TRIGGER_DISTANCE) {
      return true;
    }
    return false;
  }
  
  public boolean lidarBackNotTripped() {
    return !lidarBackTripped();
  }

  @Override
  public void log(String subdirectory, String humanReadableName) {
    Logger.processInputs(subdirectory + "/" + humanReadableName, inputs);
  }

  @Override
  public void periodic() {
    log("Subsystems", "Feeder");

    inputs.frontLidarDistance = feederCanFront.getMeasurement().distance_mm;
    inputs.backLidarDistance = feederCanBack.getMeasurement().distance_mm;
    inputs.averageBackLidarDistance = backLidarFilter.calculate(inputs.backLidarDistance);
    inputs.averageFrontLidarDistance = frontLidarFilter.calculate(inputs.frontLidarDistance);
    inputs.frontLidarIsTripped = lidarFrontTripped();
    inputs.backLidarIsTripped = lidarBackTripped();
  }
}
