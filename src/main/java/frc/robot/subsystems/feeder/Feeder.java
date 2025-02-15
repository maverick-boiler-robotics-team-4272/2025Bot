// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Check if this version of LaserCan is correct
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;

import static frc.robot.constants.HardwareMap.*;
import static frc.robot.constants.SubsystemConstants.*;


import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import frc.robot.utils.hardware.VortexBuilder;
import frc.robot.utils.logging.Loggable;

public class Feeder extends SubsystemBase implements Loggable {
  @AutoLog
  public static class FeederInputs {
    public double frontLidarDistance;
    public double backLidarDistance;
  }

  FeederInputsAutoLogged inputs = new FeederInputsAutoLogged();

  private void initInputs() {
    inputs.frontLidarDistance = 0.0;
    inputs.backLidarDistance = 0.0;
  }

  private LaserCan feederCanFront;
  private LaserCan feederCanBack;

  public SparkFlex feederControllerMotor;

  public Feeder() {
    feederControllerMotor = VortexBuilder.create(FEEDER_MOTOR_ID)
        .withInversion(true)
        .withCurrentLimit(40)
        .withIdleMode(IdleMode.kBrake)
        .build();

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
    LaserCan.Measurement measurementFront = feederCanFront.getMeasurement();
    if (measurementFront.distance_mm <= FEEDER_CAN_FRONT_TRIGGER_DISTANCE) {
      return true;
    }
    return false;
  }

  public boolean lidarBackTripped() {
    LaserCan.Measurement measurementBack = feederCanBack.getMeasurement();
    if (measurementBack.distance_mm <= FEEDER_CAN_BACK_TRIGGER_DISTANCE) {
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

    inputs.frontLidarDistance = feederCanBack.getMeasurement().distance_mm;
    inputs.backLidarDistance = feederCanFront.getMeasurement().distance_mm;
  }
}
