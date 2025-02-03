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
import frc.robot.utils.hardware.VortexBuilder;

public class FeederSubsystem extends SubsystemBase {
  private LaserCan feederCanFront;
  private LaserCan feederCanBack;

  public double feederPower;
  public boolean lidarFrontTriggered = false;
  public boolean lidarBackTriggered = false;
  public double lidarFrontDistance;
  public double lidarBackDistance;

  public SparkFlex feederControllerMotor;

  public FeederSubsystem() {
    feederControllerMotor = VortexBuilder.create(FEEDER_MOTOR_ID)
        .withInversion(false)
        .withCurrentLimit(80)
        .withIdleMode(IdleMode.kBrake)
        .build();

    lidar();
  }

  public void setFeederPower(double power) {
    feederControllerMotor.set(power);
  }

  public void lidar() {
    feederCanFront = new LaserCan(0);
    feederCanBack = new LaserCan(0);
    try {
      feederCanFront.setRangingMode(LaserCan.RangingMode.SHORT);
      feederCanFront.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      feederCanFront.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }
    try {
      feederCanBack.setRangingMode(LaserCan.RangingMode.SHORT);
      feederCanBack.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      feederCanBack.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }
  }

  public boolean lidarFrontTripped() {
      return lidarFrontTriggered;
  }
  public boolean lidarBackTripped() {
      return lidarBackTriggered;
  }

  @Override
  public void periodic() {
    LaserCan.Measurement measurementFront = feederCanFront.getMeasurement();
    LaserCan.Measurement measurementBack = feederCanBack.getMeasurement();
    if (measurementFront.distance_mm <= 2) {
      lidarFrontTriggered = true;
    } else {
      lidarFrontTriggered = false;
    }
    if (measurementBack.distance_mm <= 2) {
      lidarBackTriggered = true;
    } else {
      lidarBackTriggered = false;
    }
  }
}
