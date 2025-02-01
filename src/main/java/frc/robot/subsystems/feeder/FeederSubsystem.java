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
  private LaserCan lc;

  public double feederPower;
  public boolean lidarTriggered = false;
  public double lidarDistance;

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
    lc = new LaserCan(0);
    try {
      lc.setRangingMode(LaserCan.RangingMode.SHORT);
      lc.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }
  }

  public boolean lidarTripped() {
      return lidarTriggered;

  }

  @Override
  public void periodic() {
    LaserCan.Measurement measurement = lc.getMeasurement();
    if (measurement.distance_mm <= 2) {
      lidarTriggered = true;
    } else {
      lidarTriggered = false;
    }
  }
}
