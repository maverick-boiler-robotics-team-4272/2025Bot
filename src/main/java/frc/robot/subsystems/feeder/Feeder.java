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

public class Feeder extends SubsystemBase {
  private LaserCan feederCanFront;
  private LaserCan feederCanBack;

  public double feederPower;
  public boolean lidarFrontTriggered = false;
  public boolean lidarBackTriggered = false;
  public double lidarFrontDistance;
  public double lidarBackDistance;

  public SparkFlex feederControllerMotor;

  public Feeder() {
    feederControllerMotor = VortexBuilder.create(FEEDER_MOTOR_ID)
        .withInversion(true)
        .withCurrentLimit(40)
        .withIdleMode(IdleMode.kBrake)
        .build();

    configLidar();
  }

  public void setFeederPower(double power) {
    feederControllerMotor.set(power);
  }

  public void configLidar() {
    // configFeederCan(FEEDER_CAN_BACK_ID);
    // configFeederCan(FEEDER_CAN_FRONT_ID);
  }

  public void configFeederCan(int canID) {
    LaserCan feederCan = new LaserCan(canID);
    try {
      feederCan.setRangingMode(LaserCan.RangingMode.SHORT);
      feederCan.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      feederCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }
  }

  public boolean lidarFrontTripped() {
      LaserCan.Measurement measurementFront = feederCanFront.getMeasurement();
      if (measurementFront.distance_mm <= FEEDER_CAN_FRONT_TRIGGER_DISTANCE) {
          lidarFrontTriggered = true;
        } else {
          lidarFrontTriggered = false;
        }
      return lidarFrontTriggered;
  }

  public boolean lidarBackTripped() {
      LaserCan.Measurement measurementBack = feederCanBack.getMeasurement();
      if (measurementBack.distance_mm <= FEEDER_CAN_BACK_TRIGGER_DISTANCE) {
        lidarBackTriggered = true;
        } else {
       lidarBackTriggered = false;
        }
      return lidarBackTriggered;
  }

  @Override
  public void periodic() {
    //log?
  }
}
