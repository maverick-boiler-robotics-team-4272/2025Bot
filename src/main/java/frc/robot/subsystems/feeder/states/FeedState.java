// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder.states;

import frc.robot.subsystems.feeder.Feeder;
import frc.robot.utils.commandUtils.State;

public class FeedState extends State<Feeder> {

  double power;
  
  public FeedState(Feeder feederSubsystem, double power) {
    super(feederSubsystem);
    this.power = power;
  }

  public FeedState(Feeder feeder) {
    this(feeder, 0.5);
  }

  @Override
  public void initialize() {
    requiredSubsystem.setFeederPower(power);
  }

  @Override
  public void end(boolean interrupted) {
    requiredSubsystem.setFeederPower(0);
  }
}
