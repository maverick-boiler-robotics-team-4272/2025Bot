// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder.states;

import frc.robot.subsystems.feeder.Feeder;
import frc.robot.utils.commandUtils.State;

public class FeedLidarState extends State<Feeder> {
  
  public FeedLidarState(Feeder feederSubsystem) {
    super(feederSubsystem);
  }

  @Override
  public void initialize() {
    requiredSubsystem.setFeederPower(1);
  }

  @Override
  public void end(boolean interrupted) {
    requiredSubsystem.setFeederPower(1);
  }
}
