// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder.states;

import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.utils.commandUtils.State;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederIntake extends State<FeederSubsystem> {
  
  public FeederIntake(FeederSubsystem feederSubsystem) {
    super(feederSubsystem);
  }

  @Override
  public void initialize() {
    requiredSubsystem.setFeederPower(1);
  }

  @Override
  public void end(boolean interrupted) {
    requiredSubsystem.setFeederPower(0);
  }

  @Override
  public boolean isFinished() {
    return requiredSubsystem.lidarTripped();
  }
  
}
