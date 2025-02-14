// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coralManipulator.states;

import frc.robot.subsystems.coralManipulator.CoralManipulator;
import frc.robot.subsystems.coralManipulator.states.*;
import frc.robot.utils.commandUtils.State;

public class PositionState extends State<CoralManipulator> {
  
  public PositionState(CoralManipulator feederSubsystem) {
    super(feederSubsystem);
  }

  @Override
  public void initialize() {
    requiredSubsystem.setCoralRotation(3);
  }

  @Override
  public void end(boolean interrupted) {
    
  }
}
