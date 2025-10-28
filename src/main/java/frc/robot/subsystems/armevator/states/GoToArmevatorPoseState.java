// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.armevator.states;

import frc.robot.constants.positions.ArmevatorPositions.ArmevatorPosition;
import frc.robot.subsystems.armevator.Armevator;
import frc.robot.utils.commandUtils.State;

public class GoToArmevatorPoseState extends State<Armevator> {
  private ArmevatorPosition position;
   
  public GoToArmevatorPoseState(Armevator armevator, ArmevatorPosition position) {
    super(armevator);
    this.position = position;
  }

  @Override
  public void initialize() {
    requiredSubsystem.goToPos(position);
  }

  @Override
  public boolean isFinished() {
      return requiredSubsystem.atDesiredPosition();
  }
}
