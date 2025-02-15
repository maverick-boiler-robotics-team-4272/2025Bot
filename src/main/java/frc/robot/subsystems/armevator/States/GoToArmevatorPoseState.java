// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.armevator.states;

import frc.robot.constants.positions.ArmevatorPositions.ArmevatorPosition;
import frc.robot.subsystems.armevator.Armevator;
import frc.robot.utils.commandUtils.State;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoToArmevatorPoseState extends State<Armevator> {
  /** Creates a new GoToPos. */
  private ArmevatorPosition position;
   
  public GoToArmevatorPoseState(Armevator armevator, ArmevatorPosition position) {
    super(armevator);
    this.position = position;
  }

  @Override
  public void initialize() {
    requiredSubsystem.goToPos(position);
  }
}
