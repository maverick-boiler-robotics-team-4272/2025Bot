// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coralManipulator.states;


import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.coralManipulator.CoralManipulator;
import frc.robot.utils.commandUtils.State;

public class PositionState extends State<CoralManipulator> {
  double rotation;
  
  public PositionState(CoralManipulator feederSubsystem, double rotation) {
    super(feederSubsystem);
    this.rotation = rotation;
  } 

  @Override
  public void initialize() {
    requiredSubsystem.setWheelRotation(Rotation2d.fromRotations(rotation + requiredSubsystem.getWheelRotation().getRotations()));
  }
}
