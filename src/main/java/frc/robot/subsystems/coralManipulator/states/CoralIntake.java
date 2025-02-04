
package frc.robot.subsystems.coralManipulator.states;

import frc.robot.subsystems.coralManipulator.CoralManipulator;
import frc.robot.utils.commandUtils.State;

public class CoralIntake extends State<CoralManipulator> {
  public CoralIntake(CoralManipulator coralSubsystem) {
    super(coralSubsystem);

  }

  @Override
  public void initialize() {
    requiredSubsystem.setCoralPower(1);
  }

  @Override
  public void end(boolean interrupted) {
    requiredSubsystem.setCoralPower(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
