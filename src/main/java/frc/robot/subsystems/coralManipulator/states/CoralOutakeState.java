
package frc.robot.subsystems.coralManipulator.states;

import frc.robot.subsystems.coralManipulator.CoralManipulator;
import frc.robot.utils.commandUtils.State;

public class CoralOutakeState extends State<CoralManipulator> {
  public CoralOutakeState(CoralManipulator coralSubsystem) {
    super(coralSubsystem);
  }

  @Override
  public void initialize() {
    requiredSubsystem.setCoralPower(-0.25);
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
