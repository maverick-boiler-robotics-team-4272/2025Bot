
package frc.robot.subsystems.coralManipulator.states;

import frc.robot.subsystems.coralManipulator.CoralManipulator;
import frc.robot.utils.commandUtils.State;

public class CoralKeepState extends State<CoralManipulator> {
  double coralPower;

  public CoralKeepState(CoralManipulator coralSubsystem, double power) {
    super(coralSubsystem);
    coralPower = power;
  }

  @Override
  public void initialize() {
    requiredSubsystem.setCoralPower(-coralPower);
  }

  @Override
  public void end(boolean interrupted) {
    requiredSubsystem.setCoralPower(-coralPower);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
