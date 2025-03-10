
package frc.robot.subsystems.coralManipulator.states;

import frc.robot.subsystems.coralManipulator.CoralManipulator;
import frc.robot.utils.commandUtils.State;

public class CoralOutakeState extends State<CoralManipulator> {
  double coral;
  public CoralOutakeState(CoralManipulator coralSubsystem, double coral) {
    super(coralSubsystem);
    this.coral = coral;
  }

  @Override
  public void initialize() {
    requiredSubsystem.setCoralPower(-coral);
  }

  @Override
  public void end(boolean interrupted) {
    requiredSubsystem.setCoralPower(-coral);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
