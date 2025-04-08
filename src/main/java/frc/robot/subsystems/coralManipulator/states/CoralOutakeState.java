
package frc.robot.subsystems.coralManipulator.states;

import frc.robot.subsystems.coralManipulator.CoralManipulator;
import frc.robot.utils.commandUtils.State;

public class CoralOutakeState extends State<CoralManipulator> {
  double coralPower;

  public CoralOutakeState(CoralManipulator coralSubsystem, double power) {
    super(coralSubsystem);
    this.coralPower = power;
  }

  @Override
  public void initialize() {
    requiredSubsystem.setCoralPower(-coralPower);
  }

  @Override
  public void end(boolean interrupted) {
    requiredSubsystem.setCoralPower(-coralPower);
  }
}
