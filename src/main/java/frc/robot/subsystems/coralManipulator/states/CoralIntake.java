
package frc.robot.subsystems.coralManipulator.states;

import frc.robot.subsystems.coralManipulator.CoralSubsystem;
import frc.robot.utils.commandUtils.State;

public class CoralIntake extends State<CoralSubsystem> {
  public CoralIntake(CoralSubsystem coralSubsystem) {
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
