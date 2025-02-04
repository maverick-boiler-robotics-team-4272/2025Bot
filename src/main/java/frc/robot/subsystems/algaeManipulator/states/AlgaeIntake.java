
package frc.robot.subsystems.algaeManipulator.states;

import frc.robot.subsystems.algaeManipulator.AlgaeManipulator;
import frc.robot.utils.commandUtils.State;

public class AlgaeIntake extends State<AlgaeManipulator> {
  public AlgaeIntake(AlgaeManipulator algaeSubsystem) {
    super(algaeSubsystem);

  }

  @Override
  public void initialize() {
    requiredSubsystem.setAlgaePower(1);
  }

  @Override
  public void end(boolean interrupted) {
    requiredSubsystem.setAlgaePower(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
