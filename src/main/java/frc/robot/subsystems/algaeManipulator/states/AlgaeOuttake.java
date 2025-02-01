
package frc.robot.subsystems.algaeManipulator.states;

import frc.robot.subsystems.algaeManipulator.AlgaeSubsystem;
import frc.robot.utils.commandUtils.State;

public class AlgaeOuttake extends State<AlgaeSubsystem> {
  public AlgaeOuttake(AlgaeSubsystem algaeSubsystem) {
    super(algaeSubsystem);

  }

  @Override
  public void initialize() {
    requiredSubsystem.setAlgaePower(-1);
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
