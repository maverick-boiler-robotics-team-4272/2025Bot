
package frc.robot.subsystems.algaeManipulator.states;

import frc.robot.subsystems.algaeManipulator.AlgaeSubsystem;
import frc.robot.utils.commandUtils.State;

public class AlgaeIdle extends State<AlgaeSubsystem> {
  public AlgaeIdle(AlgaeSubsystem algaeSubsystem) {
    super(algaeSubsystem);

  }

  @Override
  public void initialize() {
    requiredSubsystem.setAlgaePower(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
