
package frc.robot.subsystems.algaeManipulator.states;

import frc.robot.subsystems.algaeManipulator.AlgaeManipulator;
import frc.robot.utils.commandUtils.State;

public class AlgaeIdle extends State<AlgaeManipulator> {
  public AlgaeIdle(AlgaeManipulator algaeSubsystem) {
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
