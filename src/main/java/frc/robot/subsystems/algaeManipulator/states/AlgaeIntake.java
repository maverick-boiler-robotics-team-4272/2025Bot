
package frc.robot.subsystems.algaeManipulator.states;

import frc.robot.subsystems.algaeManipulator.AlgaeSubsystem;
import frc.robot.utils.commandUtils.State;
import edu.wpi.first.wpilibj2.command.Command;

public class AlgaeIntake extends State<AlgaeSubsystem> {
  public AlgaeIntake(AlgaeSubsystem algaeSubsystem) {
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
