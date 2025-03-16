
package frc.robot.subsystems.climber.states;

import frc.robot.subsystems.climber.Climber;
import frc.robot.utils.commandUtils.State;

public class ClimbState extends State<Climber> {
  public ClimbState(Climber climberSubsystem) {
    super(climberSubsystem);

  }

  @Override
  public void initialize() {
    requiredSubsystem.setClimberPower(0.5);
  }

  @Override
  public void end(boolean interrupted) {
    requiredSubsystem.setClimberPower(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
