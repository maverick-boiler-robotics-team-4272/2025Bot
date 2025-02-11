
package frc.robot.subsystems.climber.states;

import frc.robot.subsystems.climber.Climber;
import frc.robot.utils.commandUtils.State;

public class LowerState extends State<Climber> {
  public LowerState(Climber climberSubsystem) {
    super(climberSubsystem);

  }

  @Override
  public void initialize() {
    requiredSubsystem.setClimberPower(-0.2);
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
