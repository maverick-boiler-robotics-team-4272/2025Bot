
package frc.robot.subsystems.climber.states;

import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.utils.commandUtils.State;

public class ClimberRotate extends State<ClimberSubsystem> {
  public ClimberRotate(ClimberSubsystem climberSubsystem) {
    super(climberSubsystem);

  }

  @Override
  public void initialize() {
    requiredSubsystem.setClimberPower(1);
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
