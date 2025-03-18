
package frc.robot.subsystems.climber.states;

import au.grapplerobotics.LaserCan;
import frc.robot.subsystems.climber.Climber;
import frc.robot.utils.commandUtils.State;

public class ClimbLidarState extends State<Climber> {
  public ClimbLidarState(Climber climberSubsystem) {
    super(climberSubsystem);
  }

  @Override
  public void initialize() {
    requiredSubsystem.setClimberPower(0.5);
    if (requiredSubsystem.climberLidarTripped() == true) {
      requiredSubsystem.setClimberPower(0);
    } else {
      requiredSubsystem.setClimberPower(0.5);
    }
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
