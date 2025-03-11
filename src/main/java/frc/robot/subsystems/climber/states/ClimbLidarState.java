package frc.robot.subsystems.climber.states;

public class ClimbLidarState {

import frc.robot.subsystems.climber.Climber;
import frc.robot.utils.commandUtils.State;

  public FeedLidarState(Feeder feederSubsystem) {
    super(feederSubsystem);
  }

  @Override
  public void initialize() {
    requiredSubsystem.setFeederPower(1);
  }

  @Override
  public void end(boolean interrupted) {
    requiredSubsystem.setFeederPower(1);
  }
}
