package frc.robot.commands;

// Commands / States
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.coralManipulator.states.*;
import frc.robot.subsystems.feeder.states.*;

// Subsystems
import frc.robot.subsystems.feeder.*;
import frc.robot.subsystems.coralManipulator.*;


public class FeederManipulatorCommand extends SequentialCommandGroup {
    public FeederManipulatorCommand(FeederSubsystem feed, double power) {
        super(
            new ParallelRaceGroup(
                new FeedState(feed, power)
            ).until(feed::lidarFrontTripped)
        );
    }
}
