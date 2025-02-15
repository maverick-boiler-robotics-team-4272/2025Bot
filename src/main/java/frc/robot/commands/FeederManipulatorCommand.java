package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PIDCommand;
// Commands / States
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.coralManipulator.states.*;
import frc.robot.subsystems.feeder.states.*;

// Subsystems
import frc.robot.subsystems.feeder.*;
import frc.robot.subsystems.coralManipulator.*;


public class FeederManipulatorCommand extends SequentialCommandGroup {
    public FeederManipulatorCommand(Feeder feed, CoralManipulator coralManipulator, double feedPower, double coralPower, double rotation) {
        super(
            new ParallelRaceGroup(
                new FeedState(feed, feedPower),
                new CoralIntakeState(coralManipulator, coralPower)
            ).until(feed::lidarBackTripped),
            new ParallelRaceGroup(
                new FeedState(feed, feedPower),
                new CoralIntakeState(coralManipulator, coralPower)
            ).until(feed::lidarBackNotTripped)            
        );
    }
}
