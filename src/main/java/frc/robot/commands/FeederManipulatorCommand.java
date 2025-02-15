package frc.robot.commands;

// Commands / States
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.coralManipulator.states.*;
import frc.robot.subsystems.feeder.states.*;
import frc.robot.subsystems.armevator.states.*;

// Subsystems
import frc.robot.subsystems.feeder.*;
import frc.robot.subsystems.coralManipulator.*;
import frc.robot.constants.positions.ArmevatorPositions.ArmevatorPosition;
import frc.robot.subsystems.armevator.*;


public class FeederManipulatorCommand extends SequentialCommandGroup {
    public FeederManipulatorCommand(Feeder feed, CoralManipulator coralManipulator, Armevator armevator, double feedPower, double coralPower, ArmevatorPosition position) {
        super(
            new ParallelRaceGroup(
                new GoToArmevatorPoseState(armevator, position),
                new SequentialCommandGroup(
                new ParallelRaceGroup(
                    new FeedState(feed, feedPower),
                    new CoralIntakeState(coralManipulator, coralPower)
                ).until(feed::lidarBackTripped),
                new ParallelRaceGroup(
                    new FeedState(feed, feedPower),
                    new CoralIntakeState(coralManipulator, coralPower)
                ).until(feed::lidarBackNotTripped)
                )
            ).until(feed::lidarBackNotTripped)
        );
    }
}
