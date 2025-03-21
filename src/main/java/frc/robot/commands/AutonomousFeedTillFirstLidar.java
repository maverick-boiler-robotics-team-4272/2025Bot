package frc.robot.commands;

import static frc.robot.constants.positions.ArmevatorPositions.FEEDING_ARMEVATOR_POSITION;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.armevator.Armevator;
import frc.robot.subsystems.armevator.states.GoToArmevatorPoseState;
import frc.robot.subsystems.coralManipulator.CoralManipulator;
import frc.robot.subsystems.coralManipulator.states.CoralIntakeState;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.states.FeedState;

public class AutonomousFeedTillFirstLidar extends SequentialCommandGroup{
    public AutonomousFeedTillFirstLidar(Feeder feed, CoralManipulator coralManipulator, Armevator armevator, double feedPower, double coralPower) {
        super(
            new ParallelRaceGroup(
                new GoToArmevatorPoseState(armevator, FEEDING_ARMEVATOR_POSITION).repeatedly(),
                new SequentialCommandGroup(
                    new ParallelRaceGroup(
                        new FeedState(feed, feedPower),
                        new CoralIntakeState(coralManipulator, coralPower)
                    ).until(() -> (feed.lidarFrontTripped() || feed.lidarBackTripped()))
                )
            )
        );
    }
}
