package frc.robot.commands;

import static frc.robot.constants.positions.ArmevatorPositions.FEEDING_ARMEVATOR_POSITION;

// Commands / States
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.coralManipulator.states.*;
import frc.robot.subsystems.feeder.states.*;
import frc.robot.subsystems.armevator.states.*;

// Subsystems
import frc.robot.subsystems.feeder.*;
import frc.robot.subsystems.coralManipulator.*;
import frc.robot.subsystems.armevator.*;


public class FeederManipulatorCommand extends SequentialCommandGroup {
    public FeederManipulatorCommand(Feeder feed, CoralManipulator coralManipulator, Armevator armevator, double feedPower, double coralPower) {
        super(
            new ParallelRaceGroup(
                new GoToArmevatorPoseState(armevator, FEEDING_ARMEVATOR_POSITION).repeatedly(),
                new SequentialCommandGroup(
                    new ParallelRaceGroup(
                        new FeedState(feed, feedPower),
                        new CoralIntakeState(coralManipulator, coralPower)
                    ).until(feed::lidarBackTripped),
                    new ParallelRaceGroup(
                        new FeedState(feed, 1.0),
                        new CoralIntakeState(coralManipulator, coralPower)
                    ).until(feed::lidarBackNotTripped),
                    new IdleState(coralManipulator, armevator::getArmRotation).withTimeout(0.1)
                )
            )
        );
    }

    public FeederManipulatorCommand(Feeder feed, CoralManipulator coralManipulator, Armevator armevator) {
        this(feed, coralManipulator, armevator, 0.65, 0.1);
    }
}
