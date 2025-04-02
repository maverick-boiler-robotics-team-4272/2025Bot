package frc.robot.commands;

import static frc.robot.constants.positions.ArmevatorPositions.*;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.armevator.Armevator;
import frc.robot.subsystems.armevator.States.GoToArmevatorPoseState;
import frc.robot.subsystems.coralManipulator.CoralManipulator;
import frc.robot.subsystems.coralManipulator.states.CoralOutakeState;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.states.PathfindThenPathState;
import frc.robot.subsystems.drivetrain.states.PathfindingState;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.states.FeedState;

public class AutoGameCommand extends SequentialCommandGroup {
    public AutoGameCommand(CommandSwerveDrivetrain drivetrain, Armevator armevator, Feeder feeder, CoralManipulator coralManipulator, BooleanSupplier leaveOverride) {
        super(
            new PathfindingState(drivetrain, drivetrain::getNextFeedPose).raceWith(
                new WaitCommand(1).andThen(
                    new FeedState(feeder).alongWith(
                        new GoToArmevatorPoseState(armevator, HOME)
                    )
                )
            ),
            new ParallelCommandGroup(
                new SequentialCommandGroup( 
                    new FeederManipulatorCommand(feeder, coralManipulator, armevator),
                    new ConditionalCommand(
                        // new GoToArmevatorPosAndGrip(armevator, coralManipulator, L4_PREP_POSITION), 
                        new PrintCommand("Hello :)"),
                        new GoToArmevatorPosAndGrip(armevator, coralManipulator), 
                        armevator::nextIsL4
                    )
                ),
                new ConditionalCommand(
                    new PathfindThenPathState(drivetrain, drivetrain::getNextMiddlePath).beforeStarting(
                        new ParallelRaceGroup(
                            new WaitUntilCommand(leaveOverride),
                            new WaitUntilCommand(feeder::lidarBackTripped),
                            new WaitUntilCommand(feeder::lidarFrontTripped)
                        )
                    ), 
                    new PathfindThenPathState(drivetrain, drivetrain::getNextPath).beforeStarting(
                        new ParallelRaceGroup(
                            new WaitUntilCommand(leaveOverride),
                            new WaitUntilCommand(feeder::lidarBackTripped),
                            new WaitUntilCommand(feeder::lidarFrontTripped)
                        )
                    ),
                    armevator::nextIsL1
                )
                   
            ),
            new GoToArmevatorPosAndGrip(armevator, coralManipulator),
            new WaitCommand(0.3).unless(() -> !armevator.nextIsL4()),
            new ConditionalCommand(
                new CoralOutakeState(coralManipulator, 1.0).withTimeout(0.2),
                new ConditionalCommand(
                    new CoralOutakeState(coralManipulator, -0.2).withTimeout(0.25), 
                    new CoralOutakeState(coralManipulator, -1.0).withTimeout(0.1), 
                    armevator::nextIsL1
                ),
                armevator::nextIsL4
            ),
            // new ConditionalCommand(
            //     new SequentialCommandGroup(
            //         new PathfindThenPathState(drivetrain, drivetrain::getNextAlgaePath)
            //             .alongWith(new GoToArmevatorPosAndGrip(armevator, coralManipulator, HOME)),
            //         new AutoAlgaeCommand(drivetrain, armevator, algaeManipulator)
            //     ), 
                new GoToArmevatorPoseState(armevator, HOME).withTimeout(0.05).unless(() -> !armevator.nextIsL4())//, 
                // drivetrain::getAlgae
            // )
        );
    }
}
