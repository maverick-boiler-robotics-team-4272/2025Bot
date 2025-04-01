package frc.robot.commands;

import static frc.robot.constants.positions.ArmevatorPositions.HOME;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.armevator.Armevator;
import frc.robot.subsystems.armevator.states.GoToArmevatorPoseState;
import frc.robot.subsystems.armevator.states.GoToNextArmevatorPoseState;
import frc.robot.subsystems.coralManipulator.CoralManipulator;
import frc.robot.subsystems.coralManipulator.states.CoralOutakeState;
import frc.robot.subsystems.coralManipulator.states.IdleState;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.states.PathfindThenPathState;
import frc.robot.subsystems.drivetrain.states.PathfindingState;
import frc.robot.subsystems.feeder.Feeder;

public class AutoGameCommand extends SequentialCommandGroup {
    public AutoGameCommand(CommandSwerveDrivetrain drivetrain, Armevator armevator, Feeder feeder, CoralManipulator coralManipulator, BooleanSupplier leaveOverride) {
        super(
            new PathfindingState(drivetrain, drivetrain::getNextFeedPose).raceWith(
                new WaitCommand(1).andThen(
                    new FeederManipulatorCommand(feeder, coralManipulator, armevator)
                )
            ),
            new ParallelCommandGroup(
                new FeederManipulatorCommand(feeder, coralManipulator, armevator).andThen(
                    new GoToNextArmevatorPoseState(armevator)
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
            new GoToNextArmevatorPoseState(armevator)
                .raceWith(new IdleState(coralManipulator, armevator::getArmRotation)),
            // new WaitCommand(0.3).unless(() -> !armevator.nextIsL4()),
            new WaitUntilCommand(drivetrain::notRocking),
            new ConditionalCommand(
                new CoralOutakeState(coralManipulator, 0.5).withTimeout(0.25),
                new CoralOutakeState(coralManipulator, -0.5).withTimeout(0.25),
                armevator::nextIsL4
            ),
            // new ConditionalCommand(
            //     new SequentialCommandGroup(
            //         new PathfindThenPathState(drivetrain, drivetrain::getNextAlgaePath)
            //             .alongWith(new GoToArmevatorPosAndGrip(armevator, coralManipulator, HOME)),
            //         new AutoAlgaeCommand(drivetrain, armevator, algaeManipulator)
            //     ), 
                new GoToArmevatorPoseState(armevator, HOME).withTimeout(0.05)//, 
                // drivetrain::getAlgae
            // )
        );
    }
}
