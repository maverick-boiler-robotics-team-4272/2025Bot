package frc.robot.commands;

import static frc.robot.constants.positions.ArmevatorPositions.*;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.SubsystemConstants.DrivetrainConstants.AutoConstants;
import frc.robot.subsystems.algaeManipulator.AlgaeManipulator;
import frc.robot.subsystems.armevator.Armevator;
import frc.robot.subsystems.armevator.states.GoToArmevatorPoseState;
import frc.robot.subsystems.coralManipulator.CoralManipulator;
import frc.robot.subsystems.coralManipulator.states.CoralOutakeState;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.states.BrakeState;
import frc.robot.subsystems.drivetrain.states.PathfindThenPathState;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.states.FeedState;

// TODO: Clean up with comments

public class AutoGameCommand extends SequentialCommandGroup {
    public AutoGameCommand(CommandSwerveDrivetrain drivetrain, Armevator armevator, Feeder feeder, CoralManipulator coralManipulator, AlgaeManipulator algaeManipulator, BooleanSupplier leaveOverride) {
        super(
            new FollowPathWithCustomTolerance(
                drivetrain.getNextFeedPath(), 
                drivetrain::getPose, 
                drivetrain::getCurrentSpeeds, 
                drivetrain::driveWithChassisSpeeds, 
                new PPHolonomicDriveController(new PIDConstants(AutoConstants.TRANSLATION_P, AutoConstants.TRANSLATION_I, AutoConstants.TRANSLATION_D), new PIDConstants(AutoConstants.ROTATION_P, AutoConstants.ROTATION_I, AutoConstants.ROTATION_D)), 
                new RobotConfig(drivetrain.getKinematics(), new PIDConstants(AutoConstants.TRANSLATION_P, AutoConstants.TRANSLATION_I, AutoConstants.TRANSLATION_D), new PIDConstants(AutoConstants.ROTATION_P, AutoConstants.ROTATION_I, AutoConstants.ROTATION_D)), //TODO: do drivetrain Mass and MOI thingy
                () -> false, 
                drivetrain.getNextFeedPose(), 
                0.01, 
                drivetrain),
            new PathfindThenPathState(drivetrain, drivetrain::getNextFeedPath).raceWith(
                new WaitCommand(0.5).andThen(
                    new FeedState(feeder).alongWith(
                        new GoToArmevatorPoseState(armevator, HOME)
                    )
                )
            ),
            new BrakeState(drivetrain),
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
                    new PathfindThenPathState(
                        drivetrain, 
                        drivetrain::getNextMiddlePath,
                        AutoConstants.LIMITED_TRANSLATION, 
                        AutoConstants.LIMITED_TRANSLATION_A
                    ).beforeStarting(
                        new ParallelRaceGroup(
                            new WaitUntilCommand(leaveOverride),
                            new WaitUntilCommand(feeder::lidarBackTripped),
                            new WaitUntilCommand(feeder::lidarFrontTripped)
                        )
                    ), 
                    new PathfindThenPathState(
                        drivetrain, 
                        drivetrain::getNextPath,
                        AutoConstants.LIMITED_TRANSLATION, 
                        AutoConstants.LIMITED_TRANSLATION_A
                    ).beforeStarting(
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
            new GoToArmevatorPoseState(armevator, HOME).withTimeout(0.05).unless(() -> !armevator.nextIsL4())//, 
        );
    }
}
