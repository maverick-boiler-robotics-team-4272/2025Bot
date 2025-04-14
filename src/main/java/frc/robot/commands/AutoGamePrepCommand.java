package frc.robot.commands;

import static frc.robot.constants.positions.ArmevatorPositions.HOME;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.SubsystemConstants.DrivetrainConstants.AutoConstants;
import frc.robot.subsystems.algaeManipulator.AlgaeManipulator;
import frc.robot.subsystems.armevator.Armevator;
import frc.robot.subsystems.armevator.states.GoToArmevatorPoseState;
import frc.robot.subsystems.armevator.states.GoToNextArmevatorPoseState;
import frc.robot.subsystems.coralManipulator.CoralManipulator;
import frc.robot.subsystems.coralManipulator.states.CoralOutakeState;
import frc.robot.subsystems.coralManipulator.states.IdleState;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.states.PathfindThenPathState;
import frc.robot.subsystems.feeder.Feeder;

// TODO: Clean up with comments

public class AutoGamePrepCommand extends SequentialCommandGroup {
    public AutoGamePrepCommand(CommandSwerveDrivetrain drivetrain, Armevator armevator, Feeder feeder, CoralManipulator coralManipulator, AlgaeManipulator algaeManipulator) {
        super(
            new ConditionalCommand(
                new PathfindThenPathState(drivetrain, drivetrain::getNextMiddlePath, AutoConstants.LIMITED_TRANSLATION, AutoConstants.LIMITED_TRANSLATION_A), 
                new PathfindThenPathState(drivetrain, drivetrain::getNextPath, AutoConstants.LIMITED_TRANSLATION, AutoConstants.LIMITED_TRANSLATION_A), 
                armevator::nextIsL1
            ),
            new GoToNextArmevatorPoseState(armevator)
                .raceWith(new IdleState(coralManipulator, armevator::getArmRotation)),
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
            new GoToArmevatorPoseState(armevator, HOME).withTimeout(0.1).unless(() -> !armevator.nextIsL4())
        );
    }
}
