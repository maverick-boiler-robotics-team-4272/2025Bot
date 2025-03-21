package frc.robot.commands;

import static frc.robot.constants.positions.ArmevatorPositions.HOME;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.armevator.Armevator;
import frc.robot.subsystems.armevator.states.GoToArmevatorPoseState;
import frc.robot.subsystems.armevator.states.GoToNextArmevatorPoseState;
import frc.robot.subsystems.coralManipulator.CoralManipulator;
import frc.robot.subsystems.coralManipulator.states.CoralOutakeState;
import frc.robot.subsystems.coralManipulator.states.IdleState;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.states.PathfindThenPathState;
import frc.robot.subsystems.feeder.Feeder;

public class AutoGamePrepCommand extends SequentialCommandGroup {
    public AutoGamePrepCommand(CommandSwerveDrivetrain drivetrain, Armevator armevator, Feeder feeder, CoralManipulator coralManipulator) {
        super(
            new PathfindThenPathState(drivetrain, drivetrain::getNextPath),
            new GoToNextArmevatorPoseState(armevator)
                .raceWith(new IdleState(coralManipulator, armevator::getArmRotation)),
            new WaitCommand(0.3),
            new ConditionalCommand(
                new CoralOutakeState(coralManipulator, 0.5).withTimeout(0.25),
                new CoralOutakeState(coralManipulator, -0.5).withTimeout(0.25),
                armevator::nextIsL4
            ),
            new GoToArmevatorPoseState(armevator, HOME).withTimeout(0.1)
        );
    }
}
