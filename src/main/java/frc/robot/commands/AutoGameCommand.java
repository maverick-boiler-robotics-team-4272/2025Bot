package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.armevator.Armevator;
import frc.robot.subsystems.coralManipulator.CoralManipulator;
import frc.robot.subsystems.coralManipulator.states.CoralOutakeState;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.states.PathfindThenPathState;
import frc.robot.subsystems.drivetrain.states.PathfindingState;
import frc.robot.subsystems.feeder.Feeder;

public class AutoGameCommand extends SequentialCommandGroup {
    public AutoGameCommand(CommandSwerveDrivetrain drivetrain, Armevator armevator, Feeder feeder, CoralManipulator coralManipulator) {
        super(
            new PathfindThenPathState(drivetrain, drivetrain::getNextPath),
            new CoralOutakeState(coralManipulator, 0.5).withTimeout(0.25),
            new PathfindingState(drivetrain, drivetrain::getNextFeedPose),
            new FeederManipulatorCommand(feeder, coralManipulator, armevator, 1.0, 0.2)
        );
    }
}
