package frc.robot.commands;

import static frc.robot.constants.positions.ArmevatorPositions.*;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.algaeManipulator.AlgaeManipulator;
import frc.robot.subsystems.algaeManipulator.states.AlgaeIntake;
import frc.robot.subsystems.armevator.Armevator;
import frc.robot.subsystems.armevator.states.GoToArmevatorPoseState;
import frc.robot.subsystems.armevator.states.GoToNextAlgaePoseState;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.states.PathfindingState;
import frc.robot.subsystems.drivetrain.states.RobotCentricState;

public class AutoAlgaeGrabCommand extends SequentialCommandGroup {
    public AutoAlgaeGrabCommand(CommandSwerveDrivetrain drivetrain, Armevator armevator, AlgaeManipulator algaeManipulator) {
        super(
            new ParallelCommandGroup(
                new GoToArmevatorPoseState(armevator, HOME),
                new PathfindingState(drivetrain, drivetrain::getNearestAlgae)
            ),
            new GoToNextAlgaePoseState(armevator)
                .raceWith(new AlgaeIntake(algaeManipulator)),
            new RobotCentricState(drivetrain, -0.3, 0).withTimeout(0.25)
        );
    }
}
