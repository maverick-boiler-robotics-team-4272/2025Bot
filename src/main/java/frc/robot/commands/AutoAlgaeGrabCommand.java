package frc.robot.commands;

import static frc.robot.constants.positions.ArmevatorPositions.*;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.SubsystemConstants.DrivetrainConstants.AutoConstants;
import frc.robot.subsystems.algaeManipulator.AlgaeManipulator;
import frc.robot.subsystems.algaeManipulator.states.AlgaeIntake;
import frc.robot.subsystems.armevator.Armevator;
import frc.robot.subsystems.armevator.states.GoToArmevatorPoseState;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.states.PathfindingState;
import frc.robot.subsystems.drivetrain.states.RobotCentricState;

public class AutoAlgaeGrabCommand extends SequentialCommandGroup {
    public AutoAlgaeGrabCommand(CommandSwerveDrivetrain drivetrain, Armevator armevator, AlgaeManipulator algaeManipulator) {
        super(
            new ParallelCommandGroup(
                new GoToArmevatorPoseState(armevator, HOME),
                new PathfindingState(drivetrain, drivetrain::getNearestAlgae, AutoConstants.LIMITED_TRANSLATION, AutoConstants.LIMITED_TRANSLATION_A)
            ),
            new ParallelCommandGroup(
                new ConditionalCommand(
                    new GoToArmevatorPoseState(armevator, ALGAE_ARMEVATOR_POSITION_TWO), 
                    new GoToArmevatorPoseState(armevator, ALGAE_ARMEVATOR_POSITION),
                    drivetrain::nextAlgaeHigh
                ),
                new AlgaeIntake(algaeManipulator).until(algaeManipulator::hasAlgae)
            ),
            // new WaitCommand(0.2),
            new RobotCentricState(drivetrain, -0.2, 0).withTimeout(0.5)
                .raceWith(new AlgaeIntake(algaeManipulator))
        );
    }
}
