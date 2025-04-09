package frc.robot.commands;

import static frc.robot.constants.positions.ArmevatorPositions.BARGE_ARMEVATOR_POSITION;
import static frc.robot.constants.positions.ArmevatorPositions.BARGE_PREP_ARMEVATOR_POSITION;
import static frc.robot.constants.positions.ArmevatorPositions.HOME;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.SubsystemConstants.DrivetrainConstants.AutoConstants;
import frc.robot.subsystems.algaeManipulator.AlgaeManipulator;
import frc.robot.subsystems.algaeManipulator.states.AlgaeIntake;
import frc.robot.subsystems.algaeManipulator.states.AlgaeOuttake;
import frc.robot.subsystems.armevator.Armevator;
import frc.robot.subsystems.armevator.states.GoToArmevatorPoseState;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.states.BrakeState;
import frc.robot.subsystems.drivetrain.states.PathfindingState;

public class AutoAlgaeCommand extends SequentialCommandGroup {
    public AutoAlgaeCommand(CommandSwerveDrivetrain drivetrain, Armevator armevator, AlgaeManipulator algaeManipulator) {
        super(
            new SequentialCommandGroup (
                new PathfindingState(drivetrain, drivetrain::getNextBargePose, AutoConstants.LIMITED_TRANSLATION, AutoConstants.LIMITED_TRANSLATION_A),
                new BrakeState(drivetrain),
                new GoToArmevatorPoseState(armevator, BARGE_PREP_ARMEVATOR_POSITION),
                // new WaitCommand(.05),
                new GoToArmevatorPoseState(armevator, BARGE_ARMEVATOR_POSITION)
            ).raceWith(new AlgaeIntake(algaeManipulator)),
            new AlgaeOuttake(algaeManipulator),
            new GoToArmevatorPoseState(armevator, HOME).withTimeout(0.05)
        );
    }
}
