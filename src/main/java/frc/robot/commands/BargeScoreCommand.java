package frc.robot.commands;

import static frc.robot.constants.positions.ArmevatorPositions.BARGE_ARMEVATOR_POSITION;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.algaeManipulator.AlgaeManipulator;
import frc.robot.subsystems.algaeManipulator.states.AlgaeIntake;
import frc.robot.subsystems.algaeManipulator.states.AlgaeOuttake;
import frc.robot.subsystems.armevator.Armevator;
import frc.robot.subsystems.armevator.States.GoToArmevatorPoseState;

public class BargeScoreCommand extends SequentialCommandGroup {
    public BargeScoreCommand(Armevator armevator, AlgaeManipulator algaeManipulator, BooleanSupplier release) {
        super(
            new ParallelCommandGroup(
                new GoToArmevatorPoseState(armevator, BARGE_ARMEVATOR_POSITION),
                new AlgaeIntake(algaeManipulator)
            ).until(release),
            new AlgaeOuttake(algaeManipulator).withTimeout(0.25)
        );

        addRequirements(armevator, algaeManipulator);
    }
}
