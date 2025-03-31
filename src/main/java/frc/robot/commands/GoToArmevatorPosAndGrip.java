package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.constants.positions.ArmevatorPositions.ArmevatorPosition;
import frc.robot.subsystems.armevator.Armevator;
import frc.robot.subsystems.armevator.states.GoToArmevatorPoseState;
import frc.robot.subsystems.armevator.states.GoToNextArmevatorPoseState;
import frc.robot.subsystems.coralManipulator.CoralManipulator;
import frc.robot.subsystems.coralManipulator.states.IdleState;

public class GoToArmevatorPosAndGrip extends ParallelRaceGroup {
    public GoToArmevatorPosAndGrip(Armevator armevator, CoralManipulator coralManipulator, ArmevatorPosition armevatorPosition) {
        super(
            new GoToArmevatorPoseState(armevator, armevatorPosition),
            new IdleState(coralManipulator, armevator::getArmRotation)
        );
    }

    public GoToArmevatorPosAndGrip(Armevator armevator, CoralManipulator coralManipulator) {
        super(
            new GoToNextArmevatorPoseState(armevator),
            new IdleState(coralManipulator, armevator::getArmRotation)
        );
    }
}
