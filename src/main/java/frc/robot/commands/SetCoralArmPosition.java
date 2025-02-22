package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
// Commands / States
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.coralManipulator.states.*;
import frc.robot.subsystems.armevator.states.*; 

// Subsystems
import frc.robot.subsystems.coralManipulator.*;
import frc.robot.constants.positions.ArmevatorPositions.ArmevatorPosition;
import frc.robot.subsystems.armevator.*;


public class SetCoralArmPosition extends SequentialCommandGroup {
    public SetCoralArmPosition(PositionState positionState, CoralManipulator coralManipulator, Armevator armAngle, double coralPower, ArmevatorPosition armPosition, double coralPosition) {
        super(
            new ParallelRaceGroup(
                new PositionState(coralManipulator, coralPosition),
                new GoToArmevatorPoseState(armAngle, armPosition),
                new CoralOutakeState(coralManipulator, coralPower)
            )
        );
    }
}
