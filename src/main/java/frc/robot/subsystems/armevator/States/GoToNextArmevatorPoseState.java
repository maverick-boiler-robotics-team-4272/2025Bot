package frc.robot.subsystems.armevator.States;

import frc.robot.subsystems.armevator.Armevator;
import frc.robot.utils.commandUtils.State;

public class GoToNextArmevatorPoseState extends State<Armevator> {
    public GoToNextArmevatorPoseState(Armevator armevator) {
        super(armevator);
    }

    @Override
    public void initialize() {
        requiredSubsystem.goToNextPos();
    }

    @Override
    public boolean isFinished() {
        return requiredSubsystem.atDesiredPosition();
    }
}
