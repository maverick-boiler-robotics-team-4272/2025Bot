package frc.robot.subsystems.armevator.states;

import frc.robot.subsystems.armevator.Armevator;
import frc.robot.utils.commandUtils.State;

public class GoToNextAlgaePoseState extends State<Armevator> {
    public GoToNextAlgaePoseState(Armevator armevator) {
        super(armevator);
    }

    @Override
    public void initialize() {
        requiredSubsystem.goToNextAlgaePos();
    }

    @Override
    public boolean isFinished() {
        return requiredSubsystem.atDesiredPosition();
    }
}
