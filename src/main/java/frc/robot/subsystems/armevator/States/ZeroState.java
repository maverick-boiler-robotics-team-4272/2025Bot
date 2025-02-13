package frc.robot.subsystems.armevator.States;

import frc.robot.subsystems.armevator.Armevator;
import frc.robot.utils.commandUtils.State;

public class ZeroState extends State<Armevator> {
    public ZeroState(Armevator armevator) {
        super(armevator);
    }
    @Override
    public void initialize() {
       requiredSubsystem.setElevatotPower(-0.1);
    }

    @Override
    public boolean isFinished() {
        return requiredSubsystem.isLimitSwitchHit();
    }

    @Override
    public void end(boolean interrupted) {
        requiredSubsystem.setElevatotPower(0);
        requiredSubsystem.resetElevator(0);
    }
}
