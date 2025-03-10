package frc.robot.subsystems.armevator.states;

import frc.robot.subsystems.armevator.Armevator;
import frc.robot.utils.commandUtils.State;

public class ZeroState extends State<Armevator> {
    public ZeroState(Armevator armevator) {
        super(armevator);
    }

    @Override
    public void initialize() {
        requiredSubsystem.disableSaftey();
        requiredSubsystem.setElevatorPower(-0.3);
    }

    @Override
    public boolean isFinished() {
        return requiredSubsystem.isLimitSwitchHit();
    }

    @Override
    public void end(boolean interrupted) {
        requiredSubsystem.setElevatorPower(0);
        requiredSubsystem.resetElevator(0);
        requiredSubsystem.enableSaftey();
    }
}
