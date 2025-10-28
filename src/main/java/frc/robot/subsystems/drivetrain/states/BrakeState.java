package frc.robot.subsystems.drivetrain.states;

import com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake;

import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.utils.commandUtils.State;

public class BrakeState extends State<CommandSwerveDrivetrain> {
    public BrakeState(CommandSwerveDrivetrain drivetrain) {
        super(drivetrain);
    }

    @Override
    public void initialize() {
        requiredSubsystem.setControl(new SwerveDriveBrake());
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
