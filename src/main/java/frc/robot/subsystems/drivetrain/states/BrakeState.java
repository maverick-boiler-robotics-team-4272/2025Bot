package frc.robot.subsystems.drivetrain.states;

import com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake;

import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.utils.commandUtils.State;

public class BrakeState extends State<CommandSwerveDrivetrain> {
    private SwerveDriveBrake request;

    public BrakeState(CommandSwerveDrivetrain drivetrain) {
        super(drivetrain);

        request = new SwerveDriveBrake();
    }

    @Override
    public void initialize() {
        requiredSubsystem.setControl(request);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
