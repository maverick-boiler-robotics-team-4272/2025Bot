package frc.robot.subsystems.drivetrain.states;

import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.utils.commandUtils.State;

public class RobotCentricState extends State<CommandSwerveDrivetrain> {
    private RobotCentric control;

    public RobotCentricState(CommandSwerveDrivetrain drivetrain, double forwardSpeed, double sidewaysSpeeds) {
        super(drivetrain);

        control = new RobotCentric().withVelocityX(sidewaysSpeeds).withVelocityY(forwardSpeed);
    }

    @Override
    public void execute() {
        requiredSubsystem.applyRequest(() -> control);
    }
}
