package frc.robot.subsystems.drivetrain.states;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.utils.commandUtils.State;

public class RobotCentricState extends State<CommandSwerveDrivetrain> {
    private RobotCentric control;

    private double forwardSpeed;
    private double sidewaysSpeed;

    public RobotCentricState(CommandSwerveDrivetrain drivetrain, double forwardSpeed, double sidewaysSpeed) {
        super(drivetrain);

        this.forwardSpeed = forwardSpeed;
        this.sidewaysSpeed = sidewaysSpeed;

        control = new RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage).withDeadband(0.01).withRotationalDeadband(0.001);
    }

    @Override
    public void execute() {
        requiredSubsystem.applyRequest(() -> control.withVelocityX(sidewaysSpeed).withVelocityY(forwardSpeed));
    }
}
