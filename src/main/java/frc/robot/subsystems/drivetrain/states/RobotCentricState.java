package frc.robot.subsystems.drivetrain.states;

import static frc.robot.constants.SubsystemConstants.DrivetrainConstants.TeleConstants.MAX_TRANSLATION;

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

        control = new RobotCentric()
            .withDeadband(0.01)
            .withRotationalDeadband(0.001)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    }

    @Override
    public void execute() {
        requiredSubsystem.setControl(
            control
                .withVelocityX(forwardSpeed * MAX_TRANSLATION)
                .withVelocityY(-sidewaysSpeed * MAX_TRANSLATION)
                .withRotationalRate(0.0)
        );
    }
}
