package frc.robot.subsystems.drivetrain.states;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.utils.commandUtils.State;

import static frc.robot.constants.SubsystemConstants.DrivetrainConstants.TeleConstants.*;
import static frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain.isSafe;

public class DriveState extends State<CommandSwerveDrivetrain> {
    private DoubleSupplier xAxis;
    private DoubleSupplier yAxis;
    private DoubleSupplier theta;

    private FieldCentric request;

    public DriveState(CommandSwerveDrivetrain drivetrain, DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier thetaSpeed) {
        super(drivetrain);

        this.xAxis = xSpeed;
        this.yAxis = ySpeed;
        this.theta = thetaSpeed;

        request = new SwerveRequest.FieldCentric()
            .withDeadband(MAX_TRANSLATION * 0.0).withRotationalDeadband(MAX_ANGULAR * 0.001)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    }

    @Override
    public void execute() {
        //TODO:may be fix for drive assist if in a periodic. execute happens only once
        if(!isSafe){
            requiredSubsystem.setControl(
                request
                    .withVelocityX(-xAxis.getAsDouble() * MAX_TRANSLATION)
                    .withVelocityY(-yAxis.getAsDouble() * MAX_TRANSLATION)
                    .withRotationalRate(-theta.getAsDouble() * MAX_ANGULAR)
            );
        } else {
            requiredSubsystem.setControl(
                request
                    .withVelocityX(-xAxis.getAsDouble() * MAX_TRANSLATION * 0.1)
                    .withVelocityY(-yAxis.getAsDouble() * MAX_TRANSLATION * 0.1)
                    .withRotationalRate(-theta.getAsDouble() * MAX_ANGULAR)
            );
        }
    }
}
