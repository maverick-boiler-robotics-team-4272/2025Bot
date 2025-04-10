package frc.robot.subsystems.drivetrain.states;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SubsystemConstants.DrivetrainConstants.AutoConstants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.utils.commandUtils.State;

public class PathfindThenPathState extends State<CommandSwerveDrivetrain> {
    Supplier<PathPlannerPath> pathSupplier;
    Command command;
    PathConstraints constraints;

    public PathfindThenPathState(CommandSwerveDrivetrain drivetrain, PathPlannerPath path) {
        super(drivetrain);

        pathSupplier = () -> path;

        this.constraints = new PathConstraints(
            AutoConstants.TRANSLATION_MAX,
            AutoConstants.TRANSLATION_MAX_A,
            AutoConstants.ROTATION_MAX.getDegrees(),
            AutoConstants.ROTATION_MAX_A.getDegrees()
        );
    }

    public PathfindThenPathState(CommandSwerveDrivetrain drivetrain, Supplier<PathPlannerPath> path) {
        super(drivetrain);

        pathSupplier = path;

        this.constraints = new PathConstraints(
            AutoConstants.TRANSLATION_MAX,
            AutoConstants.TRANSLATION_MAX_A,
            AutoConstants.ROTATION_MAX.getDegrees(),
            AutoConstants.ROTATION_MAX_A.getDegrees()
        );
    }

    public PathfindThenPathState(CommandSwerveDrivetrain drivetrain, PathPlannerPath path, PathConstraints constraints) {
        super(drivetrain);

        pathSupplier = () -> path;

        this.constraints = constraints;
    }

    public PathfindThenPathState(CommandSwerveDrivetrain drivetrain, Supplier<PathPlannerPath> path, PathConstraints constraints) {
        super(drivetrain);

        pathSupplier = path;

        this.constraints = constraints;
    }

    public PathfindThenPathState(CommandSwerveDrivetrain drivetrain, PathPlannerPath path, double maxVel, double maxAcc) {
        super(drivetrain);

        pathSupplier = () -> path;

        this.constraints = new PathConstraints(
            maxVel, 
            maxAcc, 
            AutoConstants.ROTATION_MAX.getDegrees(), 
            AutoConstants.ROTATION_MAX_A.getDegrees()
        );
    }

    public PathfindThenPathState(CommandSwerveDrivetrain drivetrain, Supplier<PathPlannerPath> path, double maxVel, double maxAcc) {
        super(drivetrain);

        pathSupplier = path;

        this.constraints = new PathConstraints(
            maxVel, 
            maxAcc, 
            AutoConstants.ROTATION_MAX.getDegrees(), 
            AutoConstants.ROTATION_MAX_A.getDegrees()
        );
    }

    @Override
    public void initialize() {
        this.command = AutoBuilder.pathfindThenFollowPath(
            pathSupplier.get(),
            constraints
        );

        command.initialize();
    }

    @Override
    public void execute() {
        command.execute();
    }

    @Override
    public void end(boolean interrupted) {
        command.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return command.isFinished();
    }
}
