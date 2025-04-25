package frc.robot.commands;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class FollowPathWithCustomTolerance extends Command {
    private final FollowPathCommand pathCommand;
    private final Supplier<Pose2d> poseSupplier;
    private final Pose2d targetPose;
    private final double toleranceMeters;
    CommandSwerveDrivetrain drivetrain;
    Supplier<Pose2d> otherPoseSupplier = () -> drivetrain.getPose();// Saved pose supplier ✅ <- that is how you know it was Chat GPT



    public FollowPathWithCustomTolerance(
            PathPlannerPath path,
            Supplier<Pose2d> poseSupplier,
            Supplier<ChassisSpeeds> speedsSupplier,
            BiConsumer<ChassisSpeeds, DriveFeedforwards> output,
            PathFollowingController controller,
            RobotConfig robotConfig,
            BooleanSupplier shouldFlipPath,
            Pose2d targetPose,
            double toleranceMeters,
            Subsystem... requirements) {

        this.poseSupplier = otherPoseSupplier; // Store robot estimated pose here
        this.targetPose = targetPose;
        this.toleranceMeters = toleranceMeters;

        this.pathCommand = new FollowPathCommand(
                path,
                poseSupplier,
                speedsSupplier,
                output,
                controller,
                robotConfig,
                shouldFlipPath,
                requirements
        );
    }

    public Supplier<Pose2d> getPoseSupplier() {
        return poseSupplier;
    }

    @Override
    public void initialize() {
        pathCommand.initialize();
    }

    @Override
    public void execute() {
        pathCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        pathCommand.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        Pose2d currentPose = getPoseSupplier().get();
        double dx = currentPose.getX() - targetPose.getX();
        double dy = currentPose.getY() - targetPose.getY();
        return Math.hypot(dx, dy) <= toleranceMeters;
    }
}
