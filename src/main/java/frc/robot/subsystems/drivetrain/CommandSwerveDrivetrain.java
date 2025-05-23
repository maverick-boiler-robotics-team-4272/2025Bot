package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.SubsystemConstants.DrivetrainConstants.AutoConstants;
import frc.robot.constants.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.utils.limelight.LimelightHelpers;
import frc.robot.utils.logging.Loggable;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

import static frc.robot.constants.FieldConstants.getGlobalPositions;
import static frc.robot.constants.SubsystemConstants.LimeLightConstants.*;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem, Loggable {
    @AutoLog
    public static class DrivetrainInputs {
        public Pose2d estimatedPose; // The Fused Pose of the robot
        public SwerveModuleState moduleStates[]; // The module states of the robot

        public boolean fuseVison; // Is the odometry fusing

        public Pose2d desiredPose; // The desired pose of pathplanner

        public double driveCurrents[]; // The current draw from all of the drive motors
        public double steerCurrents[]; // The current draw from all of the steer motors

        public Pose2d nextScorePose; // The next pose to score coral
        public Pose2d nextFeedPose; // The next pose to feed from
        public Pose2d nextBargePose; // The next barge pose to score algae

        public boolean getAlgae; // Should the robot grab the algae after scoring a coral
    }

    // Logging inputs
    private DrivetrainInputsAutoLogged inputs = new DrivetrainInputsAutoLogged();

    /**
     * Initializes the inputs for logging
     */
    private void initInputs() {
        inputs.fuseVison = false;
        inputs.estimatedPose = new Pose2d();
        inputs.desiredPose = new Pose2d();

        inputs.moduleStates = getState().ModuleStates;
        inputs.driveCurrents = new double[4];
        inputs.steerCurrents = new double[4];
        for(int i = 0; i < 4; i++) {
            var module = getModule(i);

            inputs.steerCurrents[i] = module.getSteerMotor().getStatorCurrent().getValueAsDouble();
            inputs.driveCurrents[i] = module.getDriveMotor().getStatorCurrent().getValueAsDouble();
        }

        inputs.nextScorePose = getGlobalPositions().CORAL_AB;
        inputs.nextFeedPose = getGlobalPositions().CORAL_STATION_LEFT_FAR_POINT;
        inputs.nextBargePose = getGlobalPositions().MIDDLE_BARGE;
        nextPath = getGlobalPositions().CORAL_A;
        nextBargePath = getGlobalPositions().MIDDLE_BARGE_PATH;
        nextFeedPath = getGlobalPositions().CORAL_STATION_LEFT_CLOSE;

        inputs.getAlgae = false;

        FRONT_LIMELIGHT.configure(FRONT_LIMELIGHT_POSE);
        ELEVATOR_LIMELIGHT.configure(ELEVATOR_LIMELIGHT_POSE);
        FRONT_2_LIMELIGHT.configure(FRONT_LIMELIGHT_2_POSE);
    }

    // The next path to run when the robot is pathfinding
    private PathPlannerPath nextPath;
    private PathPlannerPath nextMiddlePath;
    private PathPlannerPath nextFeedPath;
    private PathPlannerPath nextBargePath;
    
    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /* Swerve request to apply during robot-centric path following */
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        initPathPlanner();
        initInputs();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        initPathPlanner();
        initInputs();
    }   

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Toggles if algae should be grabbed next cycle
     */
    public void toggleAlgae() {
        if(inputs.getAlgae) {
            inputs.getAlgae = false;
        } else {
            inputs.getAlgae = true;
        }
    }

    public void setAlgaeGrab(boolean grab) {
        inputs.getAlgae = grab;
    }

    /**
     * @return if algae should be grabbed next cycle
     */
    public boolean getAlgae() {
        return inputs.getAlgae;
    }

    public Pose2d getNearestAlgae() {
        return getState().Pose.nearest(getGlobalPositions().ALGAE_POSES);
    }

    /**
     * Sets the next pose to pathfind to and path to follow during the autoteleop gameplay for scoring
     *
     * @param path the path to follow after reaching the starting point
     */
    public void setNextMiddlePath(Pose2d next, PathPlannerPath path) {
        inputs.nextScorePose = next;
        nextMiddlePath = path;
    }
    /**
     * Sets the next pose to pathfind to and path to follow during the autoteleop gameplay for scoring
     *
     * @param next the pose that the robot should pathfinded to; is mainly used for logging
     * @param path the path to follow after reaching the starting point
     */
    public void setNextScorePose(Pose2d next, PathPlannerPath path) {
        inputs.nextScorePose = next;
        nextPath = path;
    }
 
    /**
     * Sets the next pose to pathfind to for scoring
     *
     * @param next the pose to pathfind to next
     */
    public void setNextScorePose(Pose2d next) {
        inputs.nextScorePose = next;
    }
    
    /**
     * Sets the next pose to pathfind to for feeding
     *
     * @param next the pose to pathfind to next
     */
    public void setNextFeedPose(Pose2d next) {
        inputs.nextFeedPose = next;
    }

    public void setNextFeedPose(Pose2d next, PathPlannerPath path) {
        inputs.nextFeedPose = next;
        nextFeedPath = path;
    }

    /**
     * Sets the next pose to pathfind to and path to follow during the autoteleop gameplay for scoring
     *
     * @param next the pose that the robot should pathfinded to; is mainly used for logging
     * @param path the path to follow after reaching the starting point
     */

    public void setNextBargePose(Pose2d next, PathPlannerPath path) {
        inputs.nextBargePose = next;
        nextBargePath = path; 
    }

    /**
     * Sets the next pose to pathfind to for scoring
     *
     * @param next the pose to pathfind to next
     */
    public void setNextBargePose(Pose2d next) {
        inputs.nextBargePose = next;
    }

    /**
     * @returns the next pose that the robot will pathfind to to score
     */
    public Pose2d getNextScorePose() {
        return inputs.nextScorePose;
    }

    /**
     * @returns the next pose the robot with pathfind to to feed
     */
    public Pose2d getNextFeedPose() {
        return inputs.nextFeedPose;
    }

    /**
     * @return The next barge scoring pose
     */
    public Pose2d getNextBargePose() {
        return inputs.nextBargePose;
    }
    
    /**
     * @returns the next path that will run when pathfinding
     */
    public PathPlannerPath getNextPath() {
        return nextPath;
    }

    public PathPlannerPath getNextMiddlePath() {
        return nextMiddlePath;
    }

    public PathPlannerPath getNextFeedPath() {
        return nextFeedPath;
    }

    /**
     * @return the next path to score in the barge
     */
    public PathPlannerPath getNextBargePath() {
        return nextBargePath;
    }

    public boolean nextAlgaeHigh() {
        Pose2d algaePose = getNearestAlgae();

        return (
            algaePose.equals(getGlobalPositions().ALGAE_AB) ||
            algaePose.equals(getGlobalPositions().ALGAE_EF) ||
            algaePose.equals(getGlobalPositions().ALGAE_IJ)
        );
    }

    /**
     * Configures the AutoBuilder for autos and pathfinding
     */
    private void initPathPlanner() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getState().Pose,   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                () -> getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> setControl(
                    m_pathApplyRobotSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(AutoConstants.TRANSLATION_P, AutoConstants.TRANSLATION_I, AutoConstants.TRANSLATION_D),
                    // PID constants for rotation
                    new PIDConstants(AutoConstants.ROTATION_P, AutoConstants.ROTATION_I, AutoConstants.ROTATION_D)
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> !FieldConstants.isRedSide(),
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }

        PathPlannerLogging.setLogTargetPoseCallback((pose) -> setDesiredPose(pose));
    }

    /**
     * Fuses the odometry from the limelight readings
     */
    private void fuseOdometry() {
        FRONT_LIMELIGHT.setRobotOrientation(getState().Pose.getRotation().getDegrees());
        FRONT_2_LIMELIGHT.setRobotOrientation(getState().Pose.getRotation().getDegrees());
        ELEVATOR_LIMELIGHT.setRobotOrientation(getState().Pose.getRotation().getDegrees());

        fuseVision(FRONT_2_LIMELIGHT.getBotPoseEstimate());
        fuseVision(ELEVATOR_LIMELIGHT.getBotPoseEstimate());
        fuseVision(FRONT_LIMELIGHT.getBotPoseEstimate());
    }

    /**
     * Sets the desired pose for the robot
     * 
     * Is used for logging with the AutoBuilder and pathfinding
     *
     * @param pose the desired pose
     */
    public void setDesiredPose(Pose2d pose) {
        inputs.desiredPose = pose;
    }

    /**
     * Fuses the vision from the limelight measurment to the estimated pose
     * 
     * @param limelightMeasurement the reading from the limelight
     */
    public void fuseVision(LimelightHelpers.PoseEstimate limelightMeasurement) {
        if(limelightMeasurement != null) {
            if(
                limelightMeasurement.tagCount > 0 && 
                limelightMeasurement.avgTagDist < 3.0 && 
                Units.radiansToRotations(getState().Speeds.omegaRadiansPerSecond) < 2.0
            ) {
                setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
                addVisionMeasurement(
                    limelightMeasurement.pose,
                    // new Pose2d(limelightMeasurement.pose.getTranslation(), getState().Pose.getRotation()),
                    Utils.fpgaToCurrentTime(limelightMeasurement.timestampSeconds)
                );

                inputs.fuseVison = true;
            } else {
                inputs.fuseVison = false;
            }
        }
    }

    /**
     * @param subdirectory The subdirectory name
     * @param humanReadableName the name of the subsystem or item
     */
    @Override
    public void log(String subdirectory, String humanReadableName) {
        Logger.processInputs(subdirectory + "/" + humanReadableName, inputs);
    }

    /**
     * The periodic method in the subsystem base
     */
    @Override
    public void periodic() {
        boolean isRed = FieldConstants.isRedSide();

        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective) {
            setOperatorPerspectiveForward(
                isRed
                    ? kRedAlliancePerspectiveRotation
                    : kBlueAlliancePerspectiveRotation
            );
        }

        fuseOdometry();

        SwerveDriveState state = getState();
        
        inputs.estimatedPose = state.Pose;
        inputs.moduleStates = state.ModuleStates;
        
        for(int i = 0; i < 4; i++) {
            var module = getModule(i);

            inputs.steerCurrents[i] = module.getSteerMotor().getStatorCurrent().getValueAsDouble();
            inputs.driveCurrents[i] = module.getDriveMotor().getStatorCurrent().getValueAsDouble();
        }

        log("Subsystems", "Drivetrain");
    }

    // Below is all the drivetrain simulation stuff...
    
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    @SuppressWarnings("unused")
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    @SuppressWarnings("unused")
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
}
