// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.constants.TunerConstants;
import frc.robot.constants.positions.ArmevatorPosition;
import frc.robot.subsystems.algaeManipulator.AlgaeManipulator;
import frc.robot.subsystems.algaeManipulator.states.AlgaeIntake;
import frc.robot.subsystems.armevator.Armevator;
import frc.robot.subsystems.armevator.States.GoToArmevatorPoseState;
import frc.robot.subsystems.coralManipulator.CoralManipulator;
import frc.robot.subsystems.coralManipulator.states.CoralIntakeState;
import frc.robot.subsystems.coralManipulator.states.CoralOutakeState;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.states.DriveState;
import frc.robot.subsystems.drivetrain.states.ResetHeadingState;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.states.FeedState;

import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.constants.FieldConstants.SIDE_CHOOSER;
import static frc.robot.constants.SubsystemConstants.DrivetrainConstants.TeleConstants.MAX_TRANSLATION;

public class RobotContainer {
    private ShuffleboardTab autoTab;
    private SendableChooser<Command> autoChooser;

    private final Telemetry logger = new Telemetry(MAX_TRANSLATION);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final CoralManipulator coralManipulator = new CoralManipulator();
    public final Armevator armevator = new Armevator(coralManipulator.getArmEncoder());
    public final AlgaeManipulator algaeManipulator = new AlgaeManipulator();
    public final Feeder feeder = new Feeder();

    public RobotContainer() {
        configureBindings();
        registerNamedCommands();
        setupAutos();
    }
        
    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            new DriveState(
                drivetrain, 
                joystick::getLeftY,
                joystick::getLeftX,
                joystick::getRightX
            )
        );

        armevator.setDefaultCommand(
            new GoToArmevatorPoseState(
                armevator, 
                new ArmevatorPosition(Rotation2d.fromDegrees(10), Meters.convertFrom(0.1, Inches))
            )
        );

        // reset the field-centric heading on b press
        joystick.b().onTrue(new ResetHeadingState(drivetrain));

        joystick.x().whileTrue(
            new GoToArmevatorPoseState(
                armevator, 
                new ArmevatorPosition(
                    Rotation2d.fromDegrees(135), 
                    Meters.convertFrom(36, Inches)
                )
            )
        );

        joystick.leftBumper().whileTrue(
            new FeedState(feeder)
                .alongWith(new CoralIntakeState(coralManipulator))
        );

        joystick.rightBumper().whileTrue(
            new CoralOutakeState(coralManipulator)
        );

        joystick.povDown().whileTrue(
            new AlgaeIntake(algaeManipulator)
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // joystick.a().whileTrue(new PathfindingState(drivetrain, getGlobalPositions().CORAL_STATION_LEFT));
        // joystick.y().whileTrue(new PathfindingState(drivetrain, getGlobalPositions().CORAL_EF));

        if(!Robot.isReal()) {
            drivetrain.registerTelemetry(logger::telemeterize);
        }

        configureButtons();
    }

    private void configureButtons() {
        // var buttonTab = Shuffleboard.getTab("Buttons");
        
        // buttonTab.add("AB", new InstantCommand(() -> drivetrain.setNextScorePose(getGlobalPositions().CORAL_AB)).ignoringDisable(true));
        // buttonTab.add("CD", new InstantCommand(() -> drivetrain.setNextScorePose(getGlobalPositions().CORAL_CD)).ignoringDisable(true));
        // buttonTab.add("EF", new InstantCommand(() -> drivetrain.setNextScorePose(getGlobalPositions().CORAL_EF)).ignoringDisable(true));
        // buttonTab.add("GH", new InstantCommand(() -> drivetrain.setNextScorePose(getGlobalPositions().CORAL_GH)).ignoringDisable(true));
        // buttonTab.add("IJ", new InstantCommand(() -> drivetrain.setNextScorePose(getGlobalPositions().CORAL_IJ)).ignoringDisable(true));
        // buttonTab.add("KL", new InstantCommand(() -> drivetrain.setNextScorePose(getGlobalPositions().CORAL_KL)).ignoringDisable(true));

        // buttonTab.add("Left", new InstantCommand(() -> drivetrain.setNextFeedPose(getGlobalPositions().CORAL_STATION_LEFT)).ignoringDisable(true));
        // buttonTab.add("Right", new InstantCommand(() -> drivetrain.setNextFeedPose(getGlobalPositions().CORAL_STATION_RIGHT)).ignoringDisable(true));
    }

    private void registerNamedCommands() {
        // NamedCommands.registerCommand("Drop", new DropState(dropper).withTimeout(0.5)); //ex
    }

    private void setupAutos() {
        autoChooser = new SendableChooser<>();

        SIDE_CHOOSER.setDefaultOption("Red", "red");
        SIDE_CHOOSER.addOption("Blue", "blue");
    
        autoTab = Shuffleboard.getTab("Auto");
        autoTab.add("AutoChooser", autoChooser);
        autoTab.add("SideChooser", SIDE_CHOOSER);

        // autoChooser.setDefaultOption("5 coral!!!", new PathPlannerAuto("5 coral!!!")); //ex
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
