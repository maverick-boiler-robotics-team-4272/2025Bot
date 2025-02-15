// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.FeederManipulatorCommand;
import frc.robot.constants.TunerConstants;
import frc.robot.constants.positions.ArmevatorPositions.ArmevatorPosition;
import frc.robot.subsystems.algaeManipulator.AlgaeManipulator;
import frc.robot.subsystems.algaeManipulator.states.AlgaeIntake;
import frc.robot.subsystems.armevator.Armevator;
import frc.robot.subsystems.armevator.states.GoToArmevatorPoseState;
import frc.robot.subsystems.armevator.states.ZeroState;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.states.LowerState;
import frc.robot.subsystems.climber.states.ClimbState;
import frc.robot.subsystems.coralManipulator.CoralManipulator;
import frc.robot.subsystems.coralManipulator.states.CoralIntakeState;
import frc.robot.subsystems.coralManipulator.states.CoralOutakeState;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.states.DriveState;
import frc.robot.subsystems.drivetrain.states.PathfindThenPathState;
import frc.robot.subsystems.drivetrain.states.PathfindingState;
import frc.robot.subsystems.drivetrain.states.ResetHeadingState;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.states.FeedState;
import frc.robot.utils.controllers.ButtonBoard;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.constants.SubsystemConstants.DrivetrainConstants.TeleConstants.MAX_TRANSLATION;
import static frc.robot.constants.FieldConstants.*;

public class RobotContainer {
    private ShuffleboardTab autoTab;
    private SendableChooser<Command> autoChooser;

    private final Telemetry logger = new Telemetry(MAX_TRANSLATION);

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final ButtonBoard operatorController = new ButtonBoard(1, 2);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final CoralManipulator coralManipulator = new CoralManipulator();
    public final Armevator armevator = new Armevator(coralManipulator.getArmEncoder());
    public final AlgaeManipulator algaeManipulator = new AlgaeManipulator();
    public final Feeder feeder = new Feeder();
    public final Climber climber = new Climber();

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
                driverController::getLeftY,
                driverController::getLeftX,
                driverController::getRightX
            )
        );

        armevator.setDefaultCommand(
            new GoToArmevatorPoseState(
                armevator, 
                new ArmevatorPosition(Rotation2d.fromDegrees(10), Meters.convertFrom(0.1, Inches))
            )
        );

        // reset the field-centric heading on b press
        driverController.b().onTrue(new ResetHeadingState(drivetrain));

        driverController.x().whileTrue(
            new PathfindThenPathState(
                drivetrain, 
                drivetrain::getNextPath
            )
        );

        driverController.y().whileTrue(
            new PathfindingState(
                drivetrain,
                drivetrain::getNextFeedPose
            )
        );

        driverController.leftBumper().whileTrue(
            new FeedState(feeder, 1)
                .alongWith(new CoralIntakeState(coralManipulator, 1))
        );

        driverController.rightBumper().whileTrue(
            new CoralOutakeState(coralManipulator)
        );

        driverController.povLeft().whileTrue(
            new AlgaeIntake(algaeManipulator)
        );

        driverController.povDown().whileTrue(
            new LowerState(climber)  
        );

        driverController.povUp().whileTrue(
            new ClimbState(climber)
        );

        driverController.start().whileTrue(
            new ZeroState(armevator)  
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // joystick.a().whileTrue(new PathfindingState(drivetrain, getGlobalPositions().CORAL_STATION_LEFT));
        // joystick.y().whileTrue(new PathfindingState(drivetrain, getGlobalPositions().CORAL_EF));

        if(!Robot.isReal()) {
            drivetrain.registerTelemetry(logger::telemeterize);
        }

        configureButtons();
    }

    private void configureButtons() {
        /*
        var buttonTab = Shuffleboard.getTab("Buttons");
        
        buttonTab.add("AB", new InstantCommand(() -> drivetrain.setNextScorePose(getGlobalPositions().CORAL_AB)).ignoringDisable(true));
        buttonTab.add("CD", new InstantCommand(() -> drivetrain.setNextScorePose(getGlobalPositions().CORAL_CD)).ignoringDisable(true));
        buttonTab.add("EF", new InstantCommand(() -> drivetrain.setNextScorePose(getGlobalPositions().CORAL_EF)).ignoringDisable(true));
        buttonTab.add("GH", new InstantCommand(() -> drivetrain.setNextScorePose(getGlobalPositions().CORAL_GH)).ignoringDisable(true));
        buttonTab.add("IJ", new InstantCommand(() -> drivetrain.setNextScorePose(getGlobalPositions().CORAL_IJ)).ignoringDisable(true));
        buttonTab.add("KL", new InstantCommand(() -> drivetrain.setNextScorePose(getGlobalPositions().CORAL_KL)).ignoringDisable(true));

        buttonTab.add("Left", new InstantCommand(() -> drivetrain.setNextFeedPose(getGlobalPositions().CORAL_STATION_LEFT)).ignoringDisable(true));
        buttonTab.add("Right", new InstantCommand(() -> drivetrain.setNextFeedPose(getGlobalPositions().CORAL_STATION_RIGHT)).ignoringDisable(true));
        */
        
        operatorController.getButton(5).whileTrue(
            new GoToArmevatorPoseState(
                armevator,
                new ArmevatorPosition(
                    Rotation2d.fromDegrees(-210.0), 
                    Meters.convertFrom(30, Inches)
                )
            )  
        );

        operatorController.getButton(6).whileTrue(
            new GoToArmevatorPoseState(
                armevator,
                new ArmevatorPosition(
                    Rotation2d.fromDegrees(210.0), 
                    Meters.convertFrom(30, Inches)
                )
            )  
        );

        operatorController.getButton(14).whileTrue(
            new FeederManipulatorCommand(
                feeder, 
                coralManipulator, 
                0.2, 
                0.2, 
                0.0
            )
        );

        //Reef buttons

        //Coral A
        operatorController.getButton(11+16).whileTrue(
            new InstantCommand(() -> drivetrain.setNextScorePose(getGlobalPositions().CORAL_AB, getGlobalPositions().CORAL_A)).ignoringDisable(true)
        );

        //Coral B
        operatorController.getButton(12+16).whileTrue(
            new InstantCommand(() -> drivetrain.setNextScorePose(getGlobalPositions().CORAL_AB, getGlobalPositions().CORAL_B)).ignoringDisable(true)
        );

        //Coral C
        operatorController.getButton(13+16).whileTrue(
            new InstantCommand(() -> drivetrain.setNextScorePose(getGlobalPositions().CORAL_CD, getGlobalPositions().CORAL_C)).ignoringDisable(true)
        );

        //Coral D
        operatorController.getButton(15).whileTrue(
            new InstantCommand(() -> drivetrain.setNextScorePose(getGlobalPositions().CORAL_CD, getGlobalPositions().CORAL_D)).ignoringDisable(true)
        );

        //Coral E
        operatorController.getButton(14+16).whileTrue(
            new InstantCommand(() -> drivetrain.setNextScorePose(getGlobalPositions().CORAL_EF, getGlobalPositions().CORAL_E)).ignoringDisable(true)
        );

        //Coral F
        operatorController.getButton(12).whileTrue(
            new InstantCommand(() -> drivetrain.setNextScorePose(getGlobalPositions().CORAL_EF, getGlobalPositions().CORAL_F)).ignoringDisable(true)
        );

        //Coral G
        operatorController.getButton(5+16).whileTrue(
            new InstantCommand(() -> drivetrain.setNextScorePose(getGlobalPositions().CORAL_GH, getGlobalPositions().CORAL_G)).ignoringDisable(true)  
        );

        //Coral H
        operatorController.getButton(6+16).whileTrue(
            new InstantCommand(() -> drivetrain.setNextScorePose(getGlobalPositions().CORAL_GH, getGlobalPositions().CORAL_H)).ignoringDisable(true)  
        );

        //Coral I
        operatorController.getButton(7+16).whileTrue(
            new InstantCommand(() -> drivetrain.setNextScorePose(getGlobalPositions().CORAL_IJ, getGlobalPositions().CORAL_I)).ignoringDisable(true)  
        );

        //Coral J
        operatorController.getButton(8+16).whileTrue(
            new InstantCommand(() -> drivetrain.setNextScorePose(getGlobalPositions().CORAL_IJ, getGlobalPositions().CORAL_J)).ignoringDisable(true)  
        );

        //Coral K
        operatorController.getButton(9+16).whileTrue(
            new InstantCommand(() -> drivetrain.setNextScorePose(getGlobalPositions().CORAL_KL, getGlobalPositions().CORAL_K)).ignoringDisable(true)
        );

        //Coral L
        operatorController.getButton(10+16).whileTrue(
            new InstantCommand(() -> drivetrain.setNextScorePose(getGlobalPositions().CORAL_KL, getGlobalPositions().CORAL_L)).ignoringDisable(true)
        );
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
