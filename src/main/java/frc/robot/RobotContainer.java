// Copyright (c) FIRST and other WPILib contributors.
// eOpen Source Software; you can modify and/or share it under the terms of
// th WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AutoAlgaeCommand;
import frc.robot.commands.AutoGameCommand;
import frc.robot.commands.FeederManipulatorCommand;
import frc.robot.constants.TunerConstants;
import frc.robot.constants.positions.ArmevatorPositions;
import frc.robot.constants.positions.ArmevatorPositions.ArmevatorPosition;
import frc.robot.subsystems.algaeManipulator.AlgaeManipulator;
import frc.robot.subsystems.algaeManipulator.states.AlgaeIdle;
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
import frc.robot.subsystems.coralManipulator.states.IdleState;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.states.DriveState;
import frc.robot.subsystems.drivetrain.states.ResetHeadingState;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.states.FeedState;
import frc.robot.utils.controllers.ButtonBoard;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.constants.SubsystemConstants.DrivetrainConstants.TeleConstants.MAX_TRANSLATION;
import static frc.robot.constants.FieldConstants.*;
import static frc.robot.constants.positions.ArmevatorPositions.*;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

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
            ).repeatedly()
        );

        coralManipulator.setDefaultCommand(
            new IdleState(coralManipulator, armevator::getArmRotation)
        );

        algaeManipulator.setDefaultCommand(
            new AlgaeIdle(algaeManipulator)
        );

        // reset the field-centric heading on b press
        driverController.b().onTrue(new ResetHeadingState(drivetrain));

        driverController.x().whileTrue(
            new AutoGameCommand(
                drivetrain, 
                armevator, 
                feeder, 
                coralManipulator,
                () -> driverController.a().getAsBoolean()
            ).repeatedly()
        );

        driverController.y().whileTrue(
            new AutoAlgaeCommand(
                drivetrain, 
                armevator, 
                algaeManipulator
            )
        );

        driverController.leftBumper().whileTrue(
            new CoralIntakeState(coralManipulator, 0.5)
        );

        driverController.rightBumper().whileTrue(
            new CoralOutakeState(coralManipulator, 1)
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
        operatorController.getButton(7).whileTrue(
            new ClimbState(climber)
        );
        operatorController.getButton(8).whileTrue(
            new LowerState(climber)
        );

        operatorController.getButton(5).whileTrue(
            new FeederManipulatorCommand(
                feeder, 
                coralManipulator, 
                armevator,
                1.0, 
                0.14
            )
        );

        operatorController.getButton(6).whileTrue(
            new FeedState(feeder, -1.0)
        );

        operatorController.getButton(4 + 16).whileTrue(
            new GoToArmevatorPoseState(armevator, ALGAE_ARMEVATOR_POSITION)
                .alongWith(new AlgaeIntake(algaeManipulator)).repeatedly()
        );

        operatorController.getButton(3 + 16).whileTrue(
            new GoToArmevatorPoseState(armevator, ALGAE_ARMEVATOR_POSITION_TWO)
                .alongWith(new AlgaeIntake(algaeManipulator)).repeatedly()
        );

        operatorController.getButton(11).onTrue(
            new InstantCommand(() -> drivetrain.setNextBargePose(getGlobalPositions().LEFT_BARGE)).ignoringDisable(true)
        );

        operatorController.getButton(10).onTrue(
            new InstantCommand(() -> drivetrain.setNextBargePose(getGlobalPositions().MIDDLE_BARGE)).ignoringDisable(true)
        );

        operatorController.getButton(9).onTrue(
            new InstantCommand(() -> drivetrain.setNextBargePose(getGlobalPositions().RIGHT_BARGE)).ignoringDisable(true)
        );

        operatorController.getButton(14).whileTrue(
            // new GoToArmevatorPoseState(armevator, L1_ARMEVATOR_POSITION).repeatedly()
            new InstantCommand(() -> armevator.goToPosNext(L1_ARMEVATOR_POSITION))
        );

        operatorController.getButton(13).whileTrue(
            // new GoToArmevatorPoseState(armevator, L2_ARMEVATOR_POSITION).repeatedly()
            new InstantCommand(() -> armevator.goToPosNext(L2_ARMEVATOR_POSITION))
        );

        operatorController.getButton(16 + 2).whileTrue(
            // new GoToArmevatorPoseState(armevator, L3_ARMEVATOR_POSITION).repeatedly()
            new InstantCommand(() -> armevator.goToPosNext(L3_ARMEVATOR_POSITION))
        );

        operatorController.getButton(16 + 1).whileTrue(
            // new GoToArmevatorPoseState(armevator, L4_ARMEVATOR_POSITION).repeatedly()
            new InstantCommand(() -> armevator.goToPosNext(L4_ARMEVATOR_POSITION))
        );

        operatorController.getButton(14)
            .and(() -> !driverController.x().getAsBoolean())
            .and(() -> !driverController.y().getAsBoolean())
        .whileTrue(
            new GoToArmevatorPoseState(armevator, L1_ARMEVATOR_POSITION).repeatedly()
        );

        operatorController.getButton(13)
            .and(() -> !driverController.x().getAsBoolean())
            .and(() -> !driverController.y().getAsBoolean())
        .whileTrue(
            new GoToArmevatorPoseState(armevator, L2_ARMEVATOR_POSITION).repeatedly()
        );

        operatorController.getButton(16 + 2)
            .and(() -> !driverController.x().getAsBoolean())
            .and(() -> !driverController.y().getAsBoolean())
        .whileTrue(
            new GoToArmevatorPoseState(armevator, L3_ARMEVATOR_POSITION).repeatedly()
        );

        operatorController.getButton(16 + 1)
            .and(() -> !driverController.x().getAsBoolean())
            .and(() -> !driverController.y().getAsBoolean())
        .whileTrue(
            new GoToArmevatorPoseState(armevator, L4_ARMEVATOR_POSITION).repeatedly()
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

        operatorController.getButton(1).whileTrue(
            // new PathfindingState(drivetrain, getGlobalPositions().CORAL_STATION_LEFT)
            new InstantCommand(() -> drivetrain.setNextFeedPose(getGlobalPositions().CORAL_STATION_LEFT)).ignoringDisable(true)
        );

        operatorController.getButton(2).whileTrue(
            // new PathfindingState(drivetrain, getGlobalPositions().CORAL_STATION_RIGHT)
            new InstantCommand(() -> drivetrain.setNextFeedPose(getGlobalPositions().CORAL_STATION_RIGHT)).ignoringDisable(true)
        );
    }

    private void registerNamedCommands() {
        // NamedCommands.registerCommand("Drop", new DropState(dropper).withTimeout(0.5)); //ex
        NamedCommands.registerCommand("L4", new GoToArmevatorPoseState(armevator, L4_ARMEVATOR_POSITION).withTimeout(1));
    }

    private void setupAutos() {
        autoChooser = new SendableChooser<>();

        SIDE_CHOOSER.setDefaultOption("Red", "red");
        SIDE_CHOOSER.addOption("Blue", "blue");
    
        autoTab = Shuffleboard.getTab("Auto");
        autoTab.add("AutoChooser", autoChooser);
        autoTab.add("SideChooser", SIDE_CHOOSER);

        //autoChooser.setDefaultOption("Wheel Diam", new PathPlannerAuto("Wheel Diam"));

        autoChooser.setDefaultOption("test auto", new PathPlannerAuto("test auto"));
        // autoChooser.setDefaultOption("5 coral!!!", new PathPlannerAuto("5 coral!!!")); //ex
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
