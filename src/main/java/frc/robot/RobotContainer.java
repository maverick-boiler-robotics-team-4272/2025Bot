// Copyright (c) FIRST and other WPILib contributors.
// eOpen Source Software; you can modify and/or share it under the terms of
// th WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AutoAlgaeCommand;
import frc.robot.commands.AutoGameCommand;
import frc.robot.commands.AutoGamePrepCommand;
import frc.robot.commands.AutonomousFeedTillFirstLidar;
import frc.robot.commands.BargeScoreCommand;
import frc.robot.commands.FeederManipulatorCommand;
import frc.robot.constants.TunerConstants;
import frc.robot.constants.positions.ArmevatorPositions.ArmevatorPosition;
import frc.robot.subsystems.algaeManipulator.AlgaeManipulator;
import frc.robot.subsystems.algaeManipulator.states.AlgaeIdle;
import frc.robot.subsystems.algaeManipulator.states.AlgaeIntake;
import frc.robot.subsystems.armevator.Armevator;
import frc.robot.subsystems.armevator.states.GoToArmevatorPoseState;
import frc.robot.subsystems.armevator.states.GoToNextArmevatorPoseState;
import frc.robot.subsystems.armevator.states.ZeroState;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.states.LowerState;
import frc.robot.subsystems.climber.states.ClimbState;
import frc.robot.subsystems.coralManipulator.CoralManipulator;
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
    private boolean elliott = false; //is elliot driving? //yes
    private boolean buttonBoardInUse = true; // Is an override tool if the button board does not work.

    private ShuffleboardTab autoTab;
    private SendableChooser<Command> autoChooser;

    private final Telemetry logger = new Telemetry(MAX_TRANSLATION);

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final ButtonBoard buttonBoard = new ButtonBoard(1, 2);
    private final CommandXboxController operatorController = new CommandXboxController(3);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public static final CoralManipulator coralManipulator = new CoralManipulator();
    public static final Armevator armevator = new Armevator(coralManipulator.getArmEncoder());
    public static final AlgaeManipulator algaeManipulator = new AlgaeManipulator();
    public static final Feeder feeder = new Feeder();
    public static final Climber climber = new Climber();

    public RobotContainer() {
        setDefaultCommands();
        configureBindings();

        if(buttonBoardInUse) {
            configureButtons();
        } else {
            configureBackupBindings();
        }

        setupAutos();
    }

    private void setDefaultCommands() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        if(elliott) {
            drivetrain.setDefaultCommand(
                new DriveState(
                    drivetrain, 
                    () -> Math.pow(driverController.getLeftY(), 3),
                    () -> Math.pow(driverController.getLeftX(), 3),
                    () -> Math.pow(driverController.getRightX(), 3)
                )
            );
        } else {
            drivetrain.setDefaultCommand(
                new DriveState(
                    drivetrain, 
                    driverController::getLeftY,
                    driverController::getLeftX,
                    driverController::getRightX
                )
            ); // Andy code cause andy built like a beast with linear.
        }

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
    }
        
    private void configureBindings() {
        // reset the field-centric heading on b press
        driverController.b().onTrue(new ResetHeadingState(drivetrain).ignoringDisable(true));

        driverController.x().whileTrue(
            new AutoGameCommand(
                drivetrain, 
                armevator, 
                feeder, 
                coralManipulator,
                () -> driverController.a().getAsBoolean()
            ).repeatedly().withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );

        driverController.y().whileTrue(
            new AutoGameCommand(
                drivetrain, 
                armevator, 
                feeder, 
                coralManipulator,
                () -> driverController.a().getAsBoolean()
            ).repeatedly().beforeStarting(
                new AutoGamePrepCommand(
                    drivetrain, 
                    armevator, 
                    feeder, 
                    coralManipulator
                )
            ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );

        driverController.rightStick().whileTrue(
            new AutoAlgaeCommand(
                drivetrain, 
                armevator, 
                algaeManipulator
            )
        );

        driverController.leftTrigger().whileTrue(
            new FeederManipulatorCommand(
                feeder, 
                coralManipulator, 
                armevator
            ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );

        driverController.rightTrigger().whileTrue(
            new FeedState(feeder, -1.0)
                .alongWith(new CoralOutakeState(coralManipulator, 0.5))
        );

        if(!buttonBoardInUse) {
            driverController.rightBumper().whileTrue(
                new ConditionalCommand(
                    new CoralOutakeState(coralManipulator, 0.8), 
                    new ConditionalCommand(
                        new CoralOutakeState(coralManipulator, -0.25), 
                        new CoralOutakeState(coralManipulator, -0.8),
                        armevator::nextIsL1
                    ),
                    armevator::nextIsL4
                )
            );

            driverController.leftBumper().whileTrue(
                new ConditionalCommand(
                    new CoralOutakeState(coralManipulator, -0.8), 
                    new CoralOutakeState(coralManipulator, 0.8), 
                    armevator::nextIsL4
                )
            );
        }

        driverController.start().whileTrue(
            new ZeroState(armevator)  
        );

        driverController.back().whileTrue(
            new InstantCommand(() -> armevator.resetArm())
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        if(!Robot.isReal()) {
            drivetrain.registerTelemetry(logger::telemeterize);
        }
    }

    private void configureButtons() {
        buttonBoard.getButton(7).whileTrue(
            new ClimbState(climber)
        );
        buttonBoard.getButton(8).whileTrue(
            new LowerState(climber)
        );

        buttonBoard.getButton(5).whileTrue(
            new ConditionalCommand(
                new CoralOutakeState(coralManipulator, 0.8), 
                new ConditionalCommand(
                    new CoralOutakeState(coralManipulator, -0.25), 
                    new CoralOutakeState(coralManipulator, -0.8),
                    armevator::nextIsL1
                ),
                armevator::nextIsL4
            )
        );

        buttonBoard.getButton(6).whileTrue(
            new ConditionalCommand(
                new CoralOutakeState(coralManipulator, -0.8), 
                new CoralOutakeState(coralManipulator, 0.8),
                armevator::nextIsL4
            )  
        );

        buttonBoard.getButton(4 + 16).whileTrue(
            new GoToArmevatorPoseState(armevator, ALGAE_ARMEVATOR_POSITION)
                .alongWith(new AlgaeIntake(algaeManipulator)).repeatedly()
        );

        buttonBoard.getButton(4 + 16).onTrue(
            new InstantCommand(drivetrain::toggleAlgae)  
        );

        buttonBoard.getButton(3 + 16).whileTrue(
            new GoToArmevatorPoseState(armevator, ALGAE_ARMEVATOR_POSITION_TWO)
                .alongWith(new AlgaeIntake(algaeManipulator)).repeatedly()
        );

        buttonBoard.getButton(11).onTrue(
            new InstantCommand(() -> drivetrain.setNextBargePose(getGlobalPositions().LEFT_BARGE, getGlobalPositions().LEFT_BARGE_PATH)).ignoringDisable(true)
        );

        buttonBoard.getButton(10).onTrue(
            new InstantCommand(() -> drivetrain.setNextBargePose(getGlobalPositions().MIDDLE_BARGE, getGlobalPositions().MIDDLE_BARGE_PATH)).ignoringDisable(true)
        );

        buttonBoard.getButton(9).onTrue(
            new InstantCommand(() -> drivetrain.setNextBargePose(getGlobalPositions().RIGHT_BARGE, getGlobalPositions().RIGHT_BARGE_PATH)).ignoringDisable(true)  
        );

        buttonBoard.getButton(11).whileTrue(
            new BargeScoreCommand(armevator, algaeManipulator, () -> driverController.povLeft().getAsBoolean())
        );

        buttonBoard.getButton(10).whileTrue(
            new BargeScoreCommand(armevator, algaeManipulator, () -> driverController.povLeft().getAsBoolean())
        );

        buttonBoard.getButton(9).whileTrue(
            new BargeScoreCommand(armevator, algaeManipulator, () -> driverController.povLeft().getAsBoolean())
        );

        buttonBoard.getButton(14).whileTrue(
            // new GoToArmevatorPoseState(armevator, L1_ARMEVATOR_POSITION).repeatedly()
            new InstantCommand(() -> armevator.goToPosNext(L1_ARMEVATOR_POSITION)).ignoringDisable(true)
        );

        buttonBoard.getButton(13).whileTrue(
            // new GoToArmevatorPoseState(armevator, L2_ARMEVATOR_POSITION).repeatedly()
            new InstantCommand(() -> armevator.goToPosNext(L2_ARMEVATOR_POSITION)).ignoringDisable(true)
        );

        buttonBoard.getButton(16 + 2).whileTrue(
            // new GoToArmevatorPoseState(armevator, L3_ARMEVATOR_POSITION).repeatedly()
            new InstantCommand(() -> armevator.goToPosNext(L3_ARMEVATOR_POSITION)).ignoringDisable(true)
        );

        buttonBoard.getButton(16 + 1).whileTrue(
            // new GoToArmevatorPoseState(armevator, L4_ARMEVATOR_POSITION).repeatedly()
            new InstantCommand(() -> armevator.goToPosNext(L4_ARMEVATOR_POSITION)).ignoringDisable(true)
        );

        buttonBoard.getButton(14).whileTrue(
            new GoToArmevatorPoseState(armevator, L1_ARMEVATOR_POSITION).repeatedly()
        );

        buttonBoard.getButton(13).whileTrue(
            new GoToArmevatorPoseState(armevator, L2_ARMEVATOR_POSITION).repeatedly()
        );

        buttonBoard.getButton(16 + 2).whileTrue(
            new GoToArmevatorPoseState(armevator, L3_ARMEVATOR_POSITION).repeatedly()
        );

        buttonBoard.getButton(16 + 1).whileTrue(
            new GoToArmevatorPoseState(armevator, L4_ARMEVATOR_POSITION).repeatedly()
        );

        //Reef buttons

        //Coral A
        buttonBoard.getButton(11+16).whileTrue(
            new InstantCommand(() -> drivetrain.setNextScorePose(getGlobalPositions().CORAL_AB, getGlobalPositions().CORAL_A)).ignoringDisable(true)
        );

        //Coral B
        buttonBoard.getButton(12+16).whileTrue(
            new InstantCommand(() -> drivetrain.setNextScorePose(getGlobalPositions().CORAL_AB, getGlobalPositions().CORAL_B)).ignoringDisable(true)
        );

        //Coral C
        buttonBoard.getButton(13+16).whileTrue(
            new InstantCommand(() -> drivetrain.setNextScorePose(getGlobalPositions().CORAL_CD, getGlobalPositions().CORAL_C)).ignoringDisable(true)
        );

        //Coral D
        buttonBoard.getButton(15).whileTrue(
            new InstantCommand(() -> drivetrain.setNextScorePose(getGlobalPositions().CORAL_CD, getGlobalPositions().CORAL_D)).ignoringDisable(true)
        );

        //Coral E
        buttonBoard.getButton(14+16).whileTrue(
            new InstantCommand(() -> drivetrain.setNextScorePose(getGlobalPositions().CORAL_EF, getGlobalPositions().CORAL_E)).ignoringDisable(true)
        );

        //Coral F
        buttonBoard.getButton(12).whileTrue(
            new InstantCommand(() -> drivetrain.setNextScorePose(getGlobalPositions().CORAL_EF, getGlobalPositions().CORAL_F)).ignoringDisable(true)
        );

        //Coral G
        buttonBoard.getButton(5+16).whileTrue(
            new InstantCommand(() -> drivetrain.setNextScorePose(getGlobalPositions().CORAL_GH, getGlobalPositions().CORAL_G)).ignoringDisable(true)  
        );

        //Coral H
        buttonBoard.getButton(6+16).whileTrue(
            new InstantCommand(() -> drivetrain.setNextScorePose(getGlobalPositions().CORAL_GH, getGlobalPositions().CORAL_H)).ignoringDisable(true)  
        );

        //Coral I
        buttonBoard.getButton(7+16).whileTrue(
            new InstantCommand(() -> drivetrain.setNextScorePose(getGlobalPositions().CORAL_IJ, getGlobalPositions().CORAL_I)).ignoringDisable(true)  
        );

        //Coral J
        buttonBoard.getButton(8+16).whileTrue(
            new InstantCommand(() -> drivetrain.setNextScorePose(getGlobalPositions().CORAL_IJ, getGlobalPositions().CORAL_J)).ignoringDisable(true)  
        );

        //Coral K
        buttonBoard.getButton(9+16).whileTrue(
            new InstantCommand(() -> drivetrain.setNextScorePose(getGlobalPositions().CORAL_KL, getGlobalPositions().CORAL_K)).ignoringDisable(true)
        );

        //Coral L
        buttonBoard.getButton(10+16).whileTrue(
            new InstantCommand(() -> drivetrain.setNextScorePose(getGlobalPositions().CORAL_KL, getGlobalPositions().CORAL_L)).ignoringDisable(true)
        );

        buttonBoard.getButton(1).whileTrue(
            // new PathfindingState(drivetrain, getGlobalPositions().CORAL_STATION_LEFT)
            new InstantCommand(() -> drivetrain.setNextFeedPose(getGlobalPositions().CORAL_STATION_LEFT)).ignoringDisable(true)
        );

        buttonBoard.getButton(2).whileTrue(
            // new PathfindingState(drivetrain, getGlobalPositions().CORAL_STATION_RIGHT)
            new InstantCommand(() -> drivetrain.setNextFeedPose(getGlobalPositions().CORAL_STATION_RIGHT)).ignoringDisable(true)
        );
    }

    private void configureBackupBindings() {
        operatorController.leftTrigger().whileTrue(
            new FeederManipulatorCommand(feeder, coralManipulator, armevator, 1.0, 0.3)
        );

        operatorController.rightTrigger().whileTrue(
            new FeedState(feeder, -1.0).alongWith(new CoralOutakeState(coralManipulator, 1.0))
        );

        operatorController.a().whileTrue(
            new GoToArmevatorPoseState(armevator, L1_ARMEVATOR_POSITION).repeatedly()
        );

        operatorController.b().whileTrue(
            new GoToArmevatorPoseState(armevator, L2_ARMEVATOR_POSITION).repeatedly()
        );

        operatorController.x().whileTrue(
            new GoToArmevatorPoseState(armevator, L3_ARMEVATOR_POSITION).repeatedly()
        );

        operatorController.y().whileTrue(
            new GoToArmevatorPoseState(armevator, L4_ARMEVATOR_POSITION).repeatedly()
        );

        operatorController.rightBumper().whileTrue(
            new GoToArmevatorPoseState(armevator, ALGAE_ARMEVATOR_POSITION).alongWith(
                new AlgaeIntake(algaeManipulator)
            )
        );

        operatorController.leftBumper().whileTrue(
            new GoToArmevatorPoseState(armevator, ALGAE_ARMEVATOR_POSITION_TWO).alongWith(
                new AlgaeIntake(algaeManipulator)
            )
        );

        operatorController.povUp().whileTrue(
            new LowerState(climber)
        );

        operatorController.povDown().whileTrue(
            new ClimbState(climber)
        );

        operatorController.povLeft().whileTrue(
            new BargeScoreCommand(armevator, algaeManipulator, () -> driverController.povLeft().getAsBoolean())
        );
    }

    public static void registerNamedCommands() {
        // NamedCommands.registerCommand("Drop", new DropState(dropper).withTimeout(0.5)); //ex
        NamedCommands.registerCommand(
            "Score L4",
            new SequentialCommandGroup(
                new GoToArmevatorPoseState(armevator, L4_ARMEVATOR_POSITION)
                    .raceWith(new IdleState(coralManipulator, armevator::getArmRotation)), 
                new WaitCommand(0.2),
                new CoralOutakeState(coralManipulator, 1).withTimeout(.25)
            )
                
        );

        NamedCommands.registerCommand("Go to L4", 
            new GoToArmevatorPoseState(armevator, L4_ARMEVATOR_POSITION)
                .raceWith(new IdleState(coralManipulator, armevator::getArmRotation))
        );

        NamedCommands.registerCommand("Go home", new GoToArmevatorPoseState(armevator, HOME).withTimeout(0.1));

        NamedCommands.registerCommand("Feed to L4", 
            new SequentialCommandGroup(   
                new FeederManipulatorCommand(
                    feeder, coralManipulator, armevator, 1, 0.2
                ),
                new GoToArmevatorPoseState(armevator, L4_ARMEVATOR_POSITION)
                    .raceWith(new IdleState(coralManipulator, armevator::getArmRotation))
            )
        );
        NamedCommands.registerCommand("Next", 
            new GoToNextArmevatorPoseState(armevator)
                .raceWith(new IdleState(coralManipulator, armevator::getArmRotation))
        );

        NamedCommands.registerCommand("Feed", 
            new FeederManipulatorCommand(
                feeder, coralManipulator, armevator, 1, 0.2
            )
        );

        NamedCommands.registerCommand("Autonomous Feed", 
            new AutonomousFeedTillFirstLidar(
                feeder, coralManipulator, armevator, 1, 0.2
            )
        );
    }

    private void setupAutos() {
        autoChooser = new SendableChooser<>();

        SIDE_CHOOSER.setDefaultOption("Red", "red");
        SIDE_CHOOSER.addOption("Blue", "blue");
    
        autoTab = Shuffleboard.getTab("Auto");
        autoTab.add("AutoChooser", autoChooser);
        autoTab.add("SideChooser", SIDE_CHOOSER);

        // autoChooser.addOption("Wheel Diam", new PathPlannerAuto("Wheel Diam"));
        autoChooser.addOption("Left Auto", new PathPlannerAuto("Left Two Piece auto", false));
        autoChooser.addOption("Right Auto", new PathPlannerAuto("Right Two Piece auto", false));
        autoChooser.addOption("Right three piece auto", new PathPlannerAuto("Right three piece auto"));
        autoChooser.setDefaultOption("Left three piece auto", new PathPlannerAuto("Left three piece auto"));
        autoChooser.setDefaultOption("Middle Auto", new PathPlannerAuto("Short Auto", false));
        // autoChooser.addOption("Odometry test", new PathPlannerAuto("Wheel Diam"));
        // autoChooser.setDefaultOption("Output name", new PathPlannerAuto("auto name", boolean mirror same field)); //ex
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
