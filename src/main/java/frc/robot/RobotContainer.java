// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.Arrays;
import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.auto.AutoCommons.AutoPaths;
import frc.robot.auto.AutoManager;
import frc.robot.auto.AutoSelector;
import frc.robot.auto.routines.DrivePastLine;
import frc.robot.auto.routines.ScoreAlgaeAndCoral;
import frc.robot.auto.routines.ScoreCoral;
import frc.robot.constants.FieldConstants.Barge;
import frc.robot.constants.FieldConstants.CoralStation;
import frc.robot.constants.FieldConstants.Processor;
import frc.robot.constants.FieldConstants.Reef;
import frc.robot.constants.FieldConstants.Reef.BranchLevel;
import frc.robot.constants.FieldConstants.Reef.StagedAlgaeLevel;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOFalcon;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOFalcon550;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.commands.WheelRadiusCalibration;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOFalcon;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.manualOverrides.ManualOverrides;
import frc.robot.subsystems.objectiveTracker.ObjectiveSelectorIO;
import frc.robot.subsystems.objectiveTracker.ObjectiveSelectorIOServer;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.RobotFlippedCommand;
import frc.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.ElevatorConstants;
import frc.robot.subsystems.superstructure.elevator.ElevatorIO;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOKraken;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOSim;
import frc.robot.subsystems.superstructure.pivot.Pivot;
import frc.robot.subsystems.superstructure.pivot.PivotConstants;
import frc.robot.subsystems.superstructure.pivot.PivotIO;
import frc.robot.subsystems.superstructure.pivot.PivotIOFalcon;
import frc.robot.subsystems.superstructure.pivot.PivotIOSim;
import frc.robot.subsystems.superstructure.wrist.Wrist;
import frc.robot.subsystems.superstructure.wrist.WristIO;
import frc.robot.subsystems.superstructure.wrist.WristIOKraken;
import frc.robot.subsystems.superstructure.wrist.WristIOSim;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.apriltag.ApriltagCamera;
import frc.robot.subsystems.vision.apriltag.ApriltagCameraIO;
import frc.robot.subsystems.vision.apriltag.ApriltagCameraIOPhotonVision;
import frc.robot.subsystems.vision.apriltag.ApriltagVision;
import frc.robot.subsystems.vision.apriltag.ApriltagVisionConstants;
import frc.robot.subsystems.vision.questnav.QuestNav;
import frc.robot.subsystems.vision.questnav.QuestNavConstants;
import frc.robot.subsystems.vision.questnav.QuestNavIO;
import frc.robot.subsystems.vision.questnav.QuestNavIOQuest3S;
import frc.robot.subsystems.vision.questnav.QuestNavIOSim;
import frc.util.Perspective;
import frc.util.commands.ContinuouslySwappingCommand;
import frc.util.controllers.ButtonBoard3x3;
import frc.util.controllers.XboxController;
import frc.util.robotStructure.Mechanism3d;

public class RobotContainer {
    // Subsystems
    public final Drive drive;
    public final Superstructure superstructure;
    public final Intake intake;
    public final Climber climber;
    public final ApriltagVision apriltagVision;
    public final QuestNav questNav;
    public final ManualOverrides manualOverrides;
    public final ObjectiveTracker objectiveTracker;

    public final AutoManager autoManager;

    // Controllers
    private final XboxController driveController = new XboxController(0);
    @SuppressWarnings("unused")
    private final ButtonBoard3x3 buttonBoard = new ButtonBoard3x3(1);
    @SuppressWarnings("unused")
    private final CommandJoystick simJoystick = new CommandJoystick(5);

    @SuppressWarnings("resource")
    public RobotContainer() {
        System.out.println("[Init RobotContainer] Creating " + RobotType.getMode().name() + " " + RobotType.getRobot().name());

        switch (RobotType.getMode()) {
            case REAL:
                drive = new Drive(
                    new GyroIOPigeon2(),
                    Arrays.stream(DriveConstants.moduleConstants)
                        .map(ModuleIOFalcon550::new)
                        .toArray(ModuleIO[]::new)
                );
                superstructure = new Superstructure(
                    new Pivot(new PivotIOFalcon()),
                    new Elevator(new ElevatorIOKraken()),
                    new Wrist(new WristIOKraken())
                );
                intake = new Intake(new IntakeIOFalcon());
                climber = new Climber(new ClimberIOFalcon());
                apriltagVision = new ApriltagVision(
                    new ApriltagCamera(
                        ApriltagVisionConstants.frontLeftApriltagCamera,
                        new ApriltagCameraIOPhotonVision(ApriltagVisionConstants.frontLeftApriltagCamera)
                    ),
                    new ApriltagCamera(
                        ApriltagVisionConstants.frontRightApriltagCamera,
                        new ApriltagCameraIOPhotonVision(ApriltagVisionConstants.frontRightApriltagCamera)
                    ),
                    new ApriltagCamera(
                        ApriltagVisionConstants.backLeftApriltagCamera,
                        new ApriltagCameraIOPhotonVision(ApriltagVisionConstants.backLeftApriltagCamera)
                    ),
                    new ApriltagCamera(
                        ApriltagVisionConstants.backRightApriltagCamera,
                        new ApriltagCameraIOPhotonVision(ApriltagVisionConstants.backRightApriltagCamera)
                    )
                );
                questNav = new QuestNav(QuestNavConstants.metaQuest3S, new QuestNavIOQuest3S());
                objectiveTracker = new ObjectiveTracker(new ObjectiveSelectorIOServer());
            break;
            case SIM:
                drive = new Drive(
                    new GyroIO() {},
                    Arrays.stream(DriveConstants.moduleConstants)
                        .map(ModuleIOSim::new)
                        .toArray(ModuleIO[]::new)
                );
                superstructure = new Superstructure(
                    new Pivot(new PivotIOSim()),
                    new Elevator(new ElevatorIOSim()),
                    new Wrist(new WristIOSim())
                );
                intake = new Intake(new IntakeIOSim(simJoystick.button(1), simJoystick.button(2)));
                climber = new Climber(new ClimberIO() {});
                apriltagVision = new ApriltagVision();
                questNav = new QuestNav(QuestNavConstants.metaQuest3S, new QuestNavIOSim());
                objectiveTracker = new ObjectiveTracker(new ObjectiveSelectorIOServer());
            break;
            default:
            case REPLAY:
                drive = new Drive(
                    new GyroIO() {},
                    new ModuleIO(){},
                    new ModuleIO(){},
                    new ModuleIO(){},
                    new ModuleIO(){}
                );
                superstructure = new Superstructure(
                    new Pivot(new PivotIO() {}),
                    new Elevator(new ElevatorIO() {}),
                    new Wrist(new WristIO() {})
                );
                intake = new Intake(new IntakeIO() {});
                climber = new Climber(new ClimberIO() {});
                apriltagVision = new ApriltagVision(
                    new ApriltagCamera(
                        ApriltagVisionConstants.frontLeftApriltagCamera,
                        new ApriltagCameraIO() {}
                    ),
                    new ApriltagCamera(
                        ApriltagVisionConstants.frontRightApriltagCamera,
                        new ApriltagCameraIO() {}
                    ),
                    new ApriltagCamera(
                        ApriltagVisionConstants.backLeftApriltagCamera,
                        new ApriltagCameraIO() {}
                    ),
                    new ApriltagCamera(
                        ApriltagVisionConstants.backRightApriltagCamera,
                        new ApriltagCameraIO() {}
                    )
                );
                questNav = new QuestNav(QuestNavConstants.metaQuest3S, new QuestNavIO() {});
                objectiveTracker = new ObjectiveTracker(new ObjectiveSelectorIO() {});
            break;
        }
        manualOverrides = new ManualOverrides();
        
        drive.structureRoot
            .addChild(VisionConstants.frontLeftMount)
            .addChild(VisionConstants.frontRightMount)
            .addChild(VisionConstants.backLeftMount)
            .addChild(VisionConstants.backRightMount)
            .addChild(VisionConstants.questNavMount)
            .addChild(superstructure.pivot.mech
                .addChild(superstructure.elevator.stage2Mech
                    .addChild(superstructure.elevator.stage3Mech
                        .addChild(superstructure.elevator.stage4Mech
                            .addChild(superstructure.wrist.mech
                                .addChild(intake.coralPose)
                                .addChild(intake.algaePose)
                            )
                        )
                    )
                )
            )
        ;
        Mechanism3d.registerMechs(superstructure.pivot.mech, superstructure.elevator.stage2Mech, superstructure.elevator.stage3Mech, superstructure.elevator.stage4Mech, superstructure.wrist.mech);

        System.out.println("[Init RobotContainer] Configuring Commands");
        configureCommands();

        System.out.println("[Init RobotContainer] Configuring Notifications");
        configureNotifications();

        System.out.println("[Init RobotContainer] Configuring Autonomous Modes");
        configureAutos();
        AutoPaths.preload();
        var selector = new AutoSelector("Auto Selector");
        selector.addDefaultRoutine(new ScoreCoral(this));
        selector.addRoutine(new ScoreAlgaeAndCoral(this));
        selector.addRoutine(new DrivePastLine(this));

        autoManager = new AutoManager(selector);

        System.out.println("[Init RobotContainer] Configuring System Check");
        configureSystemCheck();

        if (RobotConstants.tuningMode) {
            new Alert("Tuning mode active", AlertType.kInfo).set(true);
        }
    }

    private void configureCommands() {
        var driveJoystick = driveController.leftStick
            .smoothRadialDeadband(DriveConstants.driveJoystickDeadbandPercent)
            .radialSensitivity(0.75)
            // .radialSlewRateLimit(DriveConstants.joystickSlewRateLimit)
        ;

        var joystickTranslational = Drive.Translational.joystickSpectatorToFieldRelative(
            driveJoystick,
            () -> false
        );

        drive.translationSubsystem.setDefaultCommand(
            drive.translationSubsystem.run(() -> {
                var fieldVec = Perspective.getCurrent().toField(
                    driveJoystick.toVector()
                    .times(
                        DriveConstants.maxDriveSpeed.in(MetersPerSecond) * 
                        DriveConstants.maxDriveSpeedEnvCoef.getAsDouble()
                    )
                );
                var fieldSpeeds = new ChassisSpeeds(
                    fieldVec.get(0),
                    fieldVec.get(1),
                    0
                );
                ChassisSpeeds robotSpeeds;
                if (driveController.leftTrigger.getAsDouble() > 0.1 && driveController.rightTrigger.getAsDouble() > 0.1) {
                    robotSpeeds = new ChassisSpeeds(
                        Math.min(driveController.leftTrigger.getAsDouble(), driveController.rightTrigger.getAsDouble()) * DriveConstants.maxAdjustmentSpeed.in(MetersPerSecond),
                        0,
                        0
                    );
                } else {
                    robotSpeeds = new ChassisSpeeds(
                        0,
                        (driveController.leftTrigger.getAsDouble() - driveController.rightTrigger.getAsDouble()) * DriveConstants.maxAdjustmentSpeed.in(MetersPerSecond),
                        0
                    );
                }
                if (objectiveTracker.getCurrentObjective().filter((objective) -> objective.getTargetDirection().isForward()).isEmpty()) {
                    robotSpeeds = new ChassisSpeeds(
                        -robotSpeeds.vxMetersPerSecond,
                        robotSpeeds.vyMetersPerSecond,
                        robotSpeeds.omegaRadiansPerSecond
                    );
                }
                drive.translationSubsystem.driveVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(fieldSpeeds, drive.getRotation()).plus(robotSpeeds));
            })
            .withName("Driver Control Field Relative")
        );
        drive.rotationalSubsystem.setDefaultCommand(
            drive.rotationalSubsystem.spin(driveController.rightStick.x().smoothDeadband(0.2).multiply(DriveConstants.maxTurnRate.in(RadiansPerSecond)).multiply(0.5))
                .withName("Robot spin")
        );

        // superstructure.setDefaultCommand(superstructure.throttle(driveController.leftStick.y(), driveController.rightStick.y(), driveController.leftTrigger.add(driveController.rightTrigger.invert())));
        // superstructure.setDefaultCommand(superstructure.goToSetpointSequenced(SuperstructureState.fromParts(Degrees.of(90), ElevatorConstants.minLength, Degrees.of(90))));
        superstructure.setDefaultCommand(superstructure.goToSetpointSequenced(SuperstructureState.idle));
        intake.setDefaultCommand(intake.idle());
        // SmartDashboard.putData("Superstructure/Down", superstructure.goToSetpoint(SuperstructureState.newConstrained(Degrees.of(90), ElevatorConstants.minLengthPhysical, Degrees.of(-60))));
        // SmartDashboard.putData("Superstructure/Up", superstructure.goToSetpoint(SuperstructureState.newConstrained(Degrees.of(90), ElevatorConstants.minLengthPhysical, Degrees.of(60))));
        // driveController.leftStickButton().onTrue(Commands.runOnce(() -> drive.setPose(Pose2d.kZero)));
        // var flickStick = driveController.rightStick.roughRadialDeadband(0.85);
        // new Trigger(() -> flickStick.magnitude() > 0 && drive.rotationalSubsystem.getCurrentCommand() == null).onTrue(
        //     drive.rotationalSubsystem.headingFromJoystick(
        //         flickStick,
        //         new Rotation2d[]{
        //             // Cardinals
        //             Rotation2d.kZero,
        //             Rotation2d.kCCW_90deg,
        //             Rotation2d.k180deg,
        //             Rotation2d.kCW_90deg,
        //         },
        //         () -> RobotConstants.intakeForward
        //     )
        //     .withName("Flick Stick")
        // );

        // driveController.rightBumper().toggleOnTrue(new ContinuouslySwappingCommand(
        //     new Supplier<Command>() {
        //         private final Command[] commands = new Command[Rack.values().length * 2];
        //         {
        //             for (var rack : Rack.values()) {
        //                 commands[rack.ordinal() * 2] = superstructure.goToSetpointSequenced(SuperstructureState.fromAlgaeForward(rack.algaeLevel));
        //                 commands[rack.ordinal() * 2 + 1] = superstructure.goToSetpointSequenced(SuperstructureState.fromAlgaeBackward(rack.algaeLevel));
        //             }
        //         }
        //         public Command get() {
        //             var rack = Rack.Rack2;
        //             if (drive.getRotation().minus(rack.getAlgaePose().getOurs().getRotation().toRotation2d()).getCos() >= 0) {
        //                 return commands[rack.ordinal() * 2];
        //             } else {
        //                 return commands[rack.ordinal() * 2 + 1];
        //             }
        //         }
        //     },
        //     Set.of(superstructure)
        // ));

        // driveController.a().onTrue(Commands.runOnce(() -> objectiveTracker.toggleSelectedNode()));
        // driveController.povUp().onTrue(Commands.runOnce(() -> objectiveTracker.moveSelectedBranch(0, 1)));
        // driveController.povDown().onTrue(Commands.runOnce(() -> objectiveTracker.moveSelectedBranch(0, -1)));
        // driveController.povLeft().onTrue(Commands.runOnce(() -> objectiveTracker.moveSelectedBranch(-1, 0)));
        // driveController.povRight().onTrue(Commands.runOnce(() -> objectiveTracker.moveSelectedBranch(1, 0)));
        
        driveController.a().whileTrue(intake.eject()); //Eject
        final Command coralIntakeCommand = new ContinuouslySwappingCommand(
            new Supplier<Command>() {
                private final RobotFlippedCommand coralStationCommands = CoralStation.intakePosition.mapToCommand((state) -> superstructure.goToSetpointSequenced(state).raceWith(intake.intakeCoral().until(intake.hasCoral)));
                public Command get() {
                    return coralStationCommands.get(objectiveTracker.getIntakeCoralObjective().getTargetDirection());
                }
            },
            Set.of(superstructure, intake)
        ).withName("Intake Coral Station");
        CommandScheduler.getInstance().getDefaultButtonLoop().bind(() -> {
            if (driveController.hid.getBButtonPressed()) {
                if (coralIntakeCommand.isScheduled()) {
                    coralIntakeCommand.cancel();
                } else {
                    coralIntakeCommand.schedule();
                }
            }
        });
        final Command stagedAlgaeIntakeCommand = new ContinuouslySwappingCommand(
            new Supplier<Command>() {
                private final RobotFlippedCommand[] stagedAlgaeCommands = Arrays.stream(StagedAlgaeLevel.values()).map((level) -> level.intakeSuperstructureStates.mapToCommand((state) -> superstructure.goToSetpointSequenced(state).alongWith(intake.intakeAlgae()))).toArray(RobotFlippedCommand[]::new);;
                public Command get() {
                    var optStagedAlgaeObjective = objectiveTracker.getIntakeAlgaeObjective();
                    if (optStagedAlgaeObjective.isEmpty()) {
                        return new InstantCommand();
                    } else {
                        var stagedAlgaeObjective = optStagedAlgaeObjective.get();
                        return stagedAlgaeCommands[stagedAlgaeObjective.algae.level.ordinal()].get(stagedAlgaeObjective.getTargetDirection());
                    }
                }
            },
            Set.of(superstructure, intake)
        ).withName("Intake Staged Algae");
        final Command groundAlgaeIntakeCommand = superstructure.goToSetpointSequenced(SuperstructureState.fromParts(PivotConstants.minAngle, ElevatorConstants.minLengthPhysical, Degrees.of(-30))).alongWith(intake.intakeAlgae().until(intake.hasAlgae)).withName("Intake Ground Algae");
        final Timer algaeIntakeButtonTimer = new Timer();
        CommandScheduler.getInstance().getDefaultButtonLoop().bind(() -> {
            if (driveController.hid.getYButtonPressed()) {
                if (stagedAlgaeIntakeCommand.isScheduled()) {
                    stagedAlgaeIntakeCommand.cancel();
                } else if (groundAlgaeIntakeCommand.isScheduled()) {
                    groundAlgaeIntakeCommand.cancel();
                } else {
                    algaeIntakeButtonTimer.start();
                }
            }
            if (driveController.hid.getYButtonReleased()) {
                if (!algaeIntakeButtonTimer.hasElapsed(1) && algaeIntakeButtonTimer.isRunning()) {
                    stagedAlgaeIntakeCommand.schedule();
                }
                algaeIntakeButtonTimer.stop();
                algaeIntakeButtonTimer.reset();
            }
            if (algaeIntakeButtonTimer.hasElapsed(1)) {
                groundAlgaeIntakeCommand.schedule();
                algaeIntakeButtonTimer.stop();
                algaeIntakeButtonTimer.reset();
            }
        });

        final Command coralScoreCommand = new ContinuouslySwappingCommand(
            new Supplier<Command>() {
                private final RobotFlippedCommand[] branchCommands = Arrays.stream(BranchLevel.values()).map((level) -> level.scoringSuperstructureStates.mapToCommand((state) -> superstructure.goToSetpointSequenced(state))).toArray(RobotFlippedCommand[]::new);
                private final Command level1 = Commands.none();
                public Command get() {
                    var scoreCoralObjective = objectiveTracker.getScoreCoralObjective();
                    if (scoreCoralObjective.branchLevel.isPresent()) {
                        return branchCommands[scoreCoralObjective.branchLevel.get().ordinal()].get(scoreCoralObjective.getTargetDirection());
                    } else {
                        return level1;
                    }
                }
            },
            Set.of(superstructure)
        ).withName("Extend to Reef");
        final Command algaeScoreCommand = new ContinuouslySwappingCommand(
            new Supplier<Command>() {
                private final RobotFlippedCommand netCommands = Barge.superstructureState.mapToCommand((state) -> superstructure.goToSetpointSequenced(state));
                private final Command processorCommand = superstructure.goToSetpointSequenced(Processor.superstructureState.getForward());
                public Command get() {
                    switch (objectiveTracker.getScoreAlgaeObjective().algaeGoal) {
                        default:
                        case NET:
                            return netCommands.get(objectiveTracker.getScoreAlgaeObjective().getTargetDirection());
                        case PROCESSOR:
                        case OPPONENT_PROCESSOR:
                            return processorCommand;
                    }
                }
            },
            Set.of(superstructure)
        ).withName("Extend to Algae Goal");

        CommandScheduler.getInstance().getDefaultButtonLoop().bind(() -> {
            if (driveController.hid.getXButtonPressed()) {
                if (!coralScoreCommand.isScheduled() && !algaeScoreCommand.isScheduled()) {
                    if (intake.hasCoral.getAsBoolean()) {
                        coralScoreCommand.schedule();
                    }
                    if (intake.hasAlgae.getAsBoolean()) {
                        algaeScoreCommand.schedule();
                    }
                } else {
                    if (coralScoreCommand.isScheduled()) {
                        coralScoreCommand.cancel();
                    }
                    if (algaeScoreCommand.isScheduled()) {
                        algaeScoreCommand.cancel();
                    }
                }
            }
        });
        driveController.leftBumper().and(() -> objectiveTracker.getCurrentObjective().isPresent()).whileTrue(drive.rotationalSubsystem.pidControlledHeading(() -> objectiveTracker.getCurrentObjective().get().getTargetPose().getRotation()));
        driveController.rightBumper().and(() -> objectiveTracker.getCurrentObjective().isPresent()).whileTrue(drive.simplePIDTo(() -> objectiveTracker.getCurrentObjective().get().getTargetPose())); //Auto drive
        // driveController.start().toggleOnTrue(
        //     Commands.parallel(
        //         climber.prepareClimb(),
        //         superstructure.prepareClimb()
        //     )
        // ); //Start Climb
        // driveController.back().toggleOnTrue(
        //     Commands.parallel(
        //         superstructure.climb(),
        //         Commands.sequence(
        //             climber.climb().until(() -> superstructure.pivot.getAngle().lt(Degrees.of(21))),
        //             climber.hold()
        //         )
        //     )
        // );
        
        driveController.leftStickButton().onTrue(Commands.runOnce(() -> drive.setPose(Reef.reefs.getOurs().racks[0].centerRobotPose.getForward())).ignoringDisable(true));

        SmartDashboard.putData("QuestNav/Quest Calibrate", questNav.determineOffsetToRobotCenter(drive));
    }

    private void configureNotifications() {}

    private void configureAutos() {
        
    }

    private void configureSystemCheck() {
        SmartDashboard.putData("System Check/Drive/Spin", 
            new Command() {
                private final Drive.Rotational rotationalSubsystem = drive.rotationalSubsystem;
                private final Timer timer = new Timer();
                {
                    addRequirements(rotationalSubsystem);
                    setName("TEST Spin");
                }
                public void initialize() {
                    timer.restart();
                }
                public void execute() {
                    rotationalSubsystem.driveVelocity(Math.sin(timer.get()) * 3);
                }
                public void end(boolean interrupted) {
                    timer.stop();
                    rotationalSubsystem.stop();
                }
            }
        );
        SmartDashboard.putData("System Check/Drive/Circle", 
            new Command() {
                private final Drive.Translational translationSubsystem = drive.translationSubsystem;
                private final Timer timer = new Timer();
                {
                    addRequirements(translationSubsystem);
                    setName("TEST Circle");
                }
                public void initialize() {
                    timer.restart();
                }
                public void execute() {
                    translationSubsystem.driveVelocity(
                        new ChassisSpeeds(
                            Math.cos(timer.get()) * 0.01,
                            Math.sin(timer.get()) * 0.01,
                            0
                        )
                    );
                }
                public void end(boolean interrupted) {
                    timer.stop();
                    translationSubsystem.stop();
                }
            }
        );
        
        SmartDashboard.putData("Wheel Calibration", Commands.defer(() -> 
            new WheelRadiusCalibration(
                drive,
                (int)WheelRadiusCalibration.MAX_SAMPLES.get(),
                WheelRadiusCalibration.SAMPLE_PERIOD.get(),
                WheelRadiusCalibration.VOLTAGE_RAMP_RATE.get(),
                WheelRadiusCalibration.MAX_VOLTAGE.get()
            ).withName("Wheel Calibration"),
            Set.of(drive.translationSubsystem, drive.rotationalSubsystem))
        );
    }
}
