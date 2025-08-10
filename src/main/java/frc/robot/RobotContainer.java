// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.Arrays;
import java.util.Set;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
import frc.robot.subsystems.drive.commands.AutoScore;
import frc.robot.subsystems.drive.commands.WheelRadiusCalibration;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOFalcon;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.manualOverrides.ManualOverrides;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker;
import frc.robot.subsystems.objectiveTracker.ReefTrackerIO;
import frc.robot.subsystems.objectiveTracker.ReefTrackerIOServer;
import frc.robot.subsystems.objectiveTracker.objectives.Objective.ObjectiveType;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.RobotFlippedCommand;
import frc.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import frc.robot.subsystems.superstructure.SuperstructureConstants;
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
import frc.util.EdgeDetector;
import frc.util.Environment;
import frc.util.Perspective;
import frc.util.commands.ContinuouslySwappingCommand;
import frc.util.controllers.ButtonBoard3x3;
import frc.util.controllers.XboxController;
import frc.util.geometry.GeomUtil;
import frc.util.loggerUtil.tunables.LoggedTunableMeasure;
import frc.util.misc.MeasureUtil;
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
                        new ApriltagCameraIOPhotonVision(ApriltagVisionConstants.frontLeftApriltagCamera),
                        Leds.getInstance().flAprilConnection
                    ),
                    new ApriltagCamera(
                        ApriltagVisionConstants.frontRightApriltagCamera,
                        new ApriltagCameraIOPhotonVision(ApriltagVisionConstants.frontRightApriltagCamera),
                        Leds.getInstance().frAprilConnection
                    ),
                    new ApriltagCamera(
                        ApriltagVisionConstants.backLeftApriltagCamera,
                        new ApriltagCameraIOPhotonVision(ApriltagVisionConstants.backLeftApriltagCamera),
                        Leds.getInstance().blAprilConnection
                    ),
                    new ApriltagCamera(
                        ApriltagVisionConstants.backRightApriltagCamera,
                        new ApriltagCameraIOPhotonVision(ApriltagVisionConstants.backRightApriltagCamera),
                        Leds.getInstance().brAprilConnection
                    )
                );
                questNav = new QuestNav(QuestNavConstants.metaQuest3S, new QuestNavIOQuest3S(), Leds.getInstance().questNavConnection);
                objectiveTracker = new ObjectiveTracker(new ReefTrackerIOServer());
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
                // intake = new Intake(new IntakeIOSim(simJoystick.button(1), simJoystick.button(2)));
                intake = new Intake(new IntakeIOSim(driveController.povDown(), simJoystick.button(2)));
                climber = new Climber(new ClimberIO() {});
                apriltagVision = new ApriltagVision(
                    new ApriltagCamera(
                        ApriltagVisionConstants.frontLeftApriltagCamera,
                        new ApriltagCameraIO() {},
                        Leds.getInstance().flAprilConnection
                    ),
                    new ApriltagCamera(
                        ApriltagVisionConstants.frontRightApriltagCamera,
                        new ApriltagCameraIO() {},
                        Leds.getInstance().frAprilConnection
                    ),
                    new ApriltagCamera(
                        ApriltagVisionConstants.backLeftApriltagCamera,
                        new ApriltagCameraIO() {},
                        Leds.getInstance().blAprilConnection
                    ),
                    new ApriltagCamera(
                        ApriltagVisionConstants.backRightApriltagCamera,
                        new ApriltagCameraIO() {},
                        Leds.getInstance().brAprilConnection
                    )
                );
                questNav = new QuestNav(QuestNavConstants.metaQuest3S, new QuestNavIOSim(), Leds.getInstance().questNavConnection);
                objectiveTracker = new ObjectiveTracker(new ReefTrackerIOServer());
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
                        new ApriltagCameraIO() {},
                        Leds.getInstance().flAprilConnection
                    ),
                    new ApriltagCamera(
                        ApriltagVisionConstants.frontRightApriltagCamera,
                        new ApriltagCameraIO() {},
                        Leds.getInstance().frAprilConnection
                    ),
                    new ApriltagCamera(
                        ApriltagVisionConstants.backLeftApriltagCamera,
                        new ApriltagCameraIO() {},
                        Leds.getInstance().blAprilConnection
                    ),
                    new ApriltagCamera(
                        ApriltagVisionConstants.backRightApriltagCamera,
                        new ApriltagCameraIO() {},
                        Leds.getInstance().brAprilConnection
                    )
                );
                questNav = new QuestNav(QuestNavConstants.metaQuest3S, new QuestNavIO() {}, Leds.getInstance().questNavConnection);
                objectiveTracker = new ObjectiveTracker(new ReefTrackerIO() {});
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
            .addChild(climber.mech)
        ;
        Mechanism3d.registerMechs(superstructure.pivot.mech, superstructure.elevator.stage2Mech, superstructure.elevator.stage3Mech, superstructure.elevator.stage4Mech, superstructure.wrist.mech, climber.mech);

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

        var joystickTranslational = Drive.Translational.joystickSpectatorToFieldRelative(driveJoystick);

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
            drive.rotationalSubsystem.spin(driveController.rightStick.x().smoothDeadband(0.1).multiply(DriveConstants.maxTurnRate.in(RadiansPerSecond)).multiply(0.5))
                .withName("Robot spin")
        );
        new Trigger(DriverStation::isDisabled).and(() -> driveJoystick.magnitude() > 0).whileTrue(drive.coast());

        superstructure.setDefaultCommand(superstructure.goToSetpointSequenced(SuperstructureConstants.idleState));
        intake.setDefaultCommand(intake.idle());
        climber.setDefaultCommand(climber.idle());

        driveController.povUp().onTrue(Commands.runOnce(() -> objectiveTracker.shiftLevelLock(1)));
        driveController.povDown().onTrue(Commands.runOnce(() -> objectiveTracker.shiftLevelLock(-1)));
        driveController.povLeft().onTrue(Commands.runOnce(() -> objectiveTracker.shiftPipeLock(-1)));
        driveController.povRight().onTrue(Commands.runOnce(() -> objectiveTracker.shiftPipeLock(1)));
        
        driveController.a().whileTrue(new ContinuouslySwappingCommand(
            new Supplier<Command>() {
                private final Command eject = intake.eject();
                private final Command ejectL1 = intake.ejectLevel1();
                private final Command ejectAlgae = intake.ejectAlgae();
                public Command get() {
                    if (intake.hasAlgae.getAsBoolean()) {
                        return ejectAlgae;
                    } else if (intake.hasCoral.getAsBoolean() && objectiveTracker.getScoreCoralObjective().getTargetBranch().isEmpty()) {
                        return ejectL1;
                    } else {
                        return eject;
                    }
                }
            },
            Set.of(intake)
        )); //Eject
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
                private final Command idle = superstructure.goToSetpointSequenced(SuperstructureConstants.idleState).alongWith(intake.idle());
                public Command get() {
                    var optStagedAlgaeObjective = objectiveTracker.getIntakeAlgaeObjective();
                    if (optStagedAlgaeObjective.isEmpty()) {
                        return idle;
                    } else {
                        var stagedAlgaeObjective = optStagedAlgaeObjective.get();
                        return stagedAlgaeCommands[stagedAlgaeObjective.getTargetAlgae().level.ordinal()].get(stagedAlgaeObjective.getTargetDirection());
                    }
                }
            },
            Set.of(superstructure, intake)
        ).deadlineFor(objectiveTracker.setTypeOverrideCommand(ObjectiveType.IntakeAlgae)).withName("Intake Staged Algae");
        final Command groundAlgaeIntakeCommand = superstructure.goToSetpointSequenced(SuperstructureState.fromParts(PivotConstants.minAngle, ElevatorConstants.minLengthPhysical, Degrees.of(-35))).alongWith(intake.intakeAlgae()).withName("Intake Ground Algae");
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
                if (!algaeIntakeButtonTimer.hasElapsed(0.25) && algaeIntakeButtonTimer.isRunning()) {
                    stagedAlgaeIntakeCommand.schedule();
                }
                algaeIntakeButtonTimer.stop();
                algaeIntakeButtonTimer.reset();
            }
            if (algaeIntakeButtonTimer.hasElapsed(0.25)) {
                groundAlgaeIntakeCommand.schedule();
                algaeIntakeButtonTimer.stop();
                algaeIntakeButtonTimer.reset();
            }
        });

        final Command coralScoreCommand = new ContinuouslySwappingCommand(
            new Supplier<Command>() {
                private final RobotFlippedCommand[] branchCommands = Arrays.stream(BranchLevel.values()).map((level) -> level.scoringSuperstructureStates.mapToCommand((state) -> superstructure.goToSetpointSequenced(state))).toArray(RobotFlippedCommand[]::new);
                private final RobotFlippedCommand level1Command = Reef.level1SuperstructureStates.mapToCommand((state) -> superstructure.goToSetpointSequenced(state));
                public Command get() {
                    var scoreCoralObjective = objectiveTracker.getScoreCoralObjective();
                    if (scoreCoralObjective.getTargetBranch().isPresent()) {
                        return branchCommands[scoreCoralObjective.getTargetBranch().get().level.ordinal()].get(scoreCoralObjective.getTargetDirection());
                    } else {
                        return level1Command.get(scoreCoralObjective.getTargetDirection());
                    }
                }
            },
            Set.of(superstructure)
        ).deadlineFor(
            Commands.startEnd(
                () -> objectiveTracker.addLevelLock(objectiveTracker.getScoreCoralObjective().getTargetBranch().map((branch) -> branch.level)),
                () -> objectiveTracker.removeLevelLock()
            )
        ).withName("Extend to Reef");
        final Command algaeScoreCommand = new ContinuouslySwappingCommand(
            new Supplier<Command>() {
                private final RobotFlippedCommand netCommands = Barge.superstructureState.mapToCommand((state) -> superstructure.goToSetpointSequenced(state));
                private final Command processorCommand = superstructure.goToSetpointSequenced(Processor.superstructureState.getForward());
                public Command get() {
                    if (objectiveTracker.getScoreAlgaeObjective().isProcessor()) {
                        return processorCommand;
                    } else {
                        return netCommands.get(objectiveTracker.getScoreAlgaeObjective().getTargetDirection());
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
        driveController.leftBumper().and(() -> objectiveTracker.getCurrentObjective().isPresent()).whileTrue(drive.rotationalSubsystem.pidControlledHeading(() -> objectiveTracker.getCurrentObjective().get().getTargetPose().getOurs().getRotation()));

        final Command autoDriveScoreCoral = this.drive.simplePIDTo(
            () -> AutoScore.getTargetPose(
                this.drive.getPose(),
                this.objectiveTracker.getScoreCoralObjective().getTargetPose().getOurs()
            )
        ).deadlineFor(
            Commands.startEnd(
                () -> {
                    if (objectiveTracker.getScoreCoralObjective().getTargetBranch().isPresent()) {
                        objectiveTracker.addPipeLock(objectiveTracker.getScoreCoralObjective().getTargetBranch().get().pipe);
                    }
                },
                () -> {
                    objectiveTracker.removePipeLock();
                }
            )
        );
        final Command autoDriveIntakeAlgae = this.drive.simplePIDTo(
            () -> AutoScore.getTargetPose(
                this.drive.getPose(),
                this.objectiveTracker.getIntakeAlgaeObjective().get().getTargetPose().getOurs()
            )
        );
        final Command autoDriveIntakeCoral = this.drive.simplePIDTo(() -> this.objectiveTracker.getIntakeCoralObjective().getTargetPose().getOurs());
        final Command autoDriveScoreAlgae = this.drive.simplePIDTo(() -> this.objectiveTracker.getScoreAlgaeObjective().getTargetPose().getOurs());
        final Command autoDriveClimb = this.drive.simplePIDTo(() -> this.objectiveTracker.getClimbObjective().getTargetPose().getOurs());
        CommandScheduler.getInstance().getDefaultButtonLoop().bind(() -> {
            if (driveController.hid.getRightBumperButtonPressed()) {
                if (this.objectiveTracker.getCurrentObjective().isEmpty()) return;
                switch (this.objectiveTracker.getCurrentObjective().get().getObjectiveType()) {
                    case ScoreCoral: autoDriveScoreCoral.schedule(); break;
                    case IntakeAlgae: autoDriveIntakeAlgae.schedule(); break;
                    case IntakeCoral: autoDriveIntakeCoral.schedule(); break;
                    case ScoreAlgae: autoDriveScoreAlgae.schedule(); break;
                    case Climb: autoDriveClimb.schedule(); break;
                }
            } else if (driveController.hid.getRightBumperButtonReleased()) {
                if (autoDriveScoreCoral.isScheduled()) {
                    autoDriveScoreCoral.cancel();
                }
                if (autoDriveIntakeAlgae.isScheduled()) {
                    autoDriveIntakeAlgae.cancel();
                }
                if (autoDriveIntakeCoral.isScheduled()) {
                    autoDriveIntakeCoral.cancel();
                }
                if (autoDriveScoreAlgae.isScheduled()) {
                    autoDriveScoreAlgae.cancel();
                }
                if (autoDriveClimb.isScheduled()) {
                    autoDriveClimb.cancel();
                }
            }
        });
        driveController.start().toggleOnTrue(
            Commands.parallel(
                climber.prepareClimb(),
                superstructure.goToSetpointSequenced(SuperstructureConstants.prepareClimbingState)
            )
            .deadlineFor(
                objectiveTracker.setTypeOverrideCommand(ObjectiveType.Climb)
            )
            // climber.testDisengageRatchet()
        );
        driveController.back().toggleOnTrue(
            Commands.parallel(
                climber.climb(),
                superstructure.goToSetpointSequenced(SuperstructureConstants.climbingState)
            )
            .deadlineFor(
                objectiveTracker.setTypeOverrideCommand(ObjectiveType.Climb)
            )
        );

        var selfRightCommand = superstructure.goToSetpointSequenced(SuperstructureConstants.selfRightingState);
        // var prepareSelfRightCommand = superstructure.goToSetpointSequenced(SuperstructureConstants.prepareSelfRightingState);
        CommandScheduler.getInstance().getDefaultButtonLoop().bind(new Runnable() {
            private boolean prevSelfRight = true;
            // private boolean prevprepare = true;
            public void run() {
                var selfRightButton = driveController.hid.getPOV() == 0;
                // var prepare = driveController.hid.getPOV() == 90;
                var tipped = !MeasureUtil.isNear(Degrees.of(0), drive.getPitch(), Degrees.of(45));
                Leds.getInstance().tipped.setFlag(tipped);
                if (selfRightButton && !prevSelfRight) {
                    if (selfRightCommand.isScheduled()) {
                        selfRightCommand.cancel();
                    } else {
                        if (tipped) {
                            selfRightCommand.schedule();
                        }
                    }
                }
                if (selfRightCommand.isScheduled() && !tipped) {
                    selfRightCommand.cancel();
                }
                // if (prepare && !prevprepare) {
                //     if (prepareSelfRightCommand.isScheduled()) {
                //         prepareSelfRightCommand.cancel();
                //     } else {
                //         // if (tipped) {
                //             prepareSelfRightCommand.schedule();
                //         // }
                //     }
                // }
                prevSelfRight = selfRightButton;
                // prevprepare = prepare;
            }
        });
        
        driveController.leftStickButton().and(driveController.rightStickButton()).onTrue(Commands.runOnce(() -> this.setPose(Reef.reefs.getOurs().racks[0].centerRobotPose.getForward())).ignoringDisable(true));
        new Trigger(() -> apriltagVision.getPose().xyStdDev() < .5)
            .onTrue(Commands.runOnce(() -> this.setPose(apriltagVision.getPose().robotPose())));

        SmartDashboard.putData("QuestNav/Quest Calibrate", questNav.determineOffsetToRobotCenter(drive));

        SmartDashboard.putData("Superstructure/Coast", this.superstructure.coast());

        CommandScheduler.getInstance().getDefaultButtonLoop().bind(new Runnable() {
            private static final LoggedTunableMeasure<AngleUnit> l4PivotTolerance = new LoggedTunableMeasure<>("Self Record/Coral/L4/Superstructure/Pivot Tolerance", Degrees.of(2));
            private static final LoggedTunableMeasure<DistanceUnit> l4ElevatorTolerance = new LoggedTunableMeasure<>("Self Record/Coral/L4/Superstructure/Elevator Tolerance", Inches.of(2));
            private static final LoggedTunableMeasure<AngleUnit> l4WristTolerance = new LoggedTunableMeasure<>("Self Record/Coral/L4/Superstructure/Wrist Tolerance", Degrees.of(5));
            private static final LoggedTunableMeasure<DistanceUnit> l4LinearTolerance = new LoggedTunableMeasure<>("Self Record/Coral/L4/Robot/Linear Tolerance", Inches.of(2));
            private static final LoggedTunableMeasure<AngleUnit> l4AngularTolerance = new LoggedTunableMeasure<>("Self Record/Coral/L4/Robot/Angular Tolerance", Degrees.of(5));
            
            private static final LoggedTunableMeasure<AngleUnit> l3PivotTolerance = new LoggedTunableMeasure<>("Self Record/Coral/L3/Superstructure/Pivot Tolerance", Degrees.of(2));
            private static final LoggedTunableMeasure<DistanceUnit> l3ElevatorTolerance = new LoggedTunableMeasure<>("Self Record/Coral/L3/Superstructure/Elevator Tolerance", Inches.of(2));
            private static final LoggedTunableMeasure<AngleUnit> l3WristTolerance = new LoggedTunableMeasure<>("Self Record/Coral/L3/Superstructure/Wrist Tolerance", Degrees.of(5));
            private static final LoggedTunableMeasure<DistanceUnit> l3LinearTolerance = new LoggedTunableMeasure<>("Self Record/Coral/L3/Robot/Linear Tolerance", Inches.of(2));
            private static final LoggedTunableMeasure<AngleUnit> l3AngularTolerance = new LoggedTunableMeasure<>("Self Record/Coral/L3/Robot/Angular Tolerance", Degrees.of(5));
            
            private static final LoggedTunableMeasure<AngleUnit> l2PivotTolerance = new LoggedTunableMeasure<>("Self Record/Coral/L2/Superstructure/Pivot Tolerance", Degrees.of(2));
            private static final LoggedTunableMeasure<DistanceUnit> l2ElevatorTolerance = new LoggedTunableMeasure<>("Self Record/Coral/L2/Superstructure/Elevator Tolerance", Inches.of(2));
            private static final LoggedTunableMeasure<AngleUnit> l2WristTolerance = new LoggedTunableMeasure<>("Self Record/Coral/L2/Superstructure/Wrist Tolerance", Degrees.of(5));
            private static final LoggedTunableMeasure<DistanceUnit> l2LinearTolerance = new LoggedTunableMeasure<>("Self Record/Coral/L2/Robot/Linear Tolerance", Inches.of(2));
            private static final LoggedTunableMeasure<AngleUnit> l2AngularTolerance = new LoggedTunableMeasure<>("Self Record/Coral/L2/Robot/Angular Tolerance", Degrees.of(5));
            
            private static final LoggedTunableMeasure<AngleUnit> l1PivotTolerance = new LoggedTunableMeasure<>("Self Record/Coral/L1/Superstructure/Pivot Tolerance", Degrees.of(5));
            private static final LoggedTunableMeasure<DistanceUnit> l1ElevatorTolerance = new LoggedTunableMeasure<>("Self Record/Coral/L1/Superstructure/Elevator Tolerance", Inches.of(2));
            private static final LoggedTunableMeasure<AngleUnit> l1WristTolerance = new LoggedTunableMeasure<>("Self Record/Coral/L1/Superstructure/Wrist Tolerance", Degrees.of(5));
            private static final LoggedTunableMeasure<DistanceUnit> l1LinearTolerance = new LoggedTunableMeasure<>("Self Record/Coral/L1/Robot/Linear Tolerance", Inches.of(6));
            private static final LoggedTunableMeasure<AngleUnit> l1AngularTolerance = new LoggedTunableMeasure<>("Self Record/Coral/L1/Robot/Angular Tolerance", Degrees.of(10));
            
            private final EdgeDetector coralEdgeDetector = new EdgeDetector();
            @Override
            public void run() {
                this.coralEdgeDetector.update(intake.hasCoral.getAsBoolean());
                if (manualOverrides.selfRecordCoralDisabled()) {return;}

                if (this.coralEdgeDetector.fallingEdge()) {
                    var scoreCoralObjective = objectiveTracker.getScoreCoralObjective();
                    final Measure<AngleUnit> pivotTolerance;
                    final Measure<DistanceUnit> elevatorTolerance;
                    final Measure<AngleUnit> wristTolerance;
                    final Measure<DistanceUnit> linearTolerance;
                    final Measure<AngleUnit> angularTolerance;
                    if (scoreCoralObjective.getTargetBranch().isPresent()) {
                        switch (scoreCoralObjective.getTargetBranch().get().level) {
                            case Level2:
                                pivotTolerance = l2PivotTolerance.get();
                                elevatorTolerance = l2ElevatorTolerance.get();
                                wristTolerance = l2WristTolerance.get();
                                linearTolerance = l2LinearTolerance.get();
                                angularTolerance = l2AngularTolerance.get();
                            break;
                            case Level3:
                                pivotTolerance = l3PivotTolerance.get();
                                elevatorTolerance = l3ElevatorTolerance.get();
                                wristTolerance = l3WristTolerance.get();
                                linearTolerance = l3LinearTolerance.get();
                                angularTolerance = l3AngularTolerance.get();
                            break;
                            case Level4: default:
                                pivotTolerance = l4PivotTolerance.get();
                                elevatorTolerance = l4ElevatorTolerance.get();
                                wristTolerance = l4WristTolerance.get();
                                linearTolerance = l4LinearTolerance.get();
                                angularTolerance = l4AngularTolerance.get();
                            break;
                        }
                    } else {
                        pivotTolerance = l1PivotTolerance.get();
                        elevatorTolerance = l1ElevatorTolerance.get();
                        wristTolerance = l1WristTolerance.get();
                        linearTolerance = l1LinearTolerance.get();
                        angularTolerance = l1AngularTolerance.get();
                    }
                    Logger.recordOutput("Self Record/Coral/Superstructure/Pivot", MeasureUtil.isNear(scoreCoralObjective.getTargetState().pivotAngle, superstructure.getCurrentState().pivotAngle, pivotTolerance));
                    Logger.recordOutput("Self Record/Coral/Superstructure/Elevator", MeasureUtil.isNear(scoreCoralObjective.getTargetState().elevatorLength, superstructure.getCurrentState().elevatorLength, elevatorTolerance));
                    Logger.recordOutput("Self Record/Coral/Superstructure/Wrist", MeasureUtil.isNear(scoreCoralObjective.getTargetState().wristAngle, superstructure.getCurrentState().wristAngle, wristTolerance));
                    Logger.recordOutput("Self Record/Coral/Robot/Linear", GeomUtil.isNear(scoreCoralObjective.getTargetPose().getOurs().getTranslation(), drive.getPose().getTranslation(), linearTolerance));
                    Logger.recordOutput("Self Record/Coral/Robot/Angular", GeomUtil.isNear(scoreCoralObjective.getTargetPose().getOurs().getRotation(), drive.getPose().getRotation(), angularTolerance));
                    if (
                        GeomUtil.isNear(scoreCoralObjective.getTargetPose().getOurs(), drive.getPose(), linearTolerance, angularTolerance)
                        && superstructure.getCurrentState().isNear(scoreCoralObjective.getTargetState(), pivotTolerance, elevatorTolerance, wristTolerance)
                    ) {
                        objectiveTracker.placeCoral(scoreCoralObjective.getTargetBranch());
                    }
                }
            }
        });
        CommandScheduler.getInstance().getDefaultButtonLoop().bind(new Runnable() {
            private static final LoggedTunableMeasure<AngleUnit> lowPivotTolerance = new LoggedTunableMeasure<>("Self Record/Algae/Low/Superstructure/Pivot Tolerance", Degrees.of(5));
            private static final LoggedTunableMeasure<DistanceUnit> lowElevatorTolerance = new LoggedTunableMeasure<>("Self Record/Algae/Low/Superstructure/Elevator Tolerance", Inches.of(4));
            private static final LoggedTunableMeasure<AngleUnit> lowWristTolerance = new LoggedTunableMeasure<>("Self Record/Algae/Low/Superstructure/Wrist Tolerance", Degrees.of(15));
            private static final LoggedTunableMeasure<DistanceUnit> lowLinearTolerance = new LoggedTunableMeasure<>("Self Record/Algae/Low/Robot/Linear Tolerance", Inches.of(12));
            private static final LoggedTunableMeasure<AngleUnit> lowAngularTolerance = new LoggedTunableMeasure<>("Self Record/Algae/Low/Robot/Angular Tolerance", Degrees.of(30));
            
            private static final LoggedTunableMeasure<AngleUnit> highPivotTolerance = new LoggedTunableMeasure<>("Self Record/Algae/High/Superstructure/Pivot Tolerance", Degrees.of(5));
            private static final LoggedTunableMeasure<DistanceUnit> highElevatorTolerance = new LoggedTunableMeasure<>("Self Record/Algae/High/Superstructure/Elevator Tolerance", Inches.of(4));
            private static final LoggedTunableMeasure<AngleUnit> highWristTolerance = new LoggedTunableMeasure<>("Self Record/Algae/High/Superstructure/Wrist Tolerance", Degrees.of(15));
            private static final LoggedTunableMeasure<DistanceUnit> highLinearTolerance = new LoggedTunableMeasure<>("Self Record/Algae/High/Robot/Linear Tolerance", Inches.of(12));
            private static final LoggedTunableMeasure<AngleUnit> highAngularTolerance = new LoggedTunableMeasure<>("Self Record/Algae/High/Robot/Angular Tolerance", Degrees.of(30));

            private final EdgeDetector algaeEdgeDetector = new EdgeDetector();
            @Override
            public void run() {
                this.algaeEdgeDetector.update(intake.hasAlgae.getAsBoolean());
                if (manualOverrides.selfRecordAlgaeDisabled()) {return;}

                if (this.algaeEdgeDetector.risingEdge()) {
                    var intakeAlgaeObjective = objectiveTracker.getIntakeAlgaeObjective();
                    if (intakeAlgaeObjective.isEmpty()) {return;}
                    final Measure<AngleUnit> pivotTolerance;
                    final Measure<DistanceUnit> elevatorTolerance;
                    final Measure<AngleUnit> wristTolerance;
                    final Measure<DistanceUnit> linearTolerance;
                    final Measure<AngleUnit> angularTolerance;
                    switch (intakeAlgaeObjective.get().getTargetAlgae().level) {
                        case Low: default:
                            pivotTolerance = lowPivotTolerance.get();
                            elevatorTolerance = lowElevatorTolerance.get();
                            wristTolerance = lowWristTolerance.get();
                            linearTolerance = lowLinearTolerance.get();
                            angularTolerance = lowAngularTolerance.get();
                        break;
                        case High:
                            pivotTolerance = highPivotTolerance.get();
                            elevatorTolerance = highElevatorTolerance.get();
                            wristTolerance = highWristTolerance.get();
                            linearTolerance = highLinearTolerance.get();
                            angularTolerance = highAngularTolerance.get();
                        break;
                    }
                    Logger.recordOutput("Self Record/Algae/Superstructure/Pivot", MeasureUtil.isNear(intakeAlgaeObjective.get().getTargetState().pivotAngle, superstructure.getCurrentState().pivotAngle, pivotTolerance));
                    Logger.recordOutput("Self Record/Algae/Superstructure/Elevator", MeasureUtil.isNear(intakeAlgaeObjective.get().getTargetState().elevatorLength, superstructure.getCurrentState().elevatorLength, elevatorTolerance));
                    Logger.recordOutput("Self Record/Algae/Superstructure/Wrist", MeasureUtil.isNear(intakeAlgaeObjective.get().getTargetState().wristAngle, superstructure.getCurrentState().wristAngle, wristTolerance));
                    Logger.recordOutput("Self Record/Algae/Robot/Linear", GeomUtil.isNear(intakeAlgaeObjective.get().getTargetPose().getOurs().getTranslation(), drive.getPose().getTranslation(), linearTolerance));
                    Logger.recordOutput("Self Record/Algae/Robot/Angular", GeomUtil.isNear(intakeAlgaeObjective.get().getTargetPose().getOurs().getRotation(), drive.getPose().getRotation(), angularTolerance));
                    if (
                        GeomUtil.isNear(intakeAlgaeObjective.get().getTargetPose().getOurs(), drive.getPose(), linearTolerance, angularTolerance)
                        && superstructure.getCurrentState().isNear(intakeAlgaeObjective.get().getTargetState(), pivotTolerance, elevatorTolerance, wristTolerance)
                    ) {
                        objectiveTracker.removeAlgae(intakeAlgaeObjective.get().getTargetAlgae());
                    }
                }
            }
        });

        CommandScheduler.getInstance().getDefaultButtonLoop().bind(new Runnable() {
            private static final LoggedTunableMeasure<AngleUnit> l4PivotTolerance = new LoggedTunableMeasure<>("Auto Eject/Coral/L4/Superstructure/Pivot Tolerance", Degrees.of(2));
            private static final LoggedTunableMeasure<DistanceUnit> l4ElevatorTolerance = new LoggedTunableMeasure<>("Auto Eject/Coral/L4/Superstructure/Elevator Tolerance", Inches.of(2));
            private static final LoggedTunableMeasure<AngleUnit> l4WristTolerance = new LoggedTunableMeasure<>("Auto Eject/Coral/L4/Superstructure/Wrist Tolerance", Degrees.of(5));
            private static final LoggedTunableMeasure<DistanceUnit> l4LinearTolerance = new LoggedTunableMeasure<>("Auto Eject/Coral/L4/Robot/Linear Tolerance", Inches.of(2));
            private static final LoggedTunableMeasure<AngleUnit> l4AngularTolerance = new LoggedTunableMeasure<>("Auto Eject/Coral/L4/Robot/Angular Tolerance", Degrees.of(5));
            
            private static final LoggedTunableMeasure<AngleUnit> l3PivotTolerance = new LoggedTunableMeasure<>("Auto Eject/Coral/L3/Superstructure/Pivot Tolerance", Degrees.of(2));
            private static final LoggedTunableMeasure<DistanceUnit> l3ElevatorTolerance = new LoggedTunableMeasure<>("Auto Eject/Coral/L3/Superstructure/Elevator Tolerance", Inches.of(2));
            private static final LoggedTunableMeasure<AngleUnit> l3WristTolerance = new LoggedTunableMeasure<>("Auto Eject/Coral/L3/Superstructure/Wrist Tolerance", Degrees.of(5));
            private static final LoggedTunableMeasure<DistanceUnit> l3LinearTolerance = new LoggedTunableMeasure<>("Auto Eject/Coral/L3/Robot/Linear Tolerance", Inches.of(2));
            private static final LoggedTunableMeasure<AngleUnit> l3AngularTolerance = new LoggedTunableMeasure<>("Auto Eject/Coral/L3/Robot/Angular Tolerance", Degrees.of(5));

            private static final LoggedTunableMeasure<AngleUnit> l2PivotTolerance = new LoggedTunableMeasure<>("Auto Eject/Coral/L2/Superstructure/Pivot Tolerance", Degrees.of(2));
            private static final LoggedTunableMeasure<DistanceUnit> l2ElevatorTolerance = new LoggedTunableMeasure<>("Auto Eject/Coral/L2/Superstructure/Elevator Tolerance", Inches.of(2));
            private static final LoggedTunableMeasure<AngleUnit> l2WristTolerance = new LoggedTunableMeasure<>("Auto Eject/Coral/L2/Superstructure/Wrist Tolerance", Degrees.of(5));
            private static final LoggedTunableMeasure<DistanceUnit> l2LinearTolerance = new LoggedTunableMeasure<>("Auto Eject/Coral/L2/Robot/Linear Tolerance", Inches.of(2));
            private static final LoggedTunableMeasure<AngleUnit> l2AngularTolerance = new LoggedTunableMeasure<>("Auto Eject/Coral/L2/Robot/Angular Tolerance", Degrees.of(5));

            private static final LoggedTunableMeasure<AngleUnit> l1PivotTolerance = new LoggedTunableMeasure<>("Auto Eject/Coral/L1/Superstructure/Pivot Tolerance", Degrees.of(2));
            private static final LoggedTunableMeasure<DistanceUnit> l1ElevatorTolerance = new LoggedTunableMeasure<>("Auto Eject/Coral/L1/Superstructure/Elevator Tolerance", Inches.of(2));
            private static final LoggedTunableMeasure<AngleUnit> l1WristTolerance = new LoggedTunableMeasure<>("Auto Eject/Coral/L1/Superstructure/Wrist Tolerance", Degrees.of(5));
            private static final LoggedTunableMeasure<DistanceUnit> l1LinearTolerance = new LoggedTunableMeasure<>("Auto Eject/Coral/L1/Robot/Linear Tolerance", Inches.of(2));
            private static final LoggedTunableMeasure<AngleUnit> l1AngularTolerance = new LoggedTunableMeasure<>("Auto Eject/Coral/L1/Robot/Angular Tolerance", Degrees.of(5));

            private final Command ejectBranch = intake.eject();
            private final Command ejectL1 = intake.ejectLevel1();

            private final Debouncer debouncer = new Debouncer(0.5, DebounceType.kRising);

            @Override
            public void run() {
                if (intake.hasCoral.getAsBoolean() && !manualOverrides.autoEjectCoralDisabled()) {
                    var scoreCoralObjective = objectiveTracker.getScoreCoralObjective();
                    final Measure<AngleUnit> pivotTolerance;
                    final Measure<DistanceUnit> elevatorTolerance;
                    final Measure<AngleUnit> wristTolerance;
                    final Measure<DistanceUnit> linearTolerance;
                    final Measure<AngleUnit> angularTolerance;
                    if (scoreCoralObjective.getTargetBranch().isPresent()) {
                        switch (scoreCoralObjective.getTargetBranch().get().level) {
                            case Level2:
                                pivotTolerance = l2PivotTolerance.get();
                                elevatorTolerance = l2ElevatorTolerance.get();
                                wristTolerance = l2WristTolerance.get();
                                linearTolerance = l2LinearTolerance.get();
                                angularTolerance = l2AngularTolerance.get();
                            break;
                            case Level3:
                                pivotTolerance = l3PivotTolerance.get();
                                elevatorTolerance = l3ElevatorTolerance.get();
                                wristTolerance = l3WristTolerance.get();
                                linearTolerance = l3LinearTolerance.get();
                                angularTolerance = l3AngularTolerance.get();
                            break;
                            case Level4: default:
                                pivotTolerance = l4PivotTolerance.get();
                                elevatorTolerance = l4ElevatorTolerance.get();
                                wristTolerance = l4WristTolerance.get();
                                linearTolerance = l4LinearTolerance.get();
                                angularTolerance = l4AngularTolerance.get();
                            break;
                        }
                    } else {
                        pivotTolerance = l1PivotTolerance.get();
                        elevatorTolerance = l1ElevatorTolerance.get();
                        wristTolerance = l1WristTolerance.get();
                        linearTolerance = l1LinearTolerance.get();
                        angularTolerance = l1AngularTolerance.get();
                    }
                    var pivotInTolerance = MeasureUtil.isNear(scoreCoralObjective.getTargetState().pivotAngle, superstructure.getCurrentState().pivotAngle, pivotTolerance);
                    var elevatorInTolerance = MeasureUtil.isNear(scoreCoralObjective.getTargetState().elevatorLength, superstructure.getCurrentState().elevatorLength, elevatorTolerance);
                    var wristInTolerance = MeasureUtil.isNear(scoreCoralObjective.getTargetState().wristAngle, superstructure.getCurrentState().wristAngle, wristTolerance);
                    var linearInTolerance = GeomUtil.isNear(scoreCoralObjective.getTargetPose().getOurs().getTranslation(), drive.getPose().getTranslation(), linearTolerance);
                    var angularInTolerance = GeomUtil.isNear(scoreCoralObjective.getTargetPose().getOurs().getRotation(), drive.getPose().getRotation(), angularTolerance);
                    Logger.recordOutput("Auto Eject/Coral/Superstructure/Pivot", pivotInTolerance);
                    Logger.recordOutput("Auto Eject/Coral/Superstructure/Elevator", elevatorInTolerance);
                    Logger.recordOutput("Auto Eject/Coral/Superstructure/Wrist", wristInTolerance);
                    Logger.recordOutput("Auto Eject/Coral/Robot/Linear", linearInTolerance);
                    Logger.recordOutput("Auto Eject/Coral/Robot/Angular", angularInTolerance);

                    if (this.debouncer.calculate(pivotInTolerance && elevatorInTolerance && wristInTolerance && linearInTolerance && angularInTolerance)) {
                        if (scoreCoralObjective.getTargetBranch().isPresent()) {
                            if (!ejectBranch.isScheduled()) {
                                ejectBranch.schedule();
                            }
                        } else {
                            if (!ejectL1.isScheduled()) {
                                ejectL1.schedule();
                            }
                        }
                    }
                } else {
                    if (ejectBranch.isScheduled()) {
                        ejectBranch.cancel();
                    }
                    if (ejectL1.isScheduled()) {
                        ejectL1.cancel();
                    }
                }
            }
        });
    }

    private void setPose(Pose2d pose) {
        questNav.setPose(pose);
        drive.setPose(pose);
    }

    private void configureNotifications() {
        intake.hasCoral
            .onTrue(
                Leds.getInstance().coralAcquired.setFlagCommand().withTimeout(1).alongWith(driveController.rumble(RumbleType.kBothRumble, 0.3).withTimeout(1))
            )
            .whileTrue(
                Leds.getInstance().coralSecured.setFlagCommand().ignoringDisable(true)
            )
        ;
        intake.hasAlgae
            .onTrue(
                Leds.getInstance().algaeAcquired.setFlagCommand().withTimeout(1).alongWith(driveController.rumble(RumbleType.kBothRumble, 0.3).withTimeout(1))
            )
            .whileTrue(
                Leds.getInstance().algaeSecured.setFlagCommand().ignoringDisable(true)
            )
        ;
        new Trigger(() -> Environment.isCompetition() && DriverStation.isTeleop() && DriverStation.getMatchTime() <= 20)
            .onTrue(
                Commands.sequence(
                    Commands.runOnce(() -> driveController.setRumble(RumbleType.kBothRumble, 0)),
                    Commands.repeatingSequence(
                        driveController.rumble(RumbleType.kBothRumble, 0.3).withTimeout(.3),
                        Commands.waitSeconds(.3)
                    ).withTimeout(3)
                )
            );
    }

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
        
        SmartDashboard.putData("Wheel Calibration", Commands.defer(
            () -> 
                new WheelRadiusCalibration(
                    drive,
                    WheelRadiusCalibration.VOLTAGE_RAMP_RATE.get(),
                    WheelRadiusCalibration.MAX_VOLTAGE.get()
                )
                .withName("Wheel Calibration"),
                Set.of(drive.translationSubsystem, drive.rotationalSubsystem)
            )
        );
    }
}
