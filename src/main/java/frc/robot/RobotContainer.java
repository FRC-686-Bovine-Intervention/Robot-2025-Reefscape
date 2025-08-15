// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.Arrays;
import java.util.Optional;
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
import frc.robot.subsystems.vision.apriltag.ApriltagPipeline;
import frc.robot.subsystems.vision.apriltag.ApriltagVision;
import frc.robot.subsystems.vision.cameras.Camera;
import frc.robot.subsystems.vision.cameras.CameraIO;
import frc.robot.subsystems.vision.cameras.CameraIOPhoton;
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
    public final QuestNav questNav;
    public final ManualOverrides manualOverrides;
    public final ObjectiveTracker objectiveTracker;
    
    public final AutoManager autoManager;
    
    // Vision
    public final Camera frontLeftCamera;
    public final Camera frontRightCamera;
    public final Camera backLeftCamera;
    public final Camera backRightCamera;
    public final Camera driverCamera;
    public final ApriltagVision apriltagVision;

    // Controllers
    private final XboxController driveController = new XboxController(0);
    @SuppressWarnings("unused")
    private final ButtonBoard3x3 buttonBoard = new ButtonBoard3x3(1);
    @SuppressWarnings("unused")
    private final CommandJoystick simJoystick = new CommandJoystick(5);

    @SuppressWarnings("resource")
    public RobotContainer() {
        System.out.println("[Init RobotContainer] Creating " + RobotType.getMode().name() + " " + RobotType.getRobot().name());

        var frontLeftPipeline = new ApriltagPipeline(1);
        var frontRightPipeline = new ApriltagPipeline(1);
        var backLeftPipeline = new ApriltagPipeline(100);
        var backRightPipeline = new ApriltagPipeline(100);

        switch (RobotType.getMode()) {
            case REAL:
                this.drive = new Drive(
                    new GyroIOPigeon2(),
                    Arrays.stream(DriveConstants.moduleConstants)
                        .map(ModuleIOFalcon550::new)
                        .toArray(ModuleIO[]::new)
                );
                this.superstructure = new Superstructure(
                    new Pivot(new PivotIOFalcon()),
                    new Elevator(new ElevatorIOKraken()),
                    new Wrist(new WristIOKraken())
                );
                this.intake = new Intake(new IntakeIOFalcon());
                this.climber = new Climber(new ClimberIOFalcon());
                this.frontLeftCamera = new Camera(
                    new CameraIOPhoton("Front Left"),
                    "Front Left",
                    VisionConstants.frontLeftMount,
                    Optional.of(Leds.getInstance().flAprilConnection),
                    frontLeftPipeline
                );
                this.frontRightCamera = new Camera(
                    new CameraIOPhoton("Front Right"),
                    "Front Right",
                    VisionConstants.frontRightMount,
                    Optional.of(Leds.getInstance().flAprilConnection),
                    frontRightPipeline
                );
                this.backLeftCamera = new Camera(
                    new CameraIOPhoton("Back Left"),
                    "Back Left",
                    VisionConstants.backLeftMount,
                    Optional.of(Leds.getInstance().flAprilConnection),
                    backLeftPipeline
                );
                this.backRightCamera = new Camera(
                    new CameraIOPhoton("Back Right"),
                    "Back Right",
                    VisionConstants.backRightMount,
                    Optional.of(Leds.getInstance().flAprilConnection),
                    backRightPipeline
                );
                this.driverCamera = new Camera(
                    new CameraIOPhoton("Driver Cam"),
                    "Driver Cam",
                    VisionConstants.driveCamMount,
                    Optional.empty()
                );
                this.questNav = new QuestNav(QuestNavConstants.metaQuest3S, new QuestNavIOQuest3S(), Leds.getInstance().questNavConnection);
                this.objectiveTracker = new ObjectiveTracker(new ReefTrackerIOServer());
            break;
            case SIM:
                this.drive = new Drive(
                    new GyroIO() {},
                    Arrays.stream(DriveConstants.moduleConstants)
                        .map(ModuleIOSim::new)
                        .toArray(ModuleIO[]::new)
                );
                this.superstructure = new Superstructure(
                    new Pivot(new PivotIOSim()),
                    new Elevator(new ElevatorIOSim()),
                    new Wrist(new WristIOSim())
                );
                this.intake = new Intake(new IntakeIOSim(this.simJoystick.button(1), this.simJoystick.button(2)));
                // intake = new Intake(new IntakeIOSim(driveController.povDown(), simJoystick.button(2)));
                this.climber = new Climber(new ClimberIO() {});
                this.frontLeftCamera = new Camera(
                    new CameraIO() {},
                    "Front Left",
                    VisionConstants.frontLeftMount,
                    Optional.of(Leds.getInstance().flAprilConnection),
                    frontLeftPipeline
                );
                this.frontRightCamera = new Camera(
                    new CameraIO() {},
                    "Front Right",
                    VisionConstants.frontRightMount,
                    Optional.of(Leds.getInstance().flAprilConnection),
                    frontRightPipeline
                );
                this.backLeftCamera = new Camera(
                    new CameraIO() {},
                    "Back Left",
                    VisionConstants.backLeftMount,
                    Optional.of(Leds.getInstance().flAprilConnection),
                    backLeftPipeline
                );
                this.backRightCamera = new Camera(
                    new CameraIO() {},
                    "Back Right",
                    VisionConstants.backRightMount,
                    Optional.of(Leds.getInstance().flAprilConnection),
                    backRightPipeline
                );
                this.driverCamera = new Camera(
                    new CameraIO() {},
                    "Driver Cam",
                    VisionConstants.driveCamMount,
                    Optional.empty()
                );
                this.questNav = new QuestNav(QuestNavConstants.metaQuest3S, new QuestNavIOSim(), Leds.getInstance().questNavConnection);
                this.objectiveTracker = new ObjectiveTracker(new ReefTrackerIOServer());
            break;
            default:
            case REPLAY:
                this.drive = new Drive(
                    new GyroIO() {},
                    new ModuleIO(){},
                    new ModuleIO(){},
                    new ModuleIO(){},
                    new ModuleIO(){}
                );
                this.superstructure = new Superstructure(
                    new Pivot(new PivotIO() {}),
                    new Elevator(new ElevatorIO() {}),
                    new Wrist(new WristIO() {})
                );
                this.intake = new Intake(new IntakeIO() {});
                this.climber = new Climber(new ClimberIO() {});
                this.frontLeftCamera = new Camera(
                    new CameraIO() {},
                    "Front Left",
                    VisionConstants.frontLeftMount,
                    Optional.of(Leds.getInstance().flAprilConnection),
                    frontLeftPipeline
                );
                this.frontRightCamera = new Camera(
                    new CameraIO() {},
                    "Front Right",
                    VisionConstants.frontRightMount,
                    Optional.of(Leds.getInstance().flAprilConnection),
                    frontRightPipeline
                );
                this.backLeftCamera = new Camera(
                    new CameraIO() {},
                    "Back Left",
                    VisionConstants.backLeftMount,
                    Optional.of(Leds.getInstance().flAprilConnection),
                    backLeftPipeline
                );
                this.backRightCamera = new Camera(
                    new CameraIO() {},
                    "Back Right",
                    VisionConstants.backRightMount,
                    Optional.of(Leds.getInstance().flAprilConnection),
                    backRightPipeline
                );
                this.driverCamera = new Camera(
                    new CameraIO() {},
                    "Driver Cam",
                    VisionConstants.driveCamMount,
                    Optional.empty()
                );
                this.questNav = new QuestNav(QuestNavConstants.metaQuest3S, new QuestNavIO() {}, Leds.getInstance().questNavConnection);
                this.objectiveTracker = new ObjectiveTracker(new ReefTrackerIO() {});
            break;
        }
        this.apriltagVision = new ApriltagVision(
            frontLeftPipeline,
            frontRightPipeline,
            backLeftPipeline,
            backRightPipeline
        );
        this.manualOverrides = new ManualOverrides();
        
        this.drive.structureRoot
            .addChild(this.frontLeftCamera.mount)
            .addChild(this.frontRightCamera.mount)
            .addChild(this.backLeftCamera.mount)
            .addChild(this.backRightCamera.mount)
            .addChild(VisionConstants.questNavMount)
            .addChild(this.superstructure.pivot.mech
                .addChild(this.superstructure.elevator.stage2Mech
                    .addChild(this.superstructure.elevator.stage3Mech
                        .addChild(this.superstructure.elevator.stage4Mech
                            .addChild(this.superstructure.wrist.mech
                                .addChild(this.driverCamera.mount)
                                .addChild(this.intake.coralPose)
                                .addChild(this.intake.algaePose)
                            )
                        )
                    )
                )
            )
            .addChild(this.climber.mech)
        ;
        Mechanism3d.registerMechs(this.superstructure.pivot.mech, this.superstructure.elevator.stage2Mech, this.superstructure.elevator.stage3Mech, this.superstructure.elevator.stage4Mech, this.superstructure.wrist.mech, this.climber.mech);

        System.out.println("[Init RobotContainer] Configuring Commands");
        this.configureCommands();

        System.out.println("[Init RobotContainer] Configuring Notifications");
        this.configureNotifications();

        System.out.println("[Init RobotContainer] Configuring Autonomous Modes");
        this.configureAutos();
        AutoPaths.preload();
        var selector = new AutoSelector("Auto Selector");
        selector.addDefaultRoutine(new ScoreCoral(this));
        selector.addRoutine(new ScoreAlgaeAndCoral(this));
        selector.addRoutine(new DrivePastLine(this));

        this.autoManager = new AutoManager(selector);

        System.out.println("[Init RobotContainer] Configuring System Check");
        this.configureSystemCheck();

        if (RobotConstants.tuningMode) {
            new Alert("Tuning mode active", AlertType.kInfo).set(true);
        }
    }

    private void configureCommands() {
        var driveJoystick = this.driveController.leftStick
            .smoothRadialDeadband(DriveConstants.driveJoystickDeadbandPercent)
            .radialSensitivity(0.75)
            // .radialSlewRateLimit(DriveConstants.joystickSlewRateLimit)
        ;

        var joystickTranslational = Drive.Translational.joystickSpectatorToFieldRelative(driveJoystick);

        this.drive.translationSubsystem.setDefaultCommand(
            this.drive.translationSubsystem.run(() -> {
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
                if (this.driveController.leftTrigger.getAsDouble() > 0.1 && this.driveController.rightTrigger.getAsDouble() > 0.1) {
                    robotSpeeds = new ChassisSpeeds(
                        Math.min(this.driveController.leftTrigger.getAsDouble(), this.driveController.rightTrigger.getAsDouble()) * DriveConstants.maxAdjustmentSpeed.in(MetersPerSecond),
                        0,
                        0
                    );
                } else {
                    robotSpeeds = new ChassisSpeeds(
                        0,
                        (this.driveController.leftTrigger.getAsDouble() - this.driveController.rightTrigger.getAsDouble()) * DriveConstants.maxAdjustmentSpeed.in(MetersPerSecond),
                        0
                    );
                }
                if (this.objectiveTracker.getCurrentObjective().filter((objective) -> objective.getTargetDirection().isForward()).isEmpty()) {
                    robotSpeeds = new ChassisSpeeds(
                        -robotSpeeds.vxMetersPerSecond,
                        robotSpeeds.vyMetersPerSecond,
                        robotSpeeds.omegaRadiansPerSecond
                    );
                }
                this.drive.translationSubsystem.driveVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(fieldSpeeds, this.drive.getRotation()).plus(robotSpeeds));
            })
            .withName("Driver Control Field Relative")
        );
        this.drive.rotationalSubsystem.setDefaultCommand(
            this.drive.rotationalSubsystem.spin(this.driveController.rightStick.x().smoothDeadband(0.05).multiply(DriveConstants.maxTurnRate.in(RadiansPerSecond)).multiply(0.5))
                .withName("Robot spin")
        );
        new Trigger(DriverStation::isDisabled).and(() -> driveJoystick.magnitude() > 0).whileTrue(drive.coast());

        this.superstructure.setDefaultCommand(this.superstructure.goToSetpointSequenced(SuperstructureConstants.idleState));
        this.intake.setDefaultCommand(this.intake.idle());
        this.climber.setDefaultCommand(this.climber.idle());

        driveController.povUp().onTrue(Commands.runOnce(() -> objectiveTracker.shiftLevelLock(1)));
        driveController.povDown().onTrue(Commands.runOnce(() -> objectiveTracker.shiftLevelLock(-1)));
        driveController.povLeft().onTrue(Commands.runOnce(() -> objectiveTracker.shiftPipeLock(-1)));
        driveController.povRight().onTrue(Commands.runOnce(() -> objectiveTracker.shiftPipeLock(1)));
        
        this.driveController.a().whileTrue(new ContinuouslySwappingCommand(
            new Supplier<Command>() {
                private final Command eject = intake.eject();
                private final Command ejectL1 = intake.ejectLevel1();
                private final Command ejectAlgae = intake.ejectAlgae();
                public Command get() {
                    if (intake.hasAlgae.getAsBoolean()) {
                        return this.ejectAlgae;
                    } else if (intake.hasCoral.getAsBoolean() && objectiveTracker.getScoreCoralObjective().getTargetBranch().isEmpty()) {
                        return this.ejectL1;
                    } else {
                        return this.eject;
                    }
                }
            },
            Set.of(this.intake)
        )); //Eject
        final Command coralIntakeCommand = new ContinuouslySwappingCommand(
            new Supplier<Command>() {
                private final RobotFlippedCommand coralStationCommands = CoralStation.intakePosition.mapToCommand((state) -> superstructure.goToSetpointSequenced(state).raceWith(intake.intakeCoral().until(intake.hasCoral)));
                public Command get() {
                    return this.coralStationCommands.get(objectiveTracker.getIntakeCoralObjective().getTargetDirection());
                }
            },
            Set.of(this.superstructure, this.intake)
        ).withName("Intake Coral Station");
        CommandScheduler.getInstance().getDefaultButtonLoop().bind(() -> {
            if (this.driveController.hid.getBButtonPressed()) {
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
                        return this.idle;
                    } else {
                        var stagedAlgaeObjective = optStagedAlgaeObjective.get();
                        return stagedAlgaeCommands[stagedAlgaeObjective.getTargetAlgae().level.ordinal()].get(stagedAlgaeObjective.getTargetDirection());
                    }
                }
            },
            Set.of(this.superstructure, this.intake)
        ).deadlineFor(objectiveTracker.setTypeOverrideCommand(ObjectiveType.IntakeAlgae)).withName("Intake Staged Algae");
        final Command groundAlgaeIntakeCommand = superstructure.goToSetpointSequenced(SuperstructureState.fromParts(PivotConstants.minAngle, ElevatorConstants.minLengthPhysical, Degrees.of(-35))).alongWith(intake.intakeAlgae()).withName("Intake Ground Algae");
        final Timer algaeIntakeButtonTimer = new Timer();
        CommandScheduler.getInstance().getDefaultButtonLoop().bind(() -> {
            if (this.driveController.hid.getYButtonPressed()) {
                if (stagedAlgaeIntakeCommand.isScheduled()) {
                    stagedAlgaeIntakeCommand.cancel();
                } else if (groundAlgaeIntakeCommand.isScheduled()) {
                    groundAlgaeIntakeCommand.cancel();
                } else {
                    algaeIntakeButtonTimer.start();
                }
            }
            if (this.driveController.hid.getYButtonReleased()) {
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
                        return this.level1Command.get(scoreCoralObjective.getTargetDirection());
                    }
                }
            },
            Set.of(this.superstructure)
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
            Set.of(this.superstructure)
        ).withName("Extend to Algae Goal");

        CommandScheduler.getInstance().getDefaultButtonLoop().bind(() -> {
            if (this.driveController.hid.getXButtonPressed()) {
                if (!coralScoreCommand.isScheduled() && !algaeScoreCommand.isScheduled()) {
                    if (this.intake.hasCoral.getAsBoolean()) {
                        coralScoreCommand.schedule();
                    }
                    if (this.intake.hasAlgae.getAsBoolean()) {
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
        driveController.rightBumper()
            .and(() -> objectiveTracker.getCurrentObjective().isPresent())
            .whileTrue(
                this.drive.simplePIDTo(
                    () -> AutoScore.getTargetPose(
                        drive.getPose(),
                        objectiveTracker.getCurrentObjective().get().getTargetPose().getOurs(),
                        objectiveTracker.getCurrentObjective().get().getObjectiveType().isReefObjective
                    )
                )
                .deadlineFor(
                    Commands.startEnd(
                        () -> {
                            if (objectiveTracker.getCurrentObjective().filter((objective) -> objective.getObjectiveType() == ObjectiveType.ScoreCoral).isPresent()) {
                                if (objectiveTracker.getScoreCoralObjective().getTargetBranch().isPresent()) {
                                    objectiveTracker.addPipeLock(objectiveTracker.getScoreCoralObjective().getTargetBranch().get().pipe);
                                }
                            }
                        },
                        () -> {
                            this.objectiveTracker.removePipeLock();
                        }
                    )
                )
            )
        ; //Auto drive
        this.driveController.start().toggleOnTrue(
            Commands.parallel(
                this.climber.prepareClimb(),
                this.superstructure.goToSetpointSequenced(SuperstructureConstants.prepareClimbingState)
            )
            .deadlineFor(
                this.objectiveTracker.setTypeOverrideCommand(ObjectiveType.Climb)
            )
            // climber.testDisengageRatchet()
        );
        this.driveController.back().toggleOnTrue(
            Commands.parallel(
                this.climber.climb(),
                this.superstructure.goToSetpointSequenced(SuperstructureConstants.climbingState)
            )
            .deadlineFor(
                this.objectiveTracker.setTypeOverrideCommand(ObjectiveType.Climb)
            )
        );

        var selfRightCommand = this.superstructure.goToSetpointSequenced(SuperstructureConstants.selfRightingState);
        // var prepareSelfRightCommand = superstructure.goToSetpointSequenced(SuperstructureConstants.prepareSelfRightingState);
        CommandScheduler.getInstance().getDefaultButtonLoop().bind(new Runnable() {
            private boolean prevSelfRight = true;
            // private boolean prevprepare = true;
            public void run() {
                var selfRightButton = driveController.hid.getPOV() == 0;
                // var prepare = driveController.hid.getPOV() == 90;
                var tipped = !MeasureUtil.isNear(Degrees.of(0), drive.getPitch(), Degrees.of(45));
                Leds.getInstance().tipped.setFlag(tipped);
                if (selfRightButton && !this.prevSelfRight) {
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
                this.prevSelfRight = selfRightButton;
                // prevprepare = prepare;
            }
        });
        
        driveController.leftStickButton().and(driveController.rightStickButton()).onTrue(Commands.runOnce(() -> this.setPose(Reef.reefs.getOurs().racks[0].centerRobotPose.getForward())).ignoringDisable(true));
        // new Trigger(() -> apriltagVision.getPose().xyStdDev() < .5)
        //     .onTrue(Commands.runOnce(() -> this.setPose(apriltagVision.getPose().robotPose())));

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
            private static final LoggedTunableMeasure<AngleUnit> autoEjectPivotTolerance = new LoggedTunableMeasure<>("Auto Eject/Coral/Superstructure/Pivot Tolerance", Degrees.of(2));
            private static final LoggedTunableMeasure<DistanceUnit> autoEjectElevatorTolerance = new LoggedTunableMeasure<>("Auto Eject/Coral/Superstructure/Elevator Tolerance", Inches.of(2));
            private static final LoggedTunableMeasure<AngleUnit> autoEjectWristTolerance = new LoggedTunableMeasure<>("Auto Eject/Coral/Superstructure/Wrist Tolerance", Degrees.of(5));
            private static final LoggedTunableMeasure<DistanceUnit> autoEjectLinearTolerance = new LoggedTunableMeasure<>("Auto Eject/Coral/Robot/Linear Tolerance", Inches.of(2));
            private static final LoggedTunableMeasure<AngleUnit> autoEjectAngularTolerance = new LoggedTunableMeasure<>("Auto Eject/Coral/Robot/Angular Tolerance", Degrees.of(5));

            private final Command ejectBranch = intake.eject();
            private final Command ejectL1 = intake.ejectLevel1();

            private final Debouncer debouncer = new Debouncer(0.5, DebounceType.kRising);

            @Override
            public void run() {
                if (intake.hasCoral.getAsBoolean() && !manualOverrides.autoEjectCoralDisabled()) {
                    var scoreCoralObjective = objectiveTracker.getScoreCoralObjective();
                    var pivotInTolerance = MeasureUtil.isNear(scoreCoralObjective.getTargetState().pivotAngle, superstructure.getCurrentState().pivotAngle, autoEjectPivotTolerance.get());
                    var elevatorInTolerance = MeasureUtil.isNear(scoreCoralObjective.getTargetState().elevatorLength, superstructure.getCurrentState().elevatorLength, autoEjectElevatorTolerance.get());
                    var wristInTolerance = MeasureUtil.isNear(scoreCoralObjective.getTargetState().wristAngle, superstructure.getCurrentState().wristAngle, autoEjectWristTolerance.get());
                    var linearInTolerance = GeomUtil.isNear(scoreCoralObjective.getTargetPose().getOurs().getTranslation(), drive.getPose().getTranslation(), autoEjectLinearTolerance.get());
                    var angularInTolerance = GeomUtil.isNear(scoreCoralObjective.getTargetPose().getOurs().getRotation(), drive.getPose().getRotation(), autoEjectAngularTolerance.get());
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
        this.questNav.setPose(pose);
        this.drive.setPose(pose);
    }

    private void configureNotifications() {
        this.intake.hasCoral
            .onTrue(
                Leds.getInstance().coralAcquired.setFlagCommand().withTimeout(1).alongWith(driveController.rumble(RumbleType.kBothRumble, 0.3).withTimeout(1))
            )
            .whileTrue(
                Leds.getInstance().coralSecured.setFlagCommand().ignoringDisable(true)
            )
        ;
        this.intake.hasAlgae
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
                    Commands.runOnce(() -> this.driveController.setRumble(RumbleType.kBothRumble, 0)),
                    Commands.repeatingSequence(
                        this.driveController.rumble(RumbleType.kBothRumble, 0.3).withTimeout(.3),
                        Commands.waitSeconds(.3)
                    ).withTimeout(3)
                )
            )
        ;
    }

    private void configureAutos() {
        
    }

    private void configureSystemCheck() {
        SmartDashboard.putData("System Check/Drive/Spin", 
            new Command() {
                private final Drive.Rotational rotationalSubsystem = drive.rotationalSubsystem;
                private final Timer timer = new Timer();
                {
                    addRequirements(this.rotationalSubsystem);
                    setName("TEST Spin");
                }
                public void initialize() {
                    this.timer.restart();
                }
                public void execute() {
                    this.rotationalSubsystem.driveVelocity(Math.sin(this.timer.get()) * 3);
                }
                public void end(boolean interrupted) {
                    this.timer.stop();
                    this.rotationalSubsystem.stop();
                }
            }
        );
        SmartDashboard.putData("System Check/Drive/Circle", 
            new Command() {
                private final Drive.Translational translationSubsystem = drive.translationSubsystem;
                private final Timer timer = new Timer();
                {
                    addRequirements(this.translationSubsystem);
                    setName("TEST Circle");
                }
                public void initialize() {
                    this.timer.restart();
                }
                public void execute() {
                    this.translationSubsystem.driveVelocity(
                        new ChassisSpeeds(
                            Math.cos(this.timer.get()) * 0.01,
                            Math.sin(this.timer.get()) * 0.01,
                            0
                        )
                    );
                }
                public void end(boolean interrupted) {
                    this.timer.stop();
                    this.translationSubsystem.stop();
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
