// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.Arrays;
import java.util.Set;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.EventLoop;
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
import frc.robot.constants.FieldConstants.Reef;
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
import frc.robot.subsystems.drive.OdometryTimestampIO;
import frc.robot.subsystems.drive.OdometryTimestampIO.OdometryTimestampIOOdometryThread;
import frc.robot.subsystems.drive.OdometryTimestampIO.OdometryTimestampIOSim;
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
import frc.robot.subsystems.superstructure.SuperstructureConstants;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.ElevatorIO;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOKraken;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOSim;
import frc.robot.subsystems.superstructure.pivot.Pivot;
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
import frc.util.LoggedTracer;
import frc.util.Perspective;
import frc.util.commands.ContinuouslySwappingCommand;
import frc.util.controllers.ButtonBoard3x3;
import frc.util.controllers.XboxController;
import frc.util.geometry.GeomUtil;
import frc.util.loggerUtil.tunables.LoggedTunable;
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

    // Event Loops
    public final EventLoop automationsLoop = new EventLoop();

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
                this.drive = new Drive(
                    new OdometryTimestampIOOdometryThread(),
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
                    Leds.getInstance().flAprilConnection::setStatus
                );
                this.frontRightCamera = new Camera(
                    new CameraIOPhoton("Front Right"),
                    "Front Right",
                    VisionConstants.frontRightMount,
                    Leds.getInstance().frAprilConnection::setStatus
                );
                this.backLeftCamera = new Camera(
                    new CameraIOPhoton("Back Left"),
                    "Back Left",
                    VisionConstants.backLeftMount,
                    Leds.getInstance().blAprilConnection::setStatus
                );
                this.backRightCamera = new Camera(
                    new CameraIOPhoton("Back Right"),
                    "Back Right",
                    VisionConstants.backRightMount,
                    Leds.getInstance().brAprilConnection::setStatus
                );
                this.driverCamera = new Camera(
                    new CameraIOPhoton("Driver Cam"),
                    "Driver Cam",
                    VisionConstants.driveCamMount,
                    (connected) -> {}
                );
                this.questNav = new QuestNav(QuestNavConstants.metaQuest3S, new QuestNavIOQuest3S(), Leds.getInstance().questNavConnection);
                this.objectiveTracker = new ObjectiveTracker(new ReefTrackerIOServer());
            break;
            case SIM:
                this.drive = new Drive(
                    new OdometryTimestampIOSim(),
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
                    Leds.getInstance().flAprilConnection::setStatus
                );
                this.frontRightCamera = new Camera(
                    new CameraIO() {},
                    "Front Right",
                    VisionConstants.frontRightMount,
                    Leds.getInstance().frAprilConnection::setStatus
                );
                this.backLeftCamera = new Camera(
                    new CameraIO() {},
                    "Back Left",
                    VisionConstants.backLeftMount,
                    Leds.getInstance().blAprilConnection::setStatus
                );
                this.backRightCamera = new Camera(
                    new CameraIO() {},
                    "Back Right",
                    VisionConstants.backRightMount,
                    Leds.getInstance().brAprilConnection::setStatus
                );
                this.driverCamera = new Camera(
                    new CameraIO() {},
                    "Driver Cam",
                    VisionConstants.driveCamMount,
                    (connected) -> {}
                );
                this.questNav = new QuestNav(QuestNavConstants.metaQuest3S, new QuestNavIOSim(), Leds.getInstance().questNavConnection);
                this.objectiveTracker = new ObjectiveTracker(new ReefTrackerIOServer());
            break;
            default:
            case REPLAY:
                this.drive = new Drive(
                    new OdometryTimestampIO() {},
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
                    Leds.getInstance().flAprilConnection::setStatus
                );
                this.frontRightCamera = new Camera(
                    new CameraIO() {},
                    "Front Right",
                    VisionConstants.frontRightMount,
                    Leds.getInstance().frAprilConnection::setStatus
                );
                this.backLeftCamera = new Camera(
                    new CameraIO() {},
                    "Back Left",
                    VisionConstants.backLeftMount,
                    Leds.getInstance().blAprilConnection::setStatus
                );
                this.backRightCamera = new Camera(
                    new CameraIO() {},
                    "Back Right",
                    VisionConstants.backRightMount,
                    Leds.getInstance().brAprilConnection::setStatus
                );
                this.driverCamera = new Camera(
                    new CameraIO() {},
                    "Driver Cam",
                    VisionConstants.driveCamMount,
                    (connected) -> {}
                );
                this.questNav = new QuestNav(QuestNavConstants.metaQuest3S, new QuestNavIO() {}, Leds.getInstance().questNavConnection);
                this.objectiveTracker = new ObjectiveTracker(new ReefTrackerIO() {});
            break;
        }
        this.apriltagVision = new ApriltagVision(
            new ApriltagPipeline(this.frontLeftCamera, 0, 1),
            new ApriltagPipeline(this.frontRightCamera, 0, 1),
            new ApriltagPipeline(this.backLeftCamera, 0, 100),
            new ApriltagPipeline(this.backRightCamera, 0, 100)
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
            .smoothRadialDeadband(0.1)
            .radialSensitivity(0.75)
            // .radialSlewRateLimit(DriveConstants.joystickSlewRateLimit)
        ;

        this.drive.translationSubsystem.setDefaultCommand(
            new Command() {
                {
                    this.addRequirements(drive.translationSubsystem);
                    this.setName("Driver Control Field Relative");
                }

                @Override
                public void execute() {
                    // Rotate joystick such that up is positive X and left is positive Y
                    var perspectiveX = +driveJoystick.y().getAsDouble();
                    var perspectiveY = -driveJoystick.x().getAsDouble();

                    // Get field relative from perspective relative
                    var perspectiveForwardDirection = Perspective.getCurrent().getForwardDirection();
                    var fieldX = perspectiveX * +perspectiveForwardDirection.getCos() + perspectiveY * -perspectiveForwardDirection.getSin();
                    var fieldY = perspectiveX * +perspectiveForwardDirection.getSin() + perspectiveY * +perspectiveForwardDirection.getCos();

                    var fieldXMetersPerSecond = fieldX * DriveConstants.maxDriveSpeed.in(MetersPerSecond) * DriveConstants.maxDriveSpeedEnvCoef.getAsDouble();
                    var fieldYMetersPerSecond = fieldY * DriveConstants.maxDriveSpeed.in(MetersPerSecond) * DriveConstants.maxDriveSpeedEnvCoef.getAsDouble();

                    // Get robot relative from robot relative
                    var robotRotation = RobotState.getInstance().getEstimatedGlobalPose().getRotation();
                    var robotXMetersPerSecond = fieldXMetersPerSecond * +robotRotation.getCos() + fieldYMetersPerSecond * +robotRotation.getSin();
                    var robotYMetersPerSecond = fieldXMetersPerSecond * -robotRotation.getSin() + fieldYMetersPerSecond * +robotRotation.getCos();

                    // Robot relative adjustments
                    double adjustX;
                    double adjustY;
                    if (driveController.leftTrigger.getAsDouble() > 0.1 && driveController.rightTrigger.getAsDouble() > 0.1) {
                        adjustX = Math.min(driveController.leftTrigger.getAsDouble(), driveController.rightTrigger.getAsDouble());
                        adjustY = 0.0;
                    } else {
                        adjustX = 0.0;
                        adjustY = driveController.leftTrigger.getAsDouble() - driveController.rightTrigger.getAsDouble();
                    }
                    if (objectiveTracker.getCurrentObjective().isPresent() && objectiveTracker.getCurrentObjective().get().getTargetDirection().isBackward()) {
                        adjustX *= -1.0;
                    }
                    var adjustXMetersPerSecond = adjustX * DriveConstants.maxAdjustmentSpeed.in(MetersPerSecond);
                    var adjustYMetersPerSecond = adjustY * DriveConstants.maxAdjustmentSpeed.in(MetersPerSecond);

                    drive.translationSubsystem.driveVelocity(
                        robotXMetersPerSecond + adjustXMetersPerSecond,
                        robotYMetersPerSecond + adjustYMetersPerSecond
                    );
                }

                @Override
                public void end(boolean interrupted) {
                    drive.translationSubsystem.stop();
                }
            }
        );
        this.drive.rotationalSubsystem.setDefaultCommand(
            this.drive.rotationalSubsystem.spin(this.driveController.rightStick.x().smoothDeadband(0.1).multiply(DriveConstants.maxTurnRate.in(RadiansPerSecond)).multiply(0.5))
                .withName("Robot spin")
        );
        new Trigger(DriverStation::isDisabled).and(() -> driveJoystick.magnitude() > 0).whileTrue(drive.coast());

        this.superstructure.setDefaultCommand(this.superstructure.goToSetpointSequenced(SuperstructureConstants.idleState));
        this.intake.setDefaultCommand(this.intake.idle());
        this.climber.setDefaultCommand(this.climber.idle());

        this.frontLeftCamera.setDefaultCommand(this.frontLeftCamera.setPipelineIndex(0));
        this.frontRightCamera.setDefaultCommand(this.frontRightCamera.setPipelineIndex(0));
        this.backLeftCamera.setDefaultCommand(this.backLeftCamera.setPipelineIndex(0));
        this.backRightCamera.setDefaultCommand(this.backRightCamera.setPipelineIndex(0));

        driveController.povUp().onTrue(Commands.runOnce(() -> objectiveTracker.shiftLevelLock(1)));
        driveController.povDown().onTrue(Commands.runOnce(() -> objectiveTracker.shiftLevelLock(-1)));
        driveController.povLeft().onTrue(Commands.runOnce(() -> objectiveTracker.shiftPipeLock(-1)));
        driveController.povRight().onTrue(Commands.runOnce(() -> objectiveTracker.shiftPipeLock(1)));
        
        // Eject
        final Command ejectNormal = this.intake.eject();
        final Command ejectL1 = this.intake.ejectL1();
        final Command ejectAlgae = this.intake.ejectAlgae();
        CommandScheduler.getInstance().getDefaultButtonLoop().bind(() -> {
            if (driveController.hid.getAButtonPressed()) {
                if (intake.hasAlgae()) {
                    ejectAlgae.schedule();
                } else if (intake.hasCoral() && objectiveTracker.getScoreCoralObjective().getTargetBranch().isEmpty()) {
                    ejectL1.schedule();
                } else {
                    ejectNormal.schedule();
                }
            } else if (driveController.hid.getAButtonReleased()) {
                if (ejectNormal.isScheduled()) {
                    ejectNormal.cancel();
                } else if (ejectL1.isScheduled()) {
                    ejectL1.cancel();
                } else if (ejectAlgae.isScheduled()) {
                    ejectAlgae.cancel();
                }
            }
        });

        // Coral Intake
        final Command coralIntakeCommand = new ContinuouslySwappingCommand(
            new Supplier<Command>() {
                private final Command forwardCommand = superstructure.goToSetpointSequenced(SuperstructureConstants.coralStationForwardState).raceWith(intake.intakeCoral().until(intake::hasCoral));
                private final Command backwardCommand = superstructure.goToSetpointSequenced(SuperstructureConstants.coralStationBackwardState).raceWith(intake.intakeCoral().until(intake::hasCoral));
                public Command get() {
                    return switch (objectiveTracker.getIntakeCoralObjective().getTargetDirection()) {
                        case Forward -> this.forwardCommand;
                        case Backward -> this.backwardCommand;
                    };
                }
            },
            Set.of(this.superstructure, this.intake)
        ).withName("Intake Coral Station");
        // Button binding below with processor extend

        // Algae Intake
        final Command stagedAlgaeIntakeCommand = new ContinuouslySwappingCommand(
            new Supplier<Command>() {
                private final Command highCommand = superstructure.goToSetpointSequenced(SuperstructureConstants.highAlgaeState).alongWith(intake.intakeAlgae());
                private final Command lowCommand = superstructure.goToSetpointSequenced(SuperstructureConstants.lowAlgaeState).alongWith(intake.intakeAlgae());
                public Command get() {
                    var intakeAlgaeObjective = objectiveTracker.getIntakeAlgaeObjective();
                    if (intakeAlgaeObjective.isEmpty()) {return this.lowCommand;}
                    return switch (intakeAlgaeObjective.get().getTargetAlgae().level) {
                        case High -> this.highCommand;
                        case Low -> this.lowCommand;
                    };
                }
            },
            Set.of(this.superstructure, this.intake)
        ).deadlineFor(objectiveTracker.setTypeOverrideCommand(ObjectiveType.IntakeAlgae)).withName("Intake Staged Algae");
        final Command groundAlgaeIntakeCommand = superstructure
            .goToSetpointSequenced(SuperstructureConstants.groundAlgaeState)
            .alongWith(intake.intakeAlgae())
            .withName("Intake Ground Algae")
        ;
        final Timer algaeIntakeButtonTimer = new Timer();
        CommandScheduler.getInstance().getDefaultButtonLoop().bind(() -> {
            if (this.driveController.hid.getYButtonPressed()) {
                if (stagedAlgaeIntakeCommand.isScheduled()) {
                    stagedAlgaeIntakeCommand.cancel();
                } else if (groundAlgaeIntakeCommand.isScheduled()) {
                    groundAlgaeIntakeCommand.cancel();
                } else if (!intake.hasCoral() && !intake.hasAlgae()) {
                    algaeIntakeButtonTimer.start();
                }
            } else if (driveController.hid.getYButtonReleased()) {
                if (!algaeIntakeButtonTimer.hasElapsed(0.25) && algaeIntakeButtonTimer.isRunning() && objectiveTracker.getIntakeAlgaeObjective().isPresent()) {
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

        // Extend
        final Command coralScoreCommand = new ContinuouslySwappingCommand(
            new Supplier<Command>() {
                private final Command l1Command = superstructure.goToSetpointSequenced(SuperstructureConstants.l1State);
                private final Command l2Command = superstructure.goToSetpointSequenced(SuperstructureConstants.l2State);
                private final Command l3Command = superstructure.goToSetpointSequenced(SuperstructureConstants.l3State);
                private final Command l4Command = superstructure.goToSetpointSequenced(SuperstructureConstants.l4State);
                public Command get() {
                    var scoreCoralObjective = objectiveTracker.getScoreCoralObjective();
                    if (scoreCoralObjective.getTargetBranch().isEmpty()) {return this.l1Command;}
                    return switch (scoreCoralObjective.getTargetBranch().get().level) {
                        case Level2 -> this.l2Command;
                        case Level3 -> this.l3Command;
                        case Level4 -> this.l4Command;
                    };
                }
            },
            Set.of(this.superstructure)
        ).deadlineFor(
            Commands.startEnd(
                () -> objectiveTracker.addLevelLock(objectiveTracker.getScoreCoralObjective().getTargetBranch().map((branch) -> branch.level)),
                () -> objectiveTracker.removeLevelLock()
            )
        ).withName("Extend to Reef");
        final Command netCommand = new ContinuouslySwappingCommand(
            new Supplier<Command>() {
                private final Command forwardCommand = superstructure.goToSetpointSequenced(SuperstructureConstants.netForwardState);
                private final Command backwardCommand = superstructure.goToSetpointSequenced(SuperstructureConstants.netBackwardState);
                public Command get() {
                    return switch (objectiveTracker.getScoreNetObjective().getTargetDirection()) {
                        case Forward -> this.forwardCommand;
                        case Backward -> this.backwardCommand;
                    };
                }
            },
            Set.of(superstructure)
        ).withName("Extend to Net");
        final Command processorCommand = superstructure
            .goToSetpointSequenced(SuperstructureConstants.processorState)
            .deadlineFor(objectiveTracker.setTypeOverrideCommand(ObjectiveType.ScoreProcessor))
            .withName("Extend to Processor")
        ;
        CommandScheduler.getInstance().getDefaultButtonLoop().bind(() -> {
            if (driveController.hid.getXButtonPressed()) {
                if (coralScoreCommand.isScheduled()) {
                    coralScoreCommand.cancel();
                } else if (netCommand.isScheduled()) {
                    netCommand.cancel();
                } else if (processorCommand.isScheduled()) {
                    processorCommand.cancel();
                } else if (intake.hasCoral()) {
                    coralScoreCommand.schedule();
                } else if (intake.hasAlgae()) {
                    netCommand.schedule();
                }
            }
            if (driveController.hid.getBButtonPressed()) {
                if (coralIntakeCommand.isScheduled()) {
                    coralIntakeCommand.cancel();
                } else if (processorCommand.isScheduled()) {
                    processorCommand.cancel();
                } else if (intake.hasAlgae()) {
                    processorCommand.schedule();
                } else if (!intake.hasCoral()) {
                    coralIntakeCommand.schedule();
                }
            }
        });

        // Auto Drive
        driveController.leftBumper().and(() -> objectiveTracker.getCurrentObjective().isPresent()).whileTrue(drive.rotationalSubsystem.pidControlledHeading(() -> objectiveTracker.getCurrentObjective().get().getTargetPose().getOurs().getRotation()));
        final Command autoDriveScoreCoral = this.drive.simplePIDTo(
            () -> AutoScore.getTargetPose(
                RobotState.getInstance().getEstimatedGlobalPose(),
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
                RobotState.getInstance().getEstimatedGlobalPose(),
                this.objectiveTracker.getIntakeAlgaeObjective().get().getTargetPose().getOurs()
            )
        );
        final Command autoDriveIntakeCoral = this.drive.simplePIDTo(() -> this.objectiveTracker.getIntakeCoralObjective().getTargetPose().getOurs());
        final Command autoDriveScoreNet = this.drive.simplePIDTo(() -> this.objectiveTracker.getScoreNetObjective().getTargetPose().getOurs());
        final Command autoDriveScoreProcessor = this.drive.simplePIDTo(() -> this.objectiveTracker.getScoreProcessorObjective().getTargetPose().getOurs());
        final Command autoDriveClimb = this.drive.simplePIDTo(() -> this.objectiveTracker.getClimbObjective().getTargetPose().getOurs());
        CommandScheduler.getInstance().getDefaultButtonLoop().bind(() -> {
            if (driveController.hid.getRightBumperButtonPressed()) {
                if (this.objectiveTracker.getCurrentObjective().isEmpty()) return;
                switch (this.objectiveTracker.getCurrentObjective().get().getObjectiveType()) {
                    case ScoreCoral: autoDriveScoreCoral.schedule(); break;
                    case IntakeAlgae: autoDriveIntakeAlgae.schedule(); break;
                    case IntakeCoral: autoDriveIntakeCoral.schedule(); break;
                    case ScoreNet: autoDriveScoreNet.schedule(); break;
                    case ScoreProcessor: autoDriveScoreProcessor.schedule(); break;
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
                if (autoDriveScoreNet.isScheduled()) {
                    autoDriveScoreNet.cancel();
                }
                if (autoDriveScoreProcessor.isScheduled()) {
                    autoDriveScoreProcessor.cancel();
                }
                if (autoDriveClimb.isScheduled()) {
                    autoDriveClimb.cancel();
                }
            }
        });

        // Climb
        driveController.start().toggleOnTrue(
            Commands.parallel(
                this.climber.prepareClimb(),
                this.superstructure.goToSetpointSequenced(SuperstructureConstants.prepareClimbingState)
            )
            .deadlineFor(
                this.objectiveTracker.setTypeOverrideCommand(ObjectiveType.Climb)
            )
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

        // Self Right
        // TODO: REIMPLEMENT SELF RIGHT WITH PROPER GYRO INTERFACE
        // var selfRightCommand = this.superstructure.goToSetpointSequenced(SuperstructureConstants.selfRightingState);
        // // var prepareSelfRightCommand = superstructure.goToSetpointSequenced(SuperstructureConstants.prepareSelfRightingState);
        // CommandScheduler.getInstance().getDefaultButtonLoop().bind(new Runnable() {
        //     private boolean prevSelfRight = true;
        //     // private boolean prevprepare = true;
        //     public void run() {
        //         var selfRightButton = driveController.hid.getPOV() == 0;
        //         // var prepare = driveController.hid.getPOV() == 90;
        //         var tipped = !MeasureUtil.isNear(Degrees.of(0), drive.getPitch(), Degrees.of(45));
        //         Leds.getInstance().tipped.setFlag(tipped);
        //         if (selfRightButton && !this.prevSelfRight) {
        //             if (selfRightCommand.isScheduled()) {
        //                 selfRightCommand.cancel();
        //             } else {
        //                 if (tipped) {
        //                     selfRightCommand.schedule();
        //                 }
        //             }
        //         }
        //         if (selfRightCommand.isScheduled() && !tipped) {
        //             selfRightCommand.cancel();
        //         }
        //         // if (prepare && !prevprepare) {
        //         //     if (prepareSelfRightCommand.isScheduled()) {
        //         //         prepareSelfRightCommand.cancel();
        //         //     } else {
        //         //         // if (tipped) {
        //         //             prepareSelfRightCommand.schedule();
        //         //         // }
        //         //     }
        //         // }
        //         this.prevSelfRight = selfRightButton;
        //         // prevprepare = prepare;
        //     }
        // });
        
        driveController.leftStickButton().and(driveController.rightStickButton()).onTrue(Commands.runOnce(() -> this.setPose(Reef.reefs.getOurs().racks[0].centerRobotPose.getForward())).ignoringDisable(true));
        // new Trigger(() -> apriltagVision.getPose().xyStdDev() < .5)
        //     .onTrue(Commands.runOnce(() -> this.setPose(apriltagVision.getPose().robotPose())));

        SmartDashboard.putData("QuestNav/Quest Calibrate", questNav.determineOffsetToRobotCenter(drive));

        SmartDashboard.putData("Superstructure/Coast", this.superstructure.coast());

        this.automationsLoop.bind(() -> {
            this.objectiveTracker.determineGoal(
                RobotState.getInstance().getEstimatedGlobalPose(),
                this.intake.hasCoral(),
                this.intake.hasAlgae()
            );
        });

        this.automationsLoop.bind(() -> {
            LoggedTracer.logEpoch("CommandScheduler Periodic/Automations/Tilt Limits/Before");
            this.drive.setTiltLimits(
                (this.superstructure.elevator.getLengthMeters() > Units.inchesToMeters(40)) ? (
                    Drive.extendedTiltLimitTunable.get()
                ) : (
                    Drive.normalTiltLimitTunable.get()
                )
            );
            LoggedTracer.logEpoch("CommandScheduler Periodic/Automations/Tilt Limits");
        });

        // Self Record Coral
        this.automationsLoop.bind(new Runnable() {
            private static final LoggedTunable<Angle> l4PivotTolerance = LoggedTunable.from("Self Record/Coral/L4/Superstructure/Pivot Tolerance", Degrees::of, 2);
            private static final LoggedTunable<Distance> l4ElevatorTolerance = LoggedTunable.from("Self Record/Coral/L4/Superstructure/Elevator Tolerance", Inches::of, 2);
            private static final LoggedTunable<Angle> l4WristTolerance = LoggedTunable.from("Self Record/Coral/L4/Superstructure/Wrist Tolerance", Degrees::of, 5);
            private static final LoggedTunable<Distance> l4LinearTolerance = LoggedTunable.from("Self Record/Coral/L4/Robot/Linear Tolerance", Inches::of, 2);
            private static final LoggedTunable<Angle> l4AngularTolerance = LoggedTunable.from("Self Record/Coral/L4/Robot/Angular Tolerance", Degrees::of, 5);
            
            private static final LoggedTunable<Angle> l3PivotTolerance = LoggedTunable.from("Self Record/Coral/L3/Superstructure/Pivot Tolerance", Degrees::of, 2);
            private static final LoggedTunable<Distance> l3ElevatorTolerance = LoggedTunable.from("Self Record/Coral/L3/Superstructure/Elevator Tolerance", Inches::of, 2);
            private static final LoggedTunable<Angle> l3WristTolerance = LoggedTunable.from("Self Record/Coral/L3/Superstructure/Wrist Tolerance", Degrees::of, 5);
            private static final LoggedTunable<Distance> l3LinearTolerance = LoggedTunable.from("Self Record/Coral/L3/Robot/Linear Tolerance", Inches::of, 2);
            private static final LoggedTunable<Angle> l3AngularTolerance = LoggedTunable.from("Self Record/Coral/L3/Robot/Angular Tolerance", Degrees::of, 5);
            
            private static final LoggedTunable<Angle> l2PivotTolerance = LoggedTunable.from("Self Record/Coral/L2/Superstructure/Pivot Tolerance", Degrees::of, 2);
            private static final LoggedTunable<Distance> l2ElevatorTolerance = LoggedTunable.from("Self Record/Coral/L2/Superstructure/Elevator Tolerance", Inches::of, 2);
            private static final LoggedTunable<Angle> l2WristTolerance = LoggedTunable.from("Self Record/Coral/L2/Superstructure/Wrist Tolerance", Degrees::of, 5);
            private static final LoggedTunable<Distance> l2LinearTolerance = LoggedTunable.from("Self Record/Coral/L2/Robot/Linear Tolerance", Inches::of, 2);
            private static final LoggedTunable<Angle> l2AngularTolerance = LoggedTunable.from("Self Record/Coral/L2/Robot/Angular Tolerance", Degrees::of, 5);
            
            private static final LoggedTunable<Angle> l1PivotTolerance = LoggedTunable.from("Self Record/Coral/L1/Superstructure/Pivot Tolerance", Degrees::of, 5);
            private static final LoggedTunable<Distance> l1ElevatorTolerance = LoggedTunable.from("Self Record/Coral/L1/Superstructure/Elevator Tolerance", Inches::of, 2);
            private static final LoggedTunable<Angle> l1WristTolerance = LoggedTunable.from("Self Record/Coral/L1/Superstructure/Wrist Tolerance", Degrees::of, 5);
            private static final LoggedTunable<Distance> l1LinearTolerance = LoggedTunable.from("Self Record/Coral/L1/Robot/Linear Tolerance", Inches::of, 6);
            private static final LoggedTunable<Angle> l1AngularTolerance = LoggedTunable.from("Self Record/Coral/L1/Robot/Angular Tolerance", Degrees::of, 10);
            
            private final EdgeDetector coralEdgeDetector = new EdgeDetector();
            @Override
            public void run() {
                LoggedTracer.logEpoch("CommandScheduler Periodic/Automations/Self Record Coral/Before");
                this.coralEdgeDetector.update(intake.hasCoral());
                if (manualOverrides.selfRecordCoralDisabled()) {
                    LoggedTracer.logEpoch("CommandScheduler Periodic/Automations/Self Record Coral");
                    return;
                }

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
                    var pivotInTolerance = MathUtil.isNear(scoreCoralObjective.getTargetState().getPivotAngleRads(), superstructure.getCurrentMeasuredState().getPivotAngleRads(), pivotTolerance.in(Radians));
                    var elevatorInTolerance = MathUtil.isNear(scoreCoralObjective.getTargetState().getElevatorLengthMeters(), superstructure.getCurrentMeasuredState().getElevatorLengthMeters(), elevatorTolerance.in(Meters));
                    var wristInTolerance = MathUtil.isNear(scoreCoralObjective.getTargetState().getWristAngleRads(), superstructure.getCurrentMeasuredState().getWristAngleRads(), wristTolerance.in(Radians));
                    var linearInTolerance = GeomUtil.isNear(scoreCoralObjective.getTargetPose().getOurs().getTranslation(), RobotState.getInstance().getEstimatedGlobalPose().getTranslation(), linearTolerance);
                    var angularInTolerance = GeomUtil.isNear(scoreCoralObjective.getTargetPose().getOurs().getRotation(), RobotState.getInstance().getEstimatedGlobalPose().getRotation(), angularTolerance);
                    Logger.recordOutput("Self Record/Coral/Superstructure/Pivot", pivotInTolerance);
                    Logger.recordOutput("Self Record/Coral/Superstructure/Elevator", elevatorInTolerance);
                    Logger.recordOutput("Self Record/Coral/Superstructure/Wrist", wristInTolerance);
                    Logger.recordOutput("Self Record/Coral/Robot/Linear", linearInTolerance);
                    Logger.recordOutput("Self Record/Coral/Robot/Angular", angularInTolerance);
                    if (
                        pivotInTolerance
                        && elevatorInTolerance
                        && wristInTolerance
                        && linearInTolerance
                        && angularInTolerance
                    ) {
                        objectiveTracker.placeCoral(scoreCoralObjective.getTargetBranch());
                    }
                }
                LoggedTracer.logEpoch("CommandScheduler Periodic/Automations/Self Record Coral");
            }
        });
        // Self Record Algae
        this.automationsLoop.bind(new Runnable() {
            private static final LoggedTunable<Angle> lowPivotTolerance = LoggedTunable.from("Self Record/Algae/Low/Superstructure/Pivot Tolerance", Degrees::of, 5);
            private static final LoggedTunable<Distance> lowElevatorTolerance = LoggedTunable.from("Self Record/Algae/Low/Superstructure/Elevator Tolerance", Inches::of, 4);
            private static final LoggedTunable<Angle> lowWristTolerance = LoggedTunable.from("Self Record/Algae/Low/Superstructure/Wrist Tolerance", Degrees::of, 15);
            private static final LoggedTunable<Distance> lowLinearTolerance = LoggedTunable.from("Self Record/Algae/Low/Robot/Linear Tolerance", Inches::of, 12);
            private static final LoggedTunable<Angle> lowAngularTolerance = LoggedTunable.from("Self Record/Algae/Low/Robot/Angular Tolerance", Degrees::of, 30);
            
            private static final LoggedTunable<Angle> highPivotTolerance = LoggedTunable.from("Self Record/Algae/High/Superstructure/Pivot Tolerance", Degrees::of, 5);
            private static final LoggedTunable<Distance> highElevatorTolerance = LoggedTunable.from("Self Record/Algae/High/Superstructure/Elevator Tolerance", Inches::of, 4);
            private static final LoggedTunable<Angle> highWristTolerance = LoggedTunable.from("Self Record/Algae/High/Superstructure/Wrist Tolerance", Degrees::of, 15);
            private static final LoggedTunable<Distance> highLinearTolerance = LoggedTunable.from("Self Record/Algae/High/Robot/Linear Tolerance", Inches::of, 12);
            private static final LoggedTunable<Angle> highAngularTolerance = LoggedTunable.from("Self Record/Algae/High/Robot/Angular Tolerance", Degrees::of, 30);

            private final EdgeDetector algaeEdgeDetector = new EdgeDetector();
            @Override
            public void run() {
                LoggedTracer.logEpoch("CommandScheduler Periodic/Automations/Self Record Algae/Before");
                this.algaeEdgeDetector.update(intake.hasAlgae());
                if (manualOverrides.selfRecordAlgaeDisabled()) {
                    LoggedTracer.logEpoch("CommandScheduler Periodic/Automations/Self Record Algae");
                    return;
                }

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
                    var pivotInTolerance = MathUtil.isNear(intakeAlgaeObjective.get().getTargetState().getPivotAngleRads(), superstructure.getCurrentMeasuredState().getPivotAngleRads(), pivotTolerance.in(Radians));
                    var elevatorInTolerance = MathUtil.isNear(intakeAlgaeObjective.get().getTargetState().getElevatorLengthMeters(), superstructure.getCurrentMeasuredState().getElevatorLengthMeters(), elevatorTolerance.in(Meters));
                    var wristInTolerance = MathUtil.isNear(intakeAlgaeObjective.get().getTargetState().getWristAngleRads(), superstructure.getCurrentMeasuredState().getWristAngleRads(), wristTolerance.in(Radians));
                    var linearInTolerance = GeomUtil.isNear(intakeAlgaeObjective.get().getTargetPose().getOurs().getTranslation(), RobotState.getInstance().getEstimatedGlobalPose().getTranslation(), linearTolerance);
                    var angularInTolerance = GeomUtil.isNear(intakeAlgaeObjective.get().getTargetPose().getOurs().getRotation(), RobotState.getInstance().getEstimatedGlobalPose().getRotation(), angularTolerance);
                    Logger.recordOutput("Self Record/Coral/Superstructure/Pivot", pivotInTolerance);
                    Logger.recordOutput("Self Record/Coral/Superstructure/Elevator", elevatorInTolerance);
                    Logger.recordOutput("Self Record/Coral/Superstructure/Wrist", wristInTolerance);
                    Logger.recordOutput("Self Record/Coral/Robot/Linear", linearInTolerance);
                    Logger.recordOutput("Self Record/Coral/Robot/Angular", angularInTolerance);
                    if (
                        pivotInTolerance
                        && elevatorInTolerance
                        && wristInTolerance
                        && linearInTolerance
                        && angularInTolerance
                    ) {
                        objectiveTracker.removeAlgae(intakeAlgaeObjective.get().getTargetAlgae());
                    }
                }
                LoggedTracer.logEpoch("CommandScheduler Periodic/Automations/Self Record Algae");
            }
        });

        // Auto Eject Coral
        this.automationsLoop.bind(new Runnable() {
            private static final LoggedTunable<Angle> l4PivotTolerance = LoggedTunable.from("Auto Eject/Coral/L4/Superstructure/Pivot Tolerance", Degrees::of, 2);
            private static final LoggedTunable<Distance> l4ElevatorTolerance = LoggedTunable.from("Auto Eject/Coral/L4/Superstructure/Elevator Tolerance", Inches::of, 2);
            private static final LoggedTunable<Angle> l4WristTolerance = LoggedTunable.from("Auto Eject/Coral/L4/Superstructure/Wrist Tolerance", Degrees::of, 5);
            private static final LoggedTunable<Distance> l4LinearTolerance = LoggedTunable.from("Auto Eject/Coral/L4/Robot/Linear Tolerance", Inches::of, 2);
            private static final LoggedTunable<Angle> l4AngularTolerance = LoggedTunable.from("Auto Eject/Coral/L4/Robot/Angular Tolerance", Degrees::of, 5);
            
            private static final LoggedTunable<Angle> l3PivotTolerance = LoggedTunable.from("Auto Eject/Coral/L3/Superstructure/Pivot Tolerance", Degrees::of, 2);
            private static final LoggedTunable<Distance> l3ElevatorTolerance = LoggedTunable.from("Auto Eject/Coral/L3/Superstructure/Elevator Tolerance", Inches::of, 2);
            private static final LoggedTunable<Angle> l3WristTolerance = LoggedTunable.from("Auto Eject/Coral/L3/Superstructure/Wrist Tolerance", Degrees::of, 5);
            private static final LoggedTunable<Distance> l3LinearTolerance = LoggedTunable.from("Auto Eject/Coral/L3/Robot/Linear Tolerance", Inches::of, 2);
            private static final LoggedTunable<Angle> l3AngularTolerance = LoggedTunable.from("Auto Eject/Coral/L3/Robot/Angular Tolerance", Degrees::of, 5);

            private static final LoggedTunable<Angle> l2PivotTolerance = LoggedTunable.from("Auto Eject/Coral/L2/Superstructure/Pivot Tolerance", Degrees::of, 2);
            private static final LoggedTunable<Distance> l2ElevatorTolerance = LoggedTunable.from("Auto Eject/Coral/L2/Superstructure/Elevator Tolerance", Inches::of, 2);
            private static final LoggedTunable<Angle> l2WristTolerance = LoggedTunable.from("Auto Eject/Coral/L2/Superstructure/Wrist Tolerance", Degrees::of, 5);
            private static final LoggedTunable<Distance> l2LinearTolerance = LoggedTunable.from("Auto Eject/Coral/L2/Robot/Linear Tolerance", Inches::of, 2);
            private static final LoggedTunable<Angle> l2AngularTolerance = LoggedTunable.from("Auto Eject/Coral/L2/Robot/Angular Tolerance", Degrees::of, 5);

            private static final LoggedTunable<Angle> l1PivotTolerance = LoggedTunable.from("Auto Eject/Coral/L1/Superstructure/Pivot Tolerance", Degrees::of, 2);
            private static final LoggedTunable<Distance> l1ElevatorTolerance = LoggedTunable.from("Auto Eject/Coral/L1/Superstructure/Elevator Tolerance", Inches::of, 2);
            private static final LoggedTunable<Angle> l1WristTolerance = LoggedTunable.from("Auto Eject/Coral/L1/Superstructure/Wrist Tolerance", Degrees::of, 5);
            private static final LoggedTunable<Distance> l1LinearTolerance = LoggedTunable.from("Auto Eject/Coral/L1/Robot/Linear Tolerance", Inches::of, 2);
            private static final LoggedTunable<Angle> l1AngularTolerance = LoggedTunable.from("Auto Eject/Coral/L1/Robot/Angular Tolerance", Degrees::of, 5);

            private final Command ejectBranch = intake.eject();
            private final Command ejectL1 = intake.ejectL1();

            private final Debouncer debouncer = new Debouncer(0.5, DebounceType.kRising);

            @Override
            public void run() {
                if (intake.hasCoral() && !manualOverrides.autoEjectCoralDisabled()) {
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
                    var pivotInTolerance = MathUtil.isNear(scoreCoralObjective.getTargetState().getPivotAngleRads(), superstructure.getCurrentMeasuredState().getPivotAngleRads(), pivotTolerance.in(Radians));
                    var elevatorInTolerance = MathUtil.isNear(scoreCoralObjective.getTargetState().getElevatorLengthMeters(), superstructure.getCurrentMeasuredState().getElevatorLengthMeters(), elevatorTolerance.in(Meters));
                    var wristInTolerance = MathUtil.isNear(scoreCoralObjective.getTargetState().getWristAngleRads(), superstructure.getCurrentMeasuredState().getWristAngleRads(), wristTolerance.in(Radians));
                    var linearInTolerance = GeomUtil.isNear(scoreCoralObjective.getTargetPose().getOurs().getTranslation(), RobotState.getInstance().getEstimatedGlobalPose().getTranslation(), linearTolerance);
                    var angularInTolerance = GeomUtil.isNear(scoreCoralObjective.getTargetPose().getOurs().getRotation(), RobotState.getInstance().getEstimatedGlobalPose().getRotation(), angularTolerance);
                    Logger.recordOutput("Self Record/Coral/Superstructure/Pivot", pivotInTolerance);
                    Logger.recordOutput("Self Record/Coral/Superstructure/Elevator", elevatorInTolerance);
                    Logger.recordOutput("Self Record/Coral/Superstructure/Wrist", wristInTolerance);
                    Logger.recordOutput("Self Record/Coral/Robot/Linear", linearInTolerance);
                    Logger.recordOutput("Self Record/Coral/Robot/Angular", angularInTolerance);

                    if (this.debouncer.calculate(pivotInTolerance && elevatorInTolerance && wristInTolerance && linearInTolerance && angularInTolerance)) {
                        if (scoreCoralObjective.getTargetBranch().isPresent()) {
                            if (!this.ejectBranch.isScheduled()) {
                                this.ejectBranch.schedule();
                            }
                        } else {
                            if (!this.ejectL1.isScheduled()) {
                                this.ejectL1.schedule();
                            }
                        }
                    }
                } else {
                    if (this.ejectBranch.isScheduled()) {
                        this.ejectBranch.cancel();
                    }
                    if (this.ejectL1.isScheduled()) {
                        this.ejectL1.cancel();
                    }
                }
                LoggedTracer.logEpoch("CommandScheduler Periodic/Automations/Auto Eject Coral");
            }
        });
    }

    private void setPose(Pose2d pose) {
        this.questNav.setPose(pose);
        RobotState.getInstance().resetPose(pose);
    }

    private void configureNotifications() {
        new Trigger(this.intake::hasCoral)
            .onTrue(
                Leds.getInstance().coralAcquired.setFlagCommand().withTimeout(1).alongWith(driveController.rumble(RumbleType.kBothRumble, 0.3).withTimeout(1))
            )
            .whileTrue(
                Leds.getInstance().coralSecured.setFlagCommand().ignoringDisable(true)
            )
        ;
        new Trigger(this.intake::hasAlgae)
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
