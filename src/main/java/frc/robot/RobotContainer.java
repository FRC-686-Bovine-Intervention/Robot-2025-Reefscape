// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.Arrays;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.auto.AutoCommons.AutoPaths;
import frc.robot.auto.AutoManager;
import frc.robot.auto.AutoSelector;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.CoralStation;
import frc.robot.constants.FieldConstants.Reef.AlgaeLevel;
import frc.robot.constants.FieldConstants.Reef.Level;
import frc.robot.constants.FieldConstants.Reef.Rack;
import frc.robot.constants.RobotConstants;
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
import frc.robot.subsystems.superstructure.Superstructure.RobotFlippedRobotPose;
import frc.robot.subsystems.superstructure.Superstructure.RobotFlippedSuperstructureState;
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
import frc.robot.subsystems.vision.apriltag.ApriltagCameraIOPhotonVision;
import frc.robot.subsystems.vision.apriltag.ApriltagVision;
import frc.robot.subsystems.vision.apriltag.ApriltagVisionConstants;
import frc.robot.subsystems.vision.bucket.BucketVision;
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
    public final ApriltagVision apriltagVision;
    // public final BucketVision bucketVision;
    public final ManualOverrides manualOverrides;
    public final ObjectiveTracker objectiveTracker;

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
                // bucketVision = new BucketVision(
                //     // new BucketCamera(
                //     //     BucketVisionConstants.bucketCamera,
                //     //     new BucketCameraIOPhotonVision(BucketVisionConstants.bucketCamera)
                //     // )
                // );
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
                apriltagVision = new ApriltagVision();
                // bucketVision = new BucketVision();
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
                apriltagVision = new ApriltagVision();
                // bucketVision = new BucketVision();
                objectiveTracker = new ObjectiveTracker(new ObjectiveSelectorIO() {});
            break;
        }
        manualOverrides = new ManualOverrides();
        

        drive.structureRoot
            .addChild(VisionConstants.frontLeftMount)
            .addChild(VisionConstants.frontRightMount)
            .addChild(VisionConstants.backLeftMount)
            .addChild(VisionConstants.backRightMount)
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
                var robotSpeeds = new ChassisSpeeds(
                    Math.min(driveController.leftTrigger.getAsDouble(), driveController.rightTrigger.getAsDouble()) * DriveConstants.maxAdjustmentSpeed.in(MetersPerSecond),
                    (driveController.leftTrigger.getAsDouble() - driveController.rightTrigger.getAsDouble()) * DriveConstants.maxAdjustmentSpeed.in(MetersPerSecond),
                    0
                );
                drive.translationSubsystem.driveVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(fieldSpeeds, drive.getRotation()).plus(robotSpeeds));
            })
            .withName("Driver Control Field Relative")
        );
        drive.rotationalSubsystem.setDefaultCommand(
            drive.rotationalSubsystem.spin(driveController.rightStick.x().smoothDeadband(0.2).multiply(DriveConstants.maxTurnRate.in(RadiansPerSecond)).multiply(0.25))
                .withName("Robot spin")
        );

        // superstructure.setDefaultCommand(superstructure.throttle(driveController.leftStick.y(), driveController.rightStick.y(), driveController.leftTrigger.add(driveController.rightTrigger.invert())));
        // superstructure.setDefaultCommand(superstructure.goToSetpointSequenced(SuperstructureState.fromParts(Degrees.of(90), ElevatorConstants.minLength, Degrees.of(90))));
        superstructure.setDefaultCommand(superstructure.goToSetpointSequenced(SuperstructureState.idle));
        intake.setDefaultCommand(intake.idle());
        SmartDashboard.putData("Superstructure/Down", superstructure.goToSetpoint(SuperstructureState.newConstrained(Degrees.of(90), ElevatorConstants.minLength, Degrees.of(-60))));
        SmartDashboard.putData("Superstructure/Up", superstructure.goToSetpoint(SuperstructureState.newConstrained(Degrees.of(90), ElevatorConstants.minLength, Degrees.of(60))));
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
        driveController.povUp().onTrue(Commands.runOnce(() -> objectiveTracker.moveSelectedCoral(0, 1)));
        driveController.povDown().onTrue(Commands.runOnce(() -> objectiveTracker.moveSelectedCoral(0, -1)));
        driveController.povLeft().onTrue(Commands.runOnce(() -> objectiveTracker.moveSelectedCoral(-1, 0)));
        driveController.povRight().onTrue(Commands.runOnce(() -> objectiveTracker.moveSelectedCoral(1, 0)));
        
        driveController.a().toggleOnTrue(new ContinuouslySwappingCommand(
            new Supplier<Command>() {
                private final Command coralStationForwardCommand = superstructure.goToSetpointSequenced(CoralStation.intakePosition.getForward());
                private final Command coralStationBackwardCommand = superstructure.goToSetpointSequenced(CoralStation.intakePosition.getBackward());
                private final Command groundAlgaeCommand = superstructure.goToSetpointSequenced(SuperstructureState.defense);
                private final Command[] stagedAlgaeCommands = new Command[AlgaeLevel.values().length * 2];
                {
                    for (var level : AlgaeLevel.values()) {
                        stagedAlgaeCommands[level.ordinal() * 2] = superstructure.goToSetpointSequenced(level.superstructurePosition.getForward());
                        stagedAlgaeCommands[level.ordinal() * 2 + 1] = superstructure.goToSetpointSequenced(level.superstructurePosition.getBackward());
                    }
                }
                public Command get() {
                    if (objectiveTracker.intakeFromCoralStation()) {
                        var stationPoses = new RobotFlippedRobotPose[] {
                            CoralStation.leftStationLeft.getOurs(),
                            CoralStation.leftStationCenter.getOurs(),
                            CoralStation.leftStationRight.getOurs(),
                            CoralStation.rightStationLeft.getOurs(),
                            CoralStation.rightStationCenter.getOurs(),
                            CoralStation.rightStationRight.getOurs(),
                        };
                        var closestStationPose = Arrays.stream(stationPoses).sorted((a,b) -> {
                            var aDistance = a.getClosest(drive.getRotation()).getTranslation().getDistance(drive.getPose().getTranslation());
                            var bDistance = b.getClosest(drive.getRotation()).getTranslation().getDistance(drive.getPose().getTranslation());
                            return (int) Math.signum(aDistance - bDistance);
                        }).findFirst().get();
                        Logger.recordOutput("Closest Station/Robot", closestStationPose.getClosest(drive.getRotation()));
                        Logger.recordOutput("Closest Station/Mechs", CoralStation.intakePosition.getClosest(closestStationPose.getForward().getRotation(), drive.getRotation()).getMechTransforms());
                        if (RobotFlippedSuperstructureState.useForward(closestStationPose.getForward().getRotation(), drive.getRotation())) {
                            return coralStationForwardCommand;
                        } else {
                            return coralStationBackwardCommand;
                        }
                    } else {
                        var algae = objectiveTracker.getSelectedStagedAlgae().get();
                        if (algae.isEmpty()) {
                            return groundAlgaeCommand;
                        } else {
                            var stagedAlgae = algae.get();
                            if (RobotFlippedSuperstructureState.useForward(stagedAlgae.rack.algaeIntakeRobotPose.getOurs().getRotation(), drive.getRotation())) {
                                return stagedAlgaeCommands[stagedAlgae.algaeLevel.ordinal() * 2];
                            } else {
                                return stagedAlgaeCommands[stagedAlgae.algaeLevel.ordinal() * 2 + 1];
                            }
                        }
                    }
                }
            },
            Set.of(superstructure)
        ).alongWith(intake.intake())); //Intake/Eject
        driveController.b().whileTrue(intake.eject());
        driveController.y().toggleOnTrue(superstructure.defense()); //Defense
        driveController.x().toggleOnTrue(new ContinuouslySwappingCommand( //Extend
            new Supplier<Command>() {
                private final Command[] commands = new Command[Level.values().length * 2];
                {
                    for (var level : Level.values()) {
                        commands[level.ordinal() * 2] = superstructure.goToSetpointSequenced(level.superstructureStates.getForward());
                        commands[level.ordinal() * 2 + 1] = superstructure.goToSetpointSequenced(level.superstructureStates.getBackward());
                    }
                }
                public Command get() {
                    var branch = objectiveTracker.getSelectedBranch();
                    if (branch.pipe.robotPose.getOurs().useForward(drive.getRotation())) {
                        return commands[branch.level.ordinal() * 2];
                    } else {
                        return commands[branch.level.ordinal() * 2 + 1];
                    }
                }
            },
            Set.of(superstructure)
        ));
        driveController.x().and(intake.hasAlgae).toggleOnTrue(new ContinuouslySwappingCommand(
            new Supplier<Command>() {
                private final Command processorCommand = superstructure.goToSetpointSequenced(SuperstructureState.newConstrained(PivotConstants.minAngle, Meters.zero(), Degrees.of(35).unaryMinus()));
                private final Command netForwardCommand = superstructure.goToSetpointSequenced(SuperstructureState.newConstrained(Degrees.of(90), Meters.zero(), Degrees.of(45).unaryMinus()));
                private final Command netBackwardCommand = superstructure.goToSetpointSequenced(SuperstructureState.newConstrained(Degrees.of(90), Meters.zero(), Degrees.of(45)));
                public Command get() {
                    switch (objectiveTracker.getAlgaeGoal()) {
                        default:
                        case NET:
                            if (RobotFlippedSuperstructureState.useForward(FieldConstants.netForwardRotation.getOurs(), drive.getRotation())) {
                                return netForwardCommand;
                            } else {
                                return netBackwardCommand;
                            }
                        case PROCESSOR:
                        case OPPONENT_PROCESSOR:
                            return processorCommand;
                    }
                }
            },
            Set.of(superstructure)
        ));
        driveController.rightBumper().whileTrue(drive.rotationalSubsystem.pidControlledHeading(() -> Optional.of(objectiveTracker.getSelectedBranch().pipe.robotPose.getOurs().getClosest(drive.getRotation()).getRotation()))); //Auto drive
        // driveController.start().toggleOnTrue(null); //Start Climb
        // driveController.back().toggleOnTrue(null); //Climb
        
        driveController.leftStickButton().onTrue(Commands.runOnce(() -> drive.setPose(Rack.Rack0.algaeIntakeRobotPose.getOurs())));
    }

    private void configureNotifications() {}

    private void configureAutos() {
        AutoPaths.preload();
        var selector = new AutoSelector("Auto Selector");

        new AutoManager(selector);
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
