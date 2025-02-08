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
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.manualOverrides.ManualOverrides;
import frc.robot.subsystems.objectiveTracker.ObjectiveSelectorIOServer;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.ElevatorIO;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOSim;
import frc.robot.subsystems.superstructure.pivot.Pivot;
import frc.robot.subsystems.superstructure.pivot.PivotConstants;
import frc.robot.subsystems.superstructure.pivot.PivotIO;
import frc.robot.subsystems.superstructure.pivot.PivotIOSim;
import frc.robot.subsystems.superstructure.wrist.Wrist;
import frc.robot.subsystems.superstructure.wrist.WristIO;
import frc.robot.subsystems.superstructure.wrist.WristIOSim;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.apriltag.ApriltagVision;
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
    public final BucketVision bucketVision;
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
                    new Pivot(new PivotIO() {}),
                    new Elevator(new ElevatorIO() {}),
                    new Wrist(new WristIO() {})
                );
                intake = new Intake(new IntakeIO() {});
                apriltagVision = new ApriltagVision(
                    // new ApriltagCamera(
                    //     ApriltagVisionConstants.frontLeftApriltagCamera,
                    //     new ApriltagCameraIOPhotonVision(ApriltagVisionConstants.frontLeftApriltagCamera)
                    // ),
                    // new ApriltagCamera(
                    //     ApriltagVisionConstants.frontRightApriltagCamera,
                    //     new ApriltagCameraIOPhotonVision(ApriltagVisionConstants.frontRightApriltagCamera)
                    // ),
                    // new ApriltagCamera(
                    //     ApriltagVisionConstants.backLeftApriltagCamera,
                    //     new ApriltagCameraIOPhotonVision(ApriltagVisionConstants.backLeftApriltagCamera)
                    // ),
                    // new ApriltagCamera(
                    //     ApriltagVisionConstants.backRightApriltagCamera,
                    //     new ApriltagCameraIOPhotonVision(ApriltagVisionConstants.backRightApriltagCamera)
                    // )
                );
                bucketVision = new BucketVision(
                    // new BucketCamera(
                    //     BucketVisionConstants.bucketCamera,
                    //     new BucketCameraIOPhotonVision(BucketVisionConstants.bucketCamera)
                    // )
                );
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
                bucketVision = new BucketVision();
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
                bucketVision = new BucketVision();
            break;
        }
        manualOverrides = new ManualOverrides();
        objectiveTracker = new ObjectiveTracker(new ObjectiveSelectorIOServer());

        drive.structureRoot
            .addChild(VisionConstants.frontLeftModuleMount)
            .addChild(VisionConstants.frontRightModuleMount)
            .addChild(VisionConstants.backLeftModuleMount)
            .addChild(VisionConstants.backRightModuleMount)
            .addChild(VisionConstants.flagStickMount)
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

        // var joystickTranslational = Drive.Translational.joystickSpectatorToFieldRelative(
        //     driveJoystick,
        //     () -> false
        // );

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

        superstructure.setDefaultCommand(superstructure.idle());
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

        driveController.a().onTrue(Commands.runOnce(() -> objectiveTracker.toggleSelectedNode()));
        driveController.povUp().onTrue(Commands.runOnce(() -> objectiveTracker.moveSelectedCoral(0, 1)));
        driveController.povDown().onTrue(Commands.runOnce(() -> objectiveTracker.moveSelectedCoral(0, -1)));
        driveController.povLeft().onTrue(Commands.runOnce(() -> objectiveTracker.moveSelectedCoral(-1, 0)));
        driveController.povRight().onTrue(Commands.runOnce(() -> objectiveTracker.moveSelectedCoral(1, 0)));
        
        // driveController.a().toggleOnTrue(null); //Intake/Eject
        driveController.y().toggleOnTrue(superstructure.defense()); //Defense
        
        driveController.x().and(intake.hasCoral).toggleOnTrue(new ContinuouslySwappingCommand( //Extend
            new Supplier<Command>() {
                private final Command[] commands = new Command[Level.values().length * 2];
                {
                    for (var level : Level.values()) {
                        commands[level.ordinal() * 2] = superstructure.goToSetpointSequenced(SuperstructureState.fromLevelForward(level));
                        commands[level.ordinal() * 2 + 1] = superstructure.goToSetpointSequenced(SuperstructureState.fromLevelBackward(level));
                    }
                }
                public Command get() {
                    var branch = objectiveTracker.getSelectedBranch();
                    if (drive.getRotation().minus(branch.branchPose.getOurs().getRotation().toRotation2d()).getCos() >= 0) {
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
                private final Command processorCommand = superstructure.goToSetpointSequenced(new SuperstructureState(PivotConstants.minAngle, Meters.zero(), Degrees.of(35).unaryMinus()));
                private final Command netForwardCommand = superstructure.goToSetpointSequenced(new SuperstructureState(Degrees.of(90), Meters.zero(), Degrees.of(45).unaryMinus()));
                private final Command netBackwardCommand = superstructure.goToSetpointSequenced(new SuperstructureState(Degrees.of(90), Meters.zero(), Degrees.of(45)));
                public Command get() {
                    switch (objectiveTracker.getAlgaeGoal()) {
                        default:
                        case NET:
                            if (drive.getRotation().minus(FieldConstants.netForwardRotation.getOurs()).getCos() >= 0) {
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
        driveController.rightBumper().whileTrue(drive.rotationalSubsystem.pidControlledHeading(() -> Optional.of(objectiveTracker.getSelectedBranch().robotPose.getOurs().getRotation()))); //Auto drive
        // driveController.start().toggleOnTrue(null); //Start Climb
        // driveController.back().toggleOnTrue(null); //Climb
        
        driveController.leftStickButton().onTrue(Commands.runOnce(() -> drive.setPose(FieldConstants.Reef.getStagedAlgae(Rack.Rack0).robotPose.getOurs())));
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
