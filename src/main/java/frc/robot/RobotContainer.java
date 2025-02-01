// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.auto.AutoCommons.AutoPaths;
import frc.robot.auto.AutoManager;
import frc.robot.auto.AutoSelector;
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
import frc.robot.subsystems.manualOverrides.ManualOverrides;
import frc.robot.subsystems.objectiveTracker.ObjectiveSelectorIOServer;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.SuperstructureSetpoint;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.ElevatorIO;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOSim;
import frc.robot.subsystems.superstructure.pivot.Pivot;
import frc.robot.subsystems.superstructure.pivot.PivotIO;
import frc.robot.subsystems.superstructure.pivot.PivotIOSim;
import frc.robot.subsystems.superstructure.wrist.Wrist;
import frc.robot.subsystems.superstructure.wrist.WristIO;
import frc.robot.subsystems.superstructure.wrist.WristIOSim;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.apriltag.ApriltagVision;
import frc.robot.subsystems.vision.bucket.BucketVision;
import frc.util.commands.ContinuouslySwappingCommand;
import frc.util.controllers.ButtonBoard3x3;
import frc.util.controllers.Joystick;
import frc.util.controllers.XboxController;
import frc.util.robotStructure.Mechanism3d;

public class RobotContainer {
    // Subsystems
    public final Drive drive;
    public final Superstructure superstructure;
    public final ApriltagVision apriltagVision;
    public final BucketVision bucketVision;
    public final ManualOverrides manualOverrides;
    public final ObjectiveTracker objectiveTracker;

    // Controllers
    private final XboxController driveController = new XboxController(0);
    private final Joystick driveJoystick;
    private final Supplier<ChassisSpeeds> joystickTranslational;
    @SuppressWarnings("unused")
    private final ButtonBoard3x3 buttonBoard = new ButtonBoard3x3(1);
    @SuppressWarnings("unused")
    private final CommandJoystick simJoystick = new CommandJoystick(2);

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
                            .addChild(superstructure.wrist.mech)
                        )
                    )
                )
            )
        ;
        Mechanism3d.registerMechs(superstructure.pivot.mech, superstructure.elevator.stage2Mech, superstructure.elevator.stage3Mech, superstructure.elevator.stage4Mech, superstructure.wrist.mech);

        driveJoystick = driveController.leftStick
            .smoothRadialDeadband(DriveConstants.driveJoystickDeadbandPercent)
            .radialSensitivity(0.75)
            // .radialSlewRateLimit(DriveConstants.joystickSlewRateLimit)
        ;

        joystickTranslational = Drive.Translational.joystickSpectatorToFieldRelative(
            driveJoystick,
            () -> false
        );

        System.out.println("[Init RobotContainer] Configuring Default Subsystem Commands");
        configureSubsystems();

        System.out.println("[Init RobotContainer] Configuring Controls");
        configureControls();

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

    private void configureSubsystems() {
        drive.translationSubsystem.setDefaultCommand(
            drive.translationSubsystem.fieldRelative(joystickTranslational)
                .withName("Driver Control Field Relative")
        );
        drive.rotationalSubsystem.setDefaultCommand(
            drive.rotationalSubsystem.spin(driveController.rightStick.x().smoothDeadband(0.2).multiply(DriveConstants.maxTurnRate.in(RadiansPerSecond)).multiply(0.25))
                .withName("Robot spin")
        );

        superstructure.pivot.setDefaultCommand(superstructure.pivot.pivotTo(Degrees.of(0)));
        superstructure.elevator.setDefaultCommand(superstructure.elevator.elevateTo(Meters.zero()));
        superstructure.wrist.setDefaultCommand(superstructure.wrist.pivotTo(Degrees.zero()));
    }

    private void configureControls() {
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


        driveController.b().toggleOnTrue(superstructure.pivotVoltage(() -> (driveController.leftTrigger.getAsDouble() - driveController.rightTrigger.getAsDouble()) * 12));
        driveController.x().toggleOnTrue(superstructure.elevatorVoltage(() -> (driveController.leftTrigger.getAsDouble() - driveController.rightTrigger.getAsDouble()) * 12));
        
        driveController.y().toggleOnTrue(superstructure.pivot.pivotTo(Degrees.of(90)));
        driveController.y().toggleOnTrue(superstructure.elevator.elevateTo(Meters.of(1)));
        driveController.leftBumper().toggleOnTrue(new ContinuouslySwappingCommand(
            new Supplier<Command>() {
                private final Command[] commands = new Command[Level.values().length * 2];
                {
                    for (var level : Level.values()) {
                        commands[level.ordinal() * 2] = superstructure.goToSetpointSequenced(SuperstructureSetpoint.fromLevelForward(level));
                        commands[level.ordinal() * 2 + 1] = superstructure.goToSetpointSequenced(SuperstructureSetpoint.fromLevelBackward(level));
                    }
                }
                public Command get() {
                    var node = objectiveTracker.getSelectedNode();
                    if (drive.getRotation().minus(node.pose.getOurs().getRotation().toRotation2d()).getCos() >= 0) {
                        return commands[node.level.ordinal() * 2];
                    } else {
                        return commands[node.level.ordinal() * 2 + 1];
                    }
                }
            },
            Set.of(superstructure.pivot, superstructure.elevator, superstructure.wrist)
        ));
        driveController.rightBumper().toggleOnTrue(new ContinuouslySwappingCommand(
            new Supplier<Command>() {
                private final Command[] commands = new Command[Rack.values().length * 2];
                {
                    for (var rack : Rack.values()) {
                        commands[rack.ordinal() * 2] = superstructure.goToSetpointSequenced(SuperstructureSetpoint.fromAlgaeForward(rack.algaeLevel));
                        commands[rack.ordinal() * 2 + 1] = superstructure.goToSetpointSequenced(SuperstructureSetpoint.fromAlgaeBackward(rack.algaeLevel));
                    }
                }
                public Command get() {
                    var rack = Rack.Rack2;
                    if (drive.getRotation().minus(rack.getAlgaePose().getOurs().getRotation().toRotation2d()).getCos() >= 0) {
                        return commands[rack.ordinal() * 2];
                    } else {
                        return commands[rack.ordinal() * 2 + 1];
                    }
                }
            },
            Set.of(superstructure.pivot, superstructure.elevator, superstructure.wrist)
        ));

        driveController.a().onTrue(Commands.runOnce(() -> objectiveTracker.toggleSelectedNode()));
        driveController.povUp().onTrue(Commands.runOnce(() -> objectiveTracker.moveSelectedCoral(0, 1)));
        driveController.povDown().onTrue(Commands.runOnce(() -> objectiveTracker.moveSelectedCoral(0, -1)));
        driveController.povLeft().onTrue(Commands.runOnce(() -> objectiveTracker.moveSelectedCoral(-1, 0)));
        driveController.povRight().onTrue(Commands.runOnce(() -> objectiveTracker.moveSelectedCoral(1, 0)));
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
