// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.Arrays;
import java.util.Optional;
import java.util.Queue;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import java.util.stream.IntStream;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.RobotState;
import frc.robot.RobotState.OdometryObservation;
import frc.robot.RobotType;
import frc.robot.RobotType.Mode;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.drive.DriveConstants.ModuleConstants;
import frc.util.LazyOptional;
import frc.util.LoggedTracer;
import frc.util.NeutralMode;
import frc.util.Perspective;
import frc.util.VirtualSubsystem;
import frc.util.controllers.Joystick;
import frc.util.flipping.AllianceFlipUtil;
import frc.util.geometry.GeomUtil;
import frc.util.loggerUtil.tunables.LoggedTunableMeasure;
import frc.util.loggerUtil.tunables.LoggedTunableNumber;
import frc.util.robotStructure.Root;

public class Drive extends VirtualSubsystem {
    public final Set<Subsystem> subsystems;
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private final Queue<Double> odometryTimestampQueue;
    private final OdometryTimestampInputsAutoLogged odometryTimestamps = new OdometryTimestampInputsAutoLogged();

    public final Root structureRoot = new Root();

    public final Module[] modules = new Module[DriveConstants.moduleConstants.length];

    private SwerveModulePosition[] lastMeasuredPositions = null;
    private SwerveModuleState[] measuredStates = new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
    };
    private ChassisSpeeds robotMeasuredSpeeds = new ChassisSpeeds();
    private ChassisSpeeds fieldMeasuredSpeeds = new ChassisSpeeds();

    private static final LoggedTunableNumber rotationCorrection = new LoggedTunableNumber("Drive/Rotation Correction", 0.125);

    private ChassisSpeeds desiredRobotSpeeds = new ChassisSpeeds();
    private Translation2d centerOfRotation = new Translation2d();
    private SwerveModuleState[] setpointStates = new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
    };

    private Twist2d fieldVelocity = new Twist2d();

    public Drive(GyroIO gyroIO, ModuleIO... moduleIOs) {
        System.out.println("[Init Drive] Instantiating Drive");
        this.gyroIO = gyroIO;
        System.out.println("[Init Drive] Gyro IO: " + this.gyroIO.getClass().getSimpleName());
        for(int i = 0; i < DriveConstants.moduleConstants.length; i++) {
            ModuleConstants config = DriveConstants.moduleConstants[i];
            System.out.println("[Init Drive] Instantiating Module " + config.name + " with Module IO: " + moduleIOs[i].getClass().getSimpleName());
            var module = new Module(moduleIOs[i], config);
            module.periodic();
            this.modules[i] = module;
        }

        this.odometryTimestampQueue = OdometryThread.getInstance().generateTimestampQueue();
        OdometryThread.getInstance().start();

        this.translationSubsystem = new Translational(this);
        this.rotationalSubsystem = new Rotational(this);
        this.subsystems = Set.of(this.translationSubsystem, this.rotationalSubsystem);
        AutoBuilder.configure(
            RobotState.getInstance()::getEstimatedGlobalPose,
            RobotState.getInstance()::resetPose,
            this::getRobotMeasuredSpeeds,
            this::runRobotSpeeds,
            autoConfig(),
            DriveConstants.robotConfig,
            AllianceFlipUtil::shouldFlip,
            this.translationSubsystem,
            this.rotationalSubsystem
        );

        var routine = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> {
                    Logger.recordOutput("SysID/Drive/State", state.toString());
                }
            ),
            new SysIdRoutine.Mechanism(
                (volts) -> {
                    Arrays.stream(this.modules).forEach((module) -> module.runVoltage(volts, Rotation2d.kZero));
                },
                (log) -> {
                    Arrays.stream(this.modules).forEach((module) -> {
                        Logger.recordOutput("SysID/Drive/" + module.config.name + "/Position", module.getWheelAngularPosition());
                        Logger.recordOutput("SysID/Drive/" + module.config.name + "/Velocity", module.getWheelAngularVelocity());
                        Logger.recordOutput("SysID/Drive/" + module.config.name + "/Voltage", module.getAppliedVoltage());
                    });
                },
                this.translationSubsystem
            )
        );

        SmartDashboard.putData("SysID/Drive/Quasi Forward", routine.quasistatic(Direction.kForward).alongWith(Commands.idle(this.rotationalSubsystem)).withName("SysID Quasistatic Forward").asProxy());
        SmartDashboard.putData("SysID/Drive/Quasi Reverse", routine.quasistatic(Direction.kReverse).alongWith(Commands.idle(this.rotationalSubsystem)).withName("SysID Quasistatic Reverse").asProxy());
        SmartDashboard.putData("SysID/Drive/Dynamic Forward", routine.dynamic(Direction.kForward).alongWith(Commands.idle(this.rotationalSubsystem)).withName("SysID Dynamic Forward").asProxy());
        SmartDashboard.putData("SysID/Drive/Dynamic Reverse", routine.dynamic(Direction.kReverse).alongWith(Commands.idle(this.rotationalSubsystem)).withName("SysID Dynamic Reverse").asProxy());
    }

    private static final SwerveModuleState[] emptyStates = new SwerveModuleState[0];
    private static final ChassisSpeeds emptySpeeds = new ChassisSpeeds();

    @Override
    public void periodic() {
        OdometryThread.getInstance().odometryLock.lock();
        LoggedTracer.logEpoch("CommandScheduler Periodic/VirtualSubsystem Periodic/Drive/Acquire Odometry Lock");

        this.odometryTimestamps.timestamps = this.odometryTimestampQueue.stream().mapToDouble(Double::doubleValue).toArray();
        Logger.processInputs("Inputs/Drive/Timestamps", this.odometryTimestamps);
        LoggedTracer.logEpoch("CommandScheduler Periodic/VirtualSubsystem Periodic/Drive/Process Timestamp Inputs");

        this.gyroIO.updateInputs(this.gyroInputs);
        Logger.processInputs("Inputs/Drive/Gyro", this.gyroInputs);
        LoggedTracer.logEpoch("CommandScheduler Periodic/VirtualSubsystem Periodic/Drive/Process Gyro Inputs");

        for (var module : modules) {
            module.periodic();
        }
        LoggedTracer.logEpoch("CommandScheduler Periodic/VirtualSubsystem Periodic/Drive/Module Periodic");

        OdometryThread.getInstance().odometryLock.unlock();

        var sampleTimestamps = (RobotType.getMode() == Mode.SIM) ? (
            new double[] {Timer.getTimestamp()}
        ) : (
            this.odometryTimestamps.timestamps
        );
        for (int sampleI = 0; sampleI < sampleTimestamps.length; sampleI++) {
            var modulePositions = new SwerveModulePosition[this.modules.length];
            for (int i = 0; i < this.modules.length; i++) {
                modulePositions[i] = this.modules[i].getModulePositions()[sampleI];
            }

            if (this.lastMeasuredPositions != null) {
                RobotState.getInstance().addOdometryObservation(new OdometryObservation(
                    sampleTimestamps[sampleI],
                    (this.gyroInputs.connected) ? (
                        Optional.of(this.gyroInputs.odometryGyroRotation[sampleI])
                    ) : (
                        Optional.empty()
                    ),
                    this.lastMeasuredPositions,
                    modulePositions
                ));
            }
            this.lastMeasuredPositions = modulePositions;
        }

        this.measuredStates = Arrays.stream(this.modules).map(Module::getModuleState).toArray(SwerveModuleState[]::new);
        Logger.recordOutput("Drive/Swerve States/Measured", this.measuredStates);

        this.robotMeasuredSpeeds = DriveConstants.kinematics.toChassisSpeeds(this.measuredStates);
        if (this.gyroInputs.connected) {
            this.robotMeasuredSpeeds.omegaRadiansPerSecond = this.gyroInputs.yawVelocity.in(RadiansPerSecond);
        }

        Logger.recordOutput("Drive/Chassis Speeds/Measured", this.robotMeasuredSpeeds);
        // RobotState.getInstance().addDriveMeasurement(this.gyroAngle, this.getModulePositions());

        // this.fieldMeasuredSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(this.robotMeasuredSpeeds, this.gyroAngle);

        // Skid Detection
        // SwerveModuleState[] rotationalStates = new SwerveModuleState[DriveConstants.modules.length];
        // SwerveModuleState[] translationalStates = new SwerveModuleState[DriveConstants.modules.length];

        // for (int i = 0; i < DriveConstants.modules.length; i++) {
        //     var rotationalState = new SwerveModuleState(-gyroInputs.yawVelocity.in(RadiansPerSecond) * DriveConstants.modules[i].moduleTranslation.getNorm(), MathExtraUtil.rotationFromVector(DriveConstants.modules[i].positiveRotVec));
        //     var rotational = new Translation2d(rotationalState.speedMetersPerSecond, rotationalState.angle);
        //     rotationalStates[i] = rotationalState;
        //     var measured = new Translation2d(measuredStates[i].speedMetersPerSecond, measuredStates[i].angle);
        //     var translational = measured.minus(rotational);
        //     translationalStates[i] = new SwerveModuleState(translational.getNorm(), translational.getAngle());
        // }

        // Logger.recordOutput("Drive/SwerveStates/Rotational States", rotationalStates);
        // Logger.recordOutput("Drive/SwerveStates/Translational States", translationalStates);

        // var minTranslational = Arrays.stream(translationalStates).mapToDouble((state) -> state.speedMetersPerSecond).map(Math::abs).min().orElse(0);
        // var maxTranslational = Arrays.stream(translationalStates).mapToDouble((state) -> state.speedMetersPerSecond).map(Math::abs).max().orElse(0);
        // var averageTranslational = Arrays.stream(translationalStates).mapToDouble((state) -> state.speedMetersPerSecond).map(Math::abs).average().orElse(0);
        // var maxDistanceFromAverage = Arrays.stream(translationalStates).mapToDouble((state) -> state.speedMetersPerSecond).map(Math::abs).map((a) -> averageTranslational - a).map(Math::abs).average().orElse(0);
        // Logger.recordOutput("Drive/Skid Detection/Min Translational Speed", minTranslational);
        // Logger.recordOutput("Drive/Skid Detection/Max Translational Speed", maxTranslational);
        // Logger.recordOutput("Drive/Skid Detection/MaxMin Ratio", maxTranslational / minTranslational);
        // Logger.recordOutput("Drive/Skid Detection/Largest From Average", maxDistanceFromAverage);

        // Clearing log fields
        Logger.recordOutput("Drive/Chassis Speeds/Setpoint", emptySpeeds);
        Logger.recordOutput("Drive/Swerve States/Setpoints", emptyStates);
        Logger.recordOutput("Drive/Swerve States/Setpoints Optimized", emptyStates);

        LoggedTracer.logEpoch("CommandScheduler Periodic/VirtualSubsystem Periodic/Drive/Clear Log Fields");
        LoggedTracer.logEpoch("CommandScheduler Periodic/VirtualSubsystem Periodic/Drive");
    }

    public void postCommandPeriodic() {
        // if (DriverStation.isDisabled()) {
        //     // TODO: UNCOMMENT IF DRIVE MOVES WITH NO COMMAND AFTER ENABLE
        //     // for (var module : modules) {
        //     //     module.stop();
        //     // }
        // } else
        if (this.translationSubsystem.needsPostProcessing || this.rotationalSubsystem.needsPostProcessing) {
            this.runRobotSpeeds(this.desiredRobotSpeeds);
        }
        LoggedTracer.logEpoch("VirtualSubsystem PostCommandPeriodic/Drive");
    }

    public void runSetpoints(SwerveModuleState... states) {
        this.translationSubsystem.needsPostProcessing = false;
        this.rotationalSubsystem.needsPostProcessing = false;
        this.setpointStates = states;
        Logger.recordOutput("Drive/Swerve States/Setpoints", this.setpointStates);
        IntStream.range(0, this.modules.length).forEach((i) -> this.modules[i].runSetpoint(this.setpointStates[i]));
        Logger.recordOutput("Drive/Swerve States/Setpoints Optimized", this.setpointStates);
    }

    private static final LoggedTunableMeasure<LinearAccelerationUnit> forwardAccelLimitTunable = new LoggedTunableMeasure<>("Drive/Accel Limits/Forward Accel Limit", MetersPerSecondPerSecond.of(5000));
    private static final LoggedTunableMeasure<LinearAccelerationUnit> skidAccelLimitTunable = new LoggedTunableMeasure<>("Drive/Accel Limits/Skid Accel Limit", MetersPerSecondPerSecond.of(60));

    public static final Supplier<TiltAccelerationLimits> normalTiltLimitTunable = TiltAccelerationLimits.getTunable("Drive/Accel Limits/Tilt Limits/Normal", new TiltAccelerationLimits(500, 500, 500, 500));
    public static final Supplier<TiltAccelerationLimits> extendedTiltLimitTunable = TiltAccelerationLimits.getTunable("Drive/Accel Limits/Tilt Limits/Extended", new TiltAccelerationLimits(10, 12, 20, 20));
    private TiltAccelerationLimits tiltLimits = new TiltAccelerationLimits(10, 10, 10, 10);
    public void setTiltLimits(TiltAccelerationLimits tiltLimits) {
        this.tiltLimits = tiltLimits;
    }
    public void runRobotSpeeds(ChassisSpeeds robotSpeeds) {
        this.desiredRobotSpeeds = robotSpeeds;
        Logger.recordOutput("Drive/Chassis Speeds/Desired Speed", this.desiredRobotSpeeds);

        var desiredDelta = this.desiredRobotSpeeds.minus(this.robotMeasuredSpeeds);
        var desiredAccel = desiredDelta.div(RobotConstants.rioUpdatePeriodSecs);
        Logger.recordOutput("Drive/Chassis Speeds/Desired Delta", desiredDelta);
        Logger.recordOutput("Drive/Chassis Speeds/Desired Accel", desiredAccel);

        var limitedAccel = desiredAccel;

        // Forward Accel Limit
        var maxMeasuredModuleSpeed = Math.hypot(this.robotMeasuredSpeeds.vxMetersPerSecond, this.robotMeasuredSpeeds.vyMetersPerSecond) + Math.abs(this.robotMeasuredSpeeds.omegaRadiansPerSecond * DriveConstants.driveBaseRadius.in(Meters));
        var maxDesiredModuleAccel = Math.hypot(limitedAccel.vxMetersPerSecond, limitedAccel.vyMetersPerSecond) + Math.abs(limitedAccel.omegaRadiansPerSecond * DriveConstants.driveBaseRadius.in(Meters));
        var forwardAccelLimit = /* (1 - (maxMeasuredModuleSpeed / DriveConstants.maxModuleSpeed.in(MetersPerSecond))) *  */forwardAccelLimitTunable.get().in(MetersPerSecondPerSecond);
        var forwardAccelLimitingFactor = forwardAccelLimit / Math.max(maxDesiredModuleAccel, forwardAccelLimit);
        Logger.recordOutput("Drive/Chassis Speeds/Forward Limit/Max Measured Module Speed", maxMeasuredModuleSpeed);
        Logger.recordOutput("Drive/Chassis Speeds/Forward Limit/Max Desired Module Accel", maxDesiredModuleAccel);
        Logger.recordOutput("Drive/Chassis Speeds/Forward Limit/Forward Accel Limit", forwardAccelLimit);
        Logger.recordOutput("Drive/Chassis Speeds/Forward Limit/Limiting Factor", forwardAccelLimitingFactor);
        limitedAccel = limitedAccel.times(forwardAccelLimitingFactor);

        // Tilt Accel Limit
        var desiredTiltAccel = Math.hypot(limitedAccel.vxMetersPerSecond, limitedAccel.vyMetersPerSecond);
        var desiredTiltAccelHeading = Math.atan2(limitedAccel.vyMetersPerSecond, limitedAccel.vxMetersPerSecond);
        var tiltAccelLimit = this.tiltLimits.getMaxTiltAccelerationMPSS(desiredTiltAccelHeading);
        var tiltAccelLimitingFactor = tiltAccelLimit / Math.max(desiredTiltAccel, tiltAccelLimit);
        Logger.recordOutput("Drive/Chassis Speeds/Tilt Limit/Desired Tilt Accel", desiredTiltAccel);
        Logger.recordOutput("Drive/Chassis Speeds/Tilt Limit/Desired Tilt Accel Heading", desiredTiltAccelHeading);
        Logger.recordOutput("Drive/Chassis Speeds/Tilt Limit/Til tAccel Limit", tiltAccelLimit);
        Logger.recordOutput("Drive/Chassis Speeds/Tilt Limit/Limiting Factor", tiltAccelLimitingFactor);
        limitedAccel = new ChassisSpeeds(
            limitedAccel.vxMetersPerSecond * tiltAccelLimitingFactor,
            limitedAccel.vyMetersPerSecond * tiltAccelLimitingFactor,
            limitedAccel.omegaRadiansPerSecond
        );

        // Skid Accel Limit
        var desiredSkidAccel = Math.hypot(limitedAccel.vxMetersPerSecond, limitedAccel.vyMetersPerSecond);
        var skidAccelLimit = skidAccelLimitTunable.get().in(MetersPerSecondPerSecond);
        var skidAccelLimitingFactor = skidAccelLimit / Math.max(desiredSkidAccel, skidAccelLimit);
        Logger.recordOutput("Drive/Chassis Speeds/Skid Limit/Desired Skid Accel", desiredSkidAccel);
        Logger.recordOutput("Drive/Chassis Speeds/Skid Limit/Skid Accel Limit", skidAccelLimit);
        Logger.recordOutput("Drive/Chassis Speeds/Skid Limit/Limiting Factor", skidAccelLimitingFactor);
        limitedAccel = new ChassisSpeeds(
            limitedAccel.vxMetersPerSecond * skidAccelLimitingFactor,
            limitedAccel.vyMetersPerSecond * skidAccelLimitingFactor,
            limitedAccel.omegaRadiansPerSecond
        );


        var limitedDelta = limitedAccel.times(RobotConstants.rioUpdatePeriodSecs);
        var limitedSpeeds = this.robotMeasuredSpeeds.plus(limitedDelta);
        // Logger.recordOutput("Drive/Chassis Speeds/Limiting Factor", limitingFactor);
        Logger.recordOutput("Drive/Chassis Speeds/Limited Accel", limitedAccel);
        Logger.recordOutput("Drive/Chassis Speeds/Limited Delta", limitedDelta);
        Logger.recordOutput("Drive/Chassis Speeds/Limited Speed", limitedSpeeds);

        ChassisSpeeds correctedSpeeds = ChassisSpeeds.discretize(limitedSpeeds, rotationCorrection.get());
        this.setpointStates = DriveConstants.kinematics.toSwerveModuleStates(correctedSpeeds, this.centerOfRotation);
        SwerveDriveKinematics.desaturateWheelSpeeds(this.setpointStates, DriveConstants.maxDriveSpeed);
        this.runSetpoints(this.setpointStates);
    }
    public void runFieldSpeeds(ChassisSpeeds fieldSpeeds) {
        this.runRobotSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(fieldSpeeds, RobotState.getInstance().getEstimatedGlobalPose().getRotation()));
    }


    public Command coast() {
        return new Command() {
            {
                addRequirements(subsystems);
                setName("Coast");
            }
            @Override
            public void initialize() {
                Arrays.stream(modules).forEach((module) -> module.stopDrive(Optional.of(NeutralMode.Coast)));
            }
            @Override
            public void end(boolean interrupted) {
                Arrays.stream(modules).forEach((module) -> module.stopDrive(Optional.of(NeutralMode.Brake)));
            }
            @Override
            public boolean runsWhenDisabled() {
                return true;
            }
        };
    }

    public Command followBluePath(PathPlannerPath path) {
        return new FollowPathCommand(
            path,
            RobotState.getInstance()::getEstimatedGlobalPose,
            this::getRobotMeasuredSpeeds,
            this::drivePPVelocity,
            autoConfig(),
            DriveConstants.robotConfig,
            AllianceFlipUtil::shouldFlip,
            this.translationSubsystem, this.rotationalSubsystem
        );
    }
    public Command followExactPath(PathPlannerPath path) {
        return new FollowPathCommand(
            path,
            RobotState.getInstance()::getEstimatedGlobalPose,
            this::getRobotMeasuredSpeeds,
            this::drivePPVelocity,
            autoConfig(),
            DriveConstants.robotConfig,
            () -> false,
            this.translationSubsystem, this.rotationalSubsystem
        );
    }

    public void drivePPVelocity(ChassisSpeeds speeds, DriveFeedforwards ff) {
        //TODO: Actually do something with PP ff
        this.runRobotSpeeds(speeds);
    }

    public void setCenterOfRotation(Translation2d cor) {
        this.centerOfRotation = cor;
        Logger.recordOutput("Drive/Center of Rotation", RobotState.getInstance().getEstimatedGlobalPose().transformBy(new Transform2d(this.centerOfRotation, Rotation2d.kZero)));
    }

    /** Stops the drive. */
    public void stop() {
        Arrays.stream(this.modules).forEach((module) -> {
            module.stopDrive(Optional.empty());
            module.stopTurn(Optional.empty());
        });
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement.
     * The modules will
     * return to their normal orientations the next time a nonzero velocity is
     * requested.
     */
    public void stopWithX() {
        IntStream.range(0, DriveConstants.moduleConstants.length).forEach((i) -> {
            this.setpointStates[i] = new SwerveModuleState(
                0,
                DriveConstants.moduleConstants[i].moduleTranslation.getAngle()
            );
        });
    }

    /**
     * Returns the measured X, Y, and theta field velocities in meters per sec. The
     * components of the
     * twist are velocities and NOT changes in position.
     */
    public Twist2d getFieldVelocity() {
        return this.fieldVelocity;
    }

    /** Returns the current pitch velocity (Y rotation) in radians per second. */
    public AngularVelocity getYawVelocity() {
        return this.gyroInputs.yawVelocity;
    }

    /** Returns the current pitch velocity (Y rotation) in radians per second. */
    public AngularVelocity getPitchVelocity() {
        return this.gyroInputs.pitchVelocity;
    }

    /** Returns the current roll velocity (X rotation) in radians per second. */
    public AngularVelocity getRollVelocity() {
        return this.gyroInputs.rollVelocity;
    }

    /** Returns an array of module positions. */
    public SwerveModulePosition[] getModulePositions() {
        return this.lastMeasuredPositions;
    }

    /** Returns the average drive distance in radians */
    public double getAverageModuleDistance() {
        double avgDist = 0.0;
        for (int i = 0; i < DriveConstants.moduleConstants.length; i++) {
            avgDist += Math.abs(this.modules[i].getWheelAngularPosition().in(Radians));
        }
        return avgDist / DriveConstants.moduleConstants.length;
    }

    public ChassisSpeeds getRobotMeasuredSpeeds() {
        return this.robotMeasuredSpeeds;
    }

    public ChassisSpeeds getFieldMeasuredSpeeds() {
        return this.fieldMeasuredSpeeds;
    }

    /** Returns the average drive velocity in radians/sec. */
    public double getCharacterizationVelocity() {
        return Arrays.stream(this.modules).map(Module::getWheelAngularVelocity).mapToDouble(AngularVelocity::baseUnitMagnitude).average().orElse(0);
    }

    // public boolean collisionDetected() {
    //     return currentSpikeTimer.hasElapsed(currentSpikeTime.in(Seconds));
    // }

    private static final LoggedTunableNumber tP = new LoggedTunableNumber("AutoDrive/tP", 1);
    private static final LoggedTunableNumber tI = new LoggedTunableNumber("AutoDrive/tI", 0);
    private static final LoggedTunableNumber tD = new LoggedTunableNumber("AutoDrive/tD", 0);
    private static final LoggedTunableNumber rP = new LoggedTunableNumber("AutoDrive/rP", 1.5);
    private static final LoggedTunableNumber rI = new LoggedTunableNumber("AutoDrive/rI", 0);
    private static final LoggedTunableNumber rD = new LoggedTunableNumber("AutoDrive/rD", 0);
    public static PPHolonomicDriveController autoConfig() {
        return new PPHolonomicDriveController(
            new PIDConstants(
                tP.get(),
                tI.get(),
                tD.get()
            ),
            new PIDConstants(
                rP.get(),
                rI.get(),
                rD.get()
            )
        );
    }
    // public static RobotConfig robotConfig() {
    //     return new RobotConfig(
    //         RobotConstants.robotWeight,
    //         RobotConstants.robotMOI,
    //         new com.pathplanner.lib.config.ModuleConfig(
    //             DriveConstants.wheelRadius,
    //             DriveConstants.maxDriveSpeed,
    //             1.0,
    //             DCMotor.getFalcon500(1),
    //             Amps.of(55),
    //             1
    //         ),
    //         DriveConstants.trackWidthX,
    //         DriveConstants.trackWidthY
    //     );
    // }

    public final Command simplePIDTo(Supplier<Pose2d> target) {
        return Commands.parallel(this.translationSubsystem.simplePIDTo(() -> target.get().getTranslation()), this.rotationalSubsystem.pidControlledOptionalHeading(() -> Optional.of(target.get().getRotation())));
    }

    public final Translational translationSubsystem;
    public static class Translational extends SubsystemBase {
        public final Drive drive;
        private boolean needsPostProcessing = false;
        
        private Translational(Drive drive) {
            this.drive = drive;
            setName("Drive/Translational");
            SmartDashboard.putData("Subsystems/Drive/Translational", this);
        }

        public void driveVelocity(double vx, double vy) {
            this.needsPostProcessing = true;
            this.drive.desiredRobotSpeeds.vxMetersPerSecond = vx;
            this.drive.desiredRobotSpeeds.vyMetersPerSecond = vy;
        }
        public void driveVelocity(ChassisSpeeds speeds) {
            this.driveVelocity(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        }

        public void stop() {
            this.needsPostProcessing = false;
            this.driveVelocity(0,0);
        }

        public Command fieldRelative(Supplier<ChassisSpeeds> speeds) {
            var subsystem = this;
            return new Command() {
                {
                    addRequirements(subsystem);
                    setName("Field Relative");
                }
                @Override
                public void execute() {
                    driveVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds.get(), RobotState.getInstance().getEstimatedGlobalPose().getRotation()));
                }
                @Override
                public void end(boolean interrupted) {
                    stop();
                }
            };
        }
        
        public Command simplePIDTo(Supplier<Translation2d> target) {
            var subsystem = this;
            return new Command() {
                private static final LoggedTunableNumber driveKP = new LoggedTunableNumber("Drivetest/P", 3);
                {
                    addRequirements(subsystem);
                    setName("Simple PID To");
                }
                @Override
                public void execute() {
                    var targetPose = target.get();
                    var distTo = RobotState.getInstance().getEstimatedGlobalPose().getTranslation().getDistance(targetPose);
                    var vec = targetPose.minus(RobotState.getInstance().getEstimatedGlobalPose().getTranslation());
                    var norm = vec.div(vec.getNorm());
                    var pterm = distTo * driveKP.getAsDouble();
                    var out = norm.times(pterm);
                    driveVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                        new ChassisSpeeds(
                            out.getX(),
                            out.getY(),
                            0
                        ),
                        RobotState.getInstance().getEstimatedGlobalPose().getRotation()
                    ));
                }
                @Override
                public void end(boolean interrupted) {
                    stop();
                }
            };
        }
    }
    public final Rotational rotationalSubsystem;
    public static class Rotational extends SubsystemBase {
        public final Drive drive;
        private boolean needsPostProcessing = false;
        
        private Rotational(Drive drive) {
            this.drive = drive;
            setName("Drive/Rotational");
            SmartDashboard.putData("Subsystems/Drive/Rotational", this);
        }

        public void driveVelocity(double omega) {
            this.needsPostProcessing = true;
            this.drive.desiredRobotSpeeds.omegaRadiansPerSecond = omega;
        }
        public void driveVelocity(ChassisSpeeds speeds) {
            this.driveVelocity(speeds.omegaRadiansPerSecond);
        }
        public void stop() {
            this.needsPostProcessing = false;
            this.driveVelocity(0);
        }

        public Command spin(DoubleSupplier omega) {
            return Commands.runEnd(() -> driveVelocity(omega.getAsDouble()), this::stop, this);
        }

        public Command defenseSpin(Joystick joystick) {
            var subsystem = this;
            return new Command() {
                {
                    addRequirements(subsystem);
                    setName("Defense Spin");
                }
                private static final LoggedTunableNumber defenseSpinLinearThreshold = new LoggedTunableNumber("Drive/Defense Spin Linear Threshold", 0.125);
                private static final Matrix<N2, N2> perpendicularMatrix = 
                    MatBuilder.fill(
                        Nat.N2(), Nat.N2(), 
                        +0,-1,
                        +1,+0
                    )
                ;
                @Override
                public void execute() {
                    // Leds.getInstance().defenseSpin.setFlag(true);
                    var joyVec = Perspective.getCurrent().toField(joystick.toVector());
                    var desiredLinear = VecBuilder.fill(drive.desiredRobotSpeeds.vxMetersPerSecond, drive.desiredRobotSpeeds.vyMetersPerSecond);
                    var fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(drive.desiredRobotSpeeds, RobotState.getInstance().getEstimatedGlobalPose().getRotation());
                    var perpendicularLinear = new Vector<N2>(perpendicularMatrix.times(
                        VecBuilder.fill(fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond)
                    ));
                    var dot = joystick.x().getAsDouble();
                    if(desiredLinear.norm() > defenseSpinLinearThreshold.get()) {
                        dot = -joyVec.dot(perpendicularLinear);
                    }
                    var omega = dot
                        * DriveConstants.maxTurnRate.in(RadiansPerSecond)
                        * DriveConstants.maxTurnRateEnvCoef.getAsDouble() * 0.25
                    ;
                    driveVelocity(omega);
                    if(desiredLinear.norm() <= defenseSpinLinearThreshold.get()) {
                        drive.setCenterOfRotation(new Translation2d());
                        return;
                    }
                    var rotateAround = GeomUtil.vectorFromRotation(
                        GeomUtil.rotationFromVector(desiredLinear)
                        // .plus(Rotation2d.fromDegrees(45 * Math.signum(velo)))
                    );
                    drive.setCenterOfRotation(
                        Arrays.stream(DriveConstants.moduleTranslations)
                        .map((t) -> new Translation2d(t.toVector().unit().times(RobotConstants.centerToBumperCorner.in(Meters))))
                        .sorted((a, b) -> 
                            (int) Math.signum(
                                b.toVector().unit().dot(rotateAround) - a.toVector().unit().dot(rotateAround)
                            )    
                        )
                        .findFirst()
                        .orElse(new Translation2d())
                    );
                }
                @Override
                public void end(boolean interrupted) {
                    stop();
                    drive.setCenterOfRotation(new Translation2d());
                    // Leds.getInstance().defenseSpin.setFlag(false);
                }
            };
        }

        public Command pidControlledOptionalHeading(Supplier<Optional<Rotation2d>> headingSupplier) {
            var subsystem = this;
            return new Command() {
                private final ProfiledPIDController headingPID = new ProfiledPIDController(
                    DriveConstants.headingKp,
                    DriveConstants.headingKi,
                    DriveConstants.headingKd,
                    new Constraints(
                        DriveConstants.maxTurnRate.in(RadiansPerSecond),
                        5000
                    )
                );
                {
                    addRequirements(subsystem);
                    setName("PID Controlled Heading");
                    headingPID.enableContinuousInput(-Math.PI, Math.PI);
                    headingPID.setTolerance(DriveConstants.headingTolerance.in(Radians), DriveConstants.omegaTolerance.in(RadiansPerSecond));
                }
                private Rotation2d desiredHeading;
                private boolean headingSet;
                @Override
                public void initialize() {
                    desiredHeading = RobotState.getInstance().getEstimatedGlobalPose().getRotation();
                    headingPID.reset(RobotState.getInstance().getEstimatedGlobalPose().getRotation().getRadians());
                }
                @Override
                public void execute() {
                    var heading = headingSupplier.get();
                    headingSet = heading.isPresent();
                    heading.ifPresent((r) -> desiredHeading = r);
                    double turnInput = headingPID.calculate(RobotState.getInstance().getEstimatedGlobalPose().getRotation().getRadians(), desiredHeading.getRadians());
                    turnInput = headingPID.atSetpoint() ? 0 : turnInput + headingPID.getSetpoint().velocity;
                    turnInput = MathUtil.clamp(
                        turnInput, 
                        -0.5 * DriveConstants.maxTurnRateEnvCoef.getAsDouble(), 
                        +0.5 * DriveConstants.maxTurnRateEnvCoef.getAsDouble()
                    );
                    driveVelocity(turnInput * DriveConstants.maxTurnRate.in(RadiansPerSecond));
                }
                @Override
                public void end(boolean interrupted) {
                    stop();
                }
                @Override
                public boolean isFinished() {
                    return !headingSet && headingPID.atSetpoint();
                }
            };
        }
        public Command pidControlledHeading(Supplier<Rotation2d> headingSupplier) {
            var subsystem = this;
            return new Command() {
                private final ProfiledPIDController headingPID = new ProfiledPIDController(
                    DriveConstants.headingKp,
                    DriveConstants.headingKi,
                    DriveConstants.headingKd,
                    new Constraints(
                        DriveConstants.maxTurnRate.in(RadiansPerSecond),
                        5000
                    )
                );
                {
                    addRequirements(subsystem);
                    setName("PID Controlled Heading");
                    headingPID.enableContinuousInput(-Math.PI, Math.PI);
                    headingPID.setTolerance(DriveConstants.headingTolerance.in(Radians), DriveConstants.omegaTolerance.in(RadiansPerSecond));
                }
                @Override
                public void initialize() {
                    headingPID.reset(RobotState.getInstance().getEstimatedGlobalPose().getRotation().getRadians());
                }
                @Override
                public void execute() {
                    var desiredHeading = headingSupplier.get();
                    double turnInput = headingPID.calculate(RobotState.getInstance().getEstimatedGlobalPose().getRotation().getRadians(), desiredHeading.getRadians());
                    turnInput = headingPID.atSetpoint() ? 0 : turnInput + headingPID.getSetpoint().velocity;
                    turnInput = MathUtil.clamp(
                        turnInput, 
                        -0.5 * DriveConstants.maxTurnRateEnvCoef.getAsDouble(), 
                        +0.5 * DriveConstants.maxTurnRateEnvCoef.getAsDouble()
                    );
                    driveVelocity(turnInput * DriveConstants.maxTurnRate.in(RadiansPerSecond));
                }
                @Override
                public void end(boolean interrupted) {
                    stop();
                }
                @Override
                public boolean isFinished() {
                    return false;
                }
            };
        }

        public Command headingFromJoystick(Joystick joystick, Rotation2d[] snapPoints, Supplier<Rotation2d> forwardDirectionSupplier) {
            return pidControlledOptionalHeading(
                new LazyOptional<Rotation2d>() {
                    private final Timer preciseTurnTimer = new Timer();
                    private final double preciseTurnTimeThreshold = 0.5;
                    private Optional<Rotation2d> outputMap(Rotation2d i) {
                        return Optional.of(i.minus(forwardDirectionSupplier.get()));
                    }
                    @Override
                    public Optional<Rotation2d> get() {
                        if(joystick.magnitude() == 0) {
                            preciseTurnTimer.restart();
                            return Optional.empty();
                        }
                        var joyHeading = GeomUtil.rotationFromVector(Perspective.getCurrent().toField(joystick.toVector()));
                        if(preciseTurnTimer.hasElapsed(preciseTurnTimeThreshold)) {
                            return outputMap(joyHeading);
                        }
                        int smallestDistanceIndex = 0;
                        double smallestDistance = Double.MAX_VALUE;
                        for(int i = 0; i < snapPoints.length; i++) {
                            var dist = Math.abs(joyHeading.minus(AllianceFlipUtil.apply(snapPoints[i])).getRadians());
                            if(dist < smallestDistance) {
                                smallestDistance = dist;
                                smallestDistanceIndex = i;
                            }
                        }
                        return outputMap(AllianceFlipUtil.apply(snapPoints[smallestDistanceIndex]));
                    }
                }
            );
        }

        public Command pointTo(Supplier<Optional<Translation2d>> posToPointTo, Supplier<Rotation2d> forward) {
            return pidControlledOptionalHeading(
                () -> posToPointTo.get().map((pointTo) -> {
                    var FORR = pointTo.minus(RobotState.getInstance().getEstimatedGlobalPose().getTranslation());
                    return new Rotation2d(FORR.getX(), FORR.getY()).minus(forward.get());
                })
            );
        }
    }
}
