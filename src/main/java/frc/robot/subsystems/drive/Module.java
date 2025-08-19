// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.subsystems.drive.DriveConstants.ModuleConstants;
import frc.util.CurrentSpikeDetector;
import frc.util.LoggedTracer;
import frc.util.NeutralMode;
import frc.util.faults.DeviceFaultAlerts;
import frc.util.faults.DeviceFaultClearer;
import frc.util.faults.DeviceFaults;
import frc.util.faults.DeviceFaults.FaultType;
import frc.util.loggerUtil.tunables.LoggedTunableFF;
import frc.util.loggerUtil.tunables.LoggedTunableMeasure;
import frc.util.loggerUtil.tunables.LoggedTunablePID;

public class Module {
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    public final ModuleConstants config;

    // private static final LoggedTunableMeasure<DistanceUnit> wheelRadius = new LoggedTunableMeasure<>("Drive/Module/WheelRadius", DriveConstants.wheelRadius, Inches);
    
    private Rotation2d angle = Rotation2d.kZero;
    private final MutAngle wheelAngularPosition = Radians.mutable(0);
    private final MutAngularVelocity wheelAngularVelocity = RadiansPerSecond.mutable(0);
    private final MutDistance wheelLinearPosition = Meters.mutable(0);
    private final MutLinearVelocity wheelLinearVelocity = MetersPerSecond.mutable(0);
    private final SwerveModuleState moduleState = new SwerveModuleState();
    private final SwerveModulePosition modulePosition = new SwerveModulePosition();
    private final SwerveModulePosition prevModulePosition = new SwerveModulePosition();

    private static final LoggedTunableMeasure<CurrentUnit> currentSpikeThreshold = new LoggedTunableMeasure<>("Drive/Current Spike Threshold", Amps.of(0)); 
    private static final LoggedTunableMeasure<TimeUnit> currentSpikeTime = new LoggedTunableMeasure<>("Drive/Current Spike Time", Seconds.of(0));
    private final CurrentSpikeDetector driveCurrentSpikeDetector = new CurrentSpikeDetector(currentSpikeThreshold, currentSpikeTime);
    
    private static final LoggedTunableMeasure<LinearVelocityUnit> brakeModeThreshold = new LoggedTunableMeasure<>("Drive/Brake Mode Threshold", InchesPerSecond.of(1)); 
    
    private static final LoggedTunablePID drivePIDConsts = new LoggedTunablePID(
        "Drive/Module/Drive/PID",
        0.1,
        0,
        0
    );
    private static final LoggedTunableFF driveFFConsts = new LoggedTunableFF(
        "Drive/Module/Drive/FF",
        0,
        0,
        2.2,
        0
    );
    private static final LoggedTunablePID azimuthPIDConsts = new LoggedTunablePID(
        "Drive/Module/Azimuth/PID",
        5*2*Math.PI,
        0*2*Math.PI,
        0*2*Math.PI
    );

    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0,0,0);

    private final DeviceFaultAlerts driveMotorActiveFaultsAlert;
    private final DeviceFaultAlerts driveMotorStickyFaultsAlert;
    private final DeviceFaultAlerts azimuthMotorActiveFaultsAlert;
    private final DeviceFaultAlerts azimuthMotorStickyFaultsAlert;
    private final DeviceFaultClearer driveMotorStickyFaultClearer;
    private final DeviceFaultClearer azimuthMotorStickyFaultClearer;

    public Module(ModuleIO io, ModuleConstants config) {
        this.io = io;
        this.config = config;

        driveFFConsts.update(this.driveFeedforward);
        this.io.configDrivePID(drivePIDConsts.getConstants());
        this.io.configAzimuthPID(azimuthPIDConsts.getConstants());

        this.driveMotorActiveFaultsAlert = new DeviceFaultAlerts(new Alert("Drive/Module " + this.config.name + "/Alerts", "Drive Motor has active faults: ", AlertType.kError));
        this.driveMotorStickyFaultsAlert = new DeviceFaultAlerts(new Alert("Drive/Module " + this.config.name + "/Alerts", "Drive Motor has sticky faults: ", AlertType.kWarning), FaultType.StatorCurrentLimit, FaultType.SupplyCurrentLimit);
        this.azimuthMotorActiveFaultsAlert = new DeviceFaultAlerts(new Alert("Drive/Module " + this.config.name + "/Alerts", "Azimuth Motor has active faults: ", AlertType.kError));
        this.azimuthMotorStickyFaultsAlert = new DeviceFaultAlerts(new Alert("Drive/Module " + this.config.name + "/Alerts", "Azimuth Motor has sticky faults: ", AlertType.kWarning), FaultType.StatorCurrentLimit, FaultType.SupplyCurrentLimit);
        this.driveMotorStickyFaultClearer = new DeviceFaultClearer("Drive/Module " + this.config.name + "/Drive Motor Sticky Faults");
        this.azimuthMotorStickyFaultClearer = new DeviceFaultClearer("Drive/Module " + this.config.name + "/Azimuth Motor Sticky Faults");
    }

    /** Updates inputs and checks tunable numbers. */
    public void periodic() {
        this.prevModulePosition.distanceMeters = this.modulePosition.distanceMeters;
        this.prevModulePosition.angle = this.modulePosition.angle;

        this.io.updateInputs(this.inputs);
        Logger.processInputs("Inputs/Drive/Module " + this.config.name, this.inputs);
        LoggedTracer.logEpoch("CommandScheduler Periodic/VirtualSubsystem Periodic/Drive/Module Periodic/" + config.name + "/Process Inputs");

        this.angle = this.config.moduleForwardDirection.plus(new Rotation2d(this.inputs.azimuthEncoder.position));
        this.moduleState.angle = this.angle;
        this.modulePosition.angle = this.angle;

        this.wheelAngularPosition.mut_replace(DriveConstants.driveRatio.applyUnsigned(this.inputs.driveMotor.encoder.position));
        this.wheelAngularVelocity.mut_replace(DriveConstants.driveRatio.applyUnsigned(this.inputs.driveMotor.encoder.velocity));
        this.wheelLinearPosition.mut_replace(DriveConstants.wheel.angleToDistance(this.wheelAngularPosition));
        this.wheelLinearVelocity.mut_replace(DriveConstants.wheel.angularVelocityToLinearVelocity(this.wheelAngularVelocity));

        this.modulePosition.distanceMeters = wheelLinearPosition.in(Meters);
        this.moduleState.speedMetersPerSecond = wheelLinearVelocity.in(MetersPerSecond);

        this.driveCurrentSpikeDetector.update(this.getDriveCurrent());

        if (driveFFConsts.hasChanged(hashCode())) {
            driveFFConsts.update(this.driveFeedforward);
        }
        if (drivePIDConsts.hasChanged(hashCode())) {
            this.io.configDrivePID(drivePIDConsts.getConstants());
        }
        if (azimuthPIDConsts.hasChanged(hashCode())) {
            this.io.configAzimuthPID(azimuthPIDConsts.getConstants());
        }

        this.driveMotorActiveFaultsAlert.updateFrom(this.inputs.driveMotorFaults.activeFaults);
        this.driveMotorStickyFaultsAlert.updateFrom(this.inputs.driveMotorFaults.stickyFaults);
        this.azimuthMotorActiveFaultsAlert.updateFrom(this.inputs.azimuthMotorFaults.activeFaults);
        this.azimuthMotorStickyFaultsAlert.updateFrom(this.inputs.azimuthMotorFaults.stickyFaults);
        this.driveMotorStickyFaultClearer.clear(this.inputs.driveMotorFaults.stickyFaults, this.io::clearDriveStickyFaults, DeviceFaults.allMask);
        this.azimuthMotorStickyFaultClearer.clear(this.inputs.azimuthMotorFaults.stickyFaults, this.io::clearAzimuthStickyFaults, DeviceFaults.allMask);
    }

    /**
     * Runs the module with the specified setpoint state. Must be called
     * periodically.
     */
    public void runSetpoint(SwerveModuleState setpoint) {
        setpoint.optimize(this.getAngle());
        
        var turnSetpoint = setpoint.angle;
        this.io.setAzimuthAngle(turnSetpoint.minus(this.config.moduleForwardDirection).getMeasure());

        setpoint.speedMetersPerSecond *= turnSetpoint.minus(this.getAngle()).getCos();

        var velocityRadPerSec = DriveConstants.driveRatio.inverse().applyUnsigned(DriveConstants.wheel.rawLinearToAngular(setpoint.speedMetersPerSecond));

        var ffout = this.driveFeedforward.calculateWithVelocities(this.wheelLinearVelocity.in(MetersPerSecond), setpoint.speedMetersPerSecond);

        var belowBrakeModeThreshold = Math.abs(setpoint.speedMetersPerSecond) < brakeModeThreshold.get().in(MetersPerSecond);

        this.io.setDriveVelocity(RadiansPerSecond.of(velocityRadPerSec), RadiansPerSecondPerSecond.zero(), Volts.of(ffout), belowBrakeModeThreshold);
    }

    /**
     * Runs the module with the specified voltage
     * Must be called periodically.
     */
    public void runVoltage(Measure<VoltageUnit> volts, Rotation2d moduleAngle) {
        this.io.setAzimuthAngle(moduleAngle.minus(this.config.moduleForwardDirection).getMeasure());
        this.io.setDriveVoltage(volts);
    }

    public void stopDrive(Optional<NeutralMode> neutralMode) {
        this.io.stopDrive(neutralMode);
    }
    public void stopTurn(Optional<NeutralMode> neutralMode) {
        this.io.stopAzimuth(neutralMode);
    }

    /** Returns the current turn angle of the module. */
    public Rotation2d getAngle() {
        return this.angle;
    }

    /** Returns the current drive position of the module in radians. */
    public Angle getWheelAngularPosition() {
        return this.wheelAngularPosition;
    }
    /** Returns the drive velocity in radians/sec. */
    public AngularVelocity getWheelAngularVelocity() {
        return this.wheelAngularVelocity;
    }
    /** Returns the current drive position of the module in radians. */
    public Distance getWheelLinearPosition() {
        return this.wheelLinearPosition;
    }
    /** Returns the drive velocity in radians/sec. */
    public LinearVelocity getWheelLinearVelocity() {
        return this.wheelLinearVelocity;
    }

    /** Returns the drive velocity in radians/sec. */
    public Voltage getAppliedVoltage() {
        return this.inputs.driveMotor.motor.appliedVoltage;
    }

    public Current getDriveCurrent() {
        return this.inputs.driveMotor.motor.statorCurrent;
    }

    public boolean currentSpiking() {
        return this.driveCurrentSpikeDetector.hasSpike();
    }

    /** Returns the module position (turn angle and drive position). */
    public SwerveModulePosition getModulePosition() {
        return this.modulePosition;
    }

    /** Returns the module state (turn angle and drive velocity). */
    public SwerveModuleState getModuleState() {
        return this.moduleState;
    }

    /** Returns change in module position since last tick */
    public SwerveModulePosition getModulePositionDelta() {
        return new SwerveModulePosition(
            this.modulePosition.distanceMeters - this.prevModulePosition.distanceMeters,
            this.angle
        );
    }
}
