// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.subsystems.drive.DriveConstants.ModuleConstants;
import frc.util.LoggedTracer;
import frc.util.NeutralMode;
import frc.util.faults.DeviceFaultAlerts;
import frc.util.faults.DeviceFaultClearer;
import frc.util.faults.DeviceFaults.FaultType;
import frc.util.loggerUtil.tunables.LoggedTunableFF;
import frc.util.loggerUtil.tunables.LoggedTunableMeasure;
import frc.util.loggerUtil.tunables.LoggedTunablePID;

public class Module {
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    public final ModuleConstants config;

    // private static final LoggedTunableMeasure<DistanceUnit> wheelRadius = new LoggedTunableMeasure<>("Drive/Module/WheelRadius", DriveConstants.wheelRadius, Inches);
    
    private final MutAngle wheelAngularPosition = Radians.mutable(0);
    private final MutAngularVelocity wheelAngularVelocity = RadiansPerSecond.mutable(0);
    private final MutDistance wheelLinearPosition = Meters.mutable(0);
    private final MutLinearVelocity wheelLinearVelocity = MetersPerSecond.mutable(0);
    private final SwerveModuleState moduleState = new SwerveModuleState();
    private final SwerveModulePosition modulePosition = new SwerveModulePosition();
    private SwerveModulePosition[] modulePositions = new SwerveModulePosition[0];

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
        LoggedTracer.logEpoch("CommandScheduler Periodic/VirtualSubsystem Periodic/Drive/Module Periodic/" + this.config.name + "/Before");
        this.io.updateInputs(this.inputs);
        LoggedTracer.logEpoch("CommandScheduler Periodic/VirtualSubsystem Periodic/Drive/Module Periodic/" + this.config.name + "/Update Inputs");
        Logger.processInputs("Inputs/Drive/Module " + this.config.name, this.inputs);
        LoggedTracer.logEpoch("CommandScheduler Periodic/VirtualSubsystem Periodic/Drive/Module Periodic/" + this.config.name + "/Process Inputs");

        this.modulePositions = new SwerveModulePosition[this.inputs.odometryDriveRads.length];
        for (int i = 0; i < this.inputs.odometryDriveRads.length; i++) {
            var angle = this.config.moduleForwardDirection.plus(Rotation2d.fromRadians(
                DriveConstants.azimuthEncoderToCarriageRatio.applyUnsigned(this.inputs.odometryAzimuthRads[i])
            ));
            var distanceMeters = DriveConstants.wheel.radiansToMeters(DriveConstants.driveMotorToWheelRatio.applyUnsigned(this.inputs.odometryDriveRads[i]));
            this.modulePositions[i] = new SwerveModulePosition(distanceMeters, angle);
        }

        var angle = this.config.moduleForwardDirection.plus(
            Rotation2d.fromRadians(
                DriveConstants.azimuthEncoderToCarriageRatio.applyUnsigned(this.inputs.azimuthEncoder.getPositionRads())
            )
        );
        this.moduleState.angle = angle;
        this.modulePosition.angle = angle;

        this.wheelAngularPosition.mut_replace(DriveConstants.driveMotorToWheelRatio.applyUnsigned(this.inputs.driveMotor.encoder.getPositionRads()), Radians);
        this.wheelAngularVelocity.mut_replace(DriveConstants.driveMotorToWheelRatio.applyUnsigned(this.inputs.driveMotor.encoder.getVelocityRadsPerSec()), RadiansPerSecond);
        this.wheelLinearPosition.mut_replace(DriveConstants.wheel.angleToDistance(this.wheelAngularPosition));
        this.wheelLinearVelocity.mut_replace(DriveConstants.wheel.angularVelocityToLinearVelocity(this.wheelAngularVelocity));

        this.modulePosition.distanceMeters = wheelLinearPosition.in(Meters);
        this.moduleState.speedMetersPerSecond = this.wheelLinearVelocity.in(MetersPerSecond);

        if (driveFFConsts.hasChanged(hashCode())) {
            driveFFConsts.update(this.driveFeedforward);
        }
        if (drivePIDConsts.hasChanged(hashCode())) {
            this.io.configDrivePID(drivePIDConsts.getConstants());
        }
        if (azimuthPIDConsts.hasChanged(hashCode())) {
            this.io.configAzimuthPID(azimuthPIDConsts.getConstants());
        }

        // this.driveMotorActiveFaultsAlert.updateFrom(this.inputs.driveMotorFaults.activeFaults);
        // this.driveMotorStickyFaultsAlert.updateFrom(this.inputs.driveMotorFaults.stickyFaults);
        // this.azimuthMotorActiveFaultsAlert.updateFrom(this.inputs.azimuthMotorFaults.activeFaults);
        // this.azimuthMotorStickyFaultsAlert.updateFrom(this.inputs.azimuthMotorFaults.stickyFaults);
        // this.driveMotorStickyFaultClearer.clear(this.inputs.driveMotorFaults.stickyFaults, this.io::clearDriveStickyFaults, DeviceFaults.allMask);
        // this.azimuthMotorStickyFaultClearer.clear(this.inputs.azimuthMotorFaults.stickyFaults, this.io::clearAzimuthStickyFaults, DeviceFaults.allMask);

        LoggedTracer.logEpoch("CommandScheduler Periodic/VirtualSubsystem Periodic/Drive/Module Periodic/" + this.config.name + "/Periodic");
        LoggedTracer.logEpoch("CommandScheduler Periodic/VirtualSubsystem Periodic/Drive/Module Periodic/" + this.config.name);
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

        var velocityRadPerSec = DriveConstants.driveMotorToWheelRatio.inverse().applyUnsigned(DriveConstants.wheel.metersToRadians(setpoint.speedMetersPerSecond));

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
        return this.moduleState.angle;
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
    public double getAppliedVolts() {
        return this.inputs.driveMotor.motor.getAppliedVolts();
    }

    public double getDriveStatorCurrentAmps() {
        return this.inputs.driveMotor.motor.getStatorCurrentAmps();
    }

    public SwerveModulePosition[] getModulePositions() {
        return this.modulePositions;
    }

    /** Returns the module state (turn angle and drive velocity). */
    public SwerveModuleState getModuleState() {
        return this.moduleState;
    }
}
