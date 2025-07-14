// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.CurrentUnit;
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
import frc.robot.subsystems.drive.DriveConstants.ModuleConstants;
import frc.util.CurrentSpikeDetector;
import frc.util.loggerUtil.tunables.LoggedTunableMeasure;

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

    public Module(ModuleIO io, ModuleConstants config) {
        this.io = io;
        this.config = config;
    }

    /** Updates inputs and checks tunable numbers. */
    public void periodic() {
        prevModulePosition.distanceMeters = modulePosition.distanceMeters;
        prevModulePosition.angle = modulePosition.angle;

        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Drive/Module " + config.name, inputs);

        angle = config.moduleForwardDirection.plus(new Rotation2d(inputs.turnMotor.encoder.position));
        moduleState.angle = angle;
        modulePosition.angle = angle;

        wheelAngularPosition.mut_replace(DriveConstants.driveGearRatio.applyUnsigned(inputs.driveMotor.encoder.position));
        wheelAngularVelocity.mut_replace(DriveConstants.driveGearRatio.applyUnsigned(inputs.driveMotor.encoder.velocity));
        wheelLinearPosition.mut_replace(DriveConstants.wheel.angleToDistance(wheelAngularPosition));
        wheelLinearVelocity.mut_replace(DriveConstants.wheel.angularVelocityToLinearVelocity(wheelAngularVelocity));

        modulePosition.distanceMeters = wheelLinearPosition.in(Meters);
        moduleState.speedMetersPerSecond = wheelLinearVelocity.in(MetersPerSecond);

        driveCurrentSpikeDetector.update(getDriveCurrent());
    }

    /**
     * Runs the module with the specified setpoint state. Must be called
     * periodically.
     */
    public void runSetpoint(SwerveModuleState setpoint) {
        setpoint.optimize(getAngle());
        
        var turnSetpoint = setpoint.angle;
        io.setTurnAngle(turnSetpoint.minus(config.moduleForwardDirection).getMeasure());

        setpoint.speedMetersPerSecond *= turnSetpoint.minus(getAngle()).getCos();

        double velocityRadPerSec = DriveConstants.driveGearRatio.inverse().applyUnsigned(DriveConstants.wheel.rawLinearToAngular(setpoint.speedMetersPerSecond));
        io.setDriveVelocity(RadiansPerSecond.of(velocityRadPerSec));
    }

    /**
     * Runs the module with the specified voltage while controlling to zero degrees.
     * Must be called periodically.
     */
    public void runVoltage(Measure<VoltageUnit> volts, Rotation2d moduleAngle) {
        io.setTurnAngle(moduleAngle.minus(config.moduleForwardDirection).getMeasure());
        io.setDriveVoltage(volts);
    }

    /** Disables all outputs to motors. */
    public void stop() {
        io.stop();
    }

    /** Sets whether brake mode is enabled. */
    public void setBrakeMode(boolean enabled) {
        io.setDriveBrakeMode(enabled);
        io.setTurnBrakeMode(enabled);
    }

    /** Returns the current turn angle of the module. */
    public Rotation2d getAngle() {
        return angle;
    }

    /** Returns the current drive position of the module in radians. */
    public Angle getWheelAngularPosition() {
        return wheelAngularPosition;
    }
    /** Returns the drive velocity in radians/sec. */
    public AngularVelocity getWheelAngularVelocity() {
        return wheelAngularVelocity;
    }
    /** Returns the current drive position of the module in radians. */
    public Distance getWheelLinearPosition() {
        return wheelLinearPosition;
    }
    /** Returns the drive velocity in radians/sec. */
    public LinearVelocity getWheelLinearVelocity() {
        return wheelLinearVelocity;
    }

    /** Returns the drive velocity in radians/sec. */
    public Voltage getAppliedVoltage() {
        return inputs.driveMotor.motor.appliedVoltage;
    }

    public Current getDriveCurrent() {
        return inputs.driveMotor.motor.current;
    }

    public boolean currentSpiking() {
        return driveCurrentSpikeDetector.hasSpike();
    }

    /** Returns the module position (turn angle and drive position). */
    public SwerveModulePosition getModulePosition() {
        return modulePosition;
    }

    /** Returns the module state (turn angle and drive velocity). */
    public SwerveModuleState getModuleState() {
        return moduleState;
    }

    /** Returns change in module position since last tick */
    public SwerveModulePosition getModulePositionDelta() {
        return new SwerveModulePosition(
            modulePosition.distanceMeters - prevModulePosition.distanceMeters,
            angle
        );
    }
    
    /** Zeros module encoders. */
    // public void zeroEncoders() {
    //     io.zeroEncoders();
    //     // need to also reset prevModulePosition because drive is driven by deltas in
    //     // position
    //     prevModulePosition = getPosition();
    // }
}
