package frc.robot.subsystems.superstructure.pivot;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.constants.RobotConstants;
import frc.util.FFConstants;
import frc.util.LoggedTracer;
import frc.util.NeutralMode;
import frc.util.PIDConstants;
import frc.util.faults.DeviceFaultAlerts;
import frc.util.faults.DeviceFaultClearer;
import frc.util.faults.DeviceFaults.FaultType;
import frc.util.loggerUtil.tunables.LoggedTunable;
import frc.util.robotStructure.angle.ArmMech;

public class Pivot {
    private final PivotIO io;
    private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

    private static final LoggedTunable<TrapezoidProfile.Constraints> profileConsts = LoggedTunable.fromDashboardUnits(
        "Superstructure/Pivot/Profile",
        DegreesPerSecond,
        DegreesPerSecondPerSecond,
        RadiansPerSecond,
        RadiansPerSecondPerSecond,
        new TrapezoidProfile.Constraints(
            225,
            450
        )
    );
    private static final LoggedTunable<FFConstants> ffConsts = LoggedTunable.from(
        "Superstructure/Pivot/FF",
        new FFConstants(
            0,
            0,
            17 /2/Math.PI,
            0
        )
    );
    private static final LoggedTunable<PIDConstants> pidConsts = LoggedTunable.from(
        "Superstructure/Pivot/PID",
        new PIDConstants(
            150,
            0,
            0
        )
    );

    private TrapezoidProfile motionProfile = new TrapezoidProfile(profileConsts.get());
    private final State measuredState = new State();
    private final State setpointState = new State();
    private final State goalState = new State();
    private boolean motionProfiling = false;
    private final ArmFeedforward feedforward = new ArmFeedforward(0,0,0,0);

    private double angleRads = 0.0;
    private double velocityRadsPerSec = 0.0;

    public final ArmMech mech = new ArmMech(PivotConstants.pivotBase);

    private final DeviceFaultAlerts leftMotorActiveFaultsAlert = new DeviceFaultAlerts(new Alert("Superstructure/Pivot/Alerts", "Left Motor has active faults: ", AlertType.kError));
    private final DeviceFaultAlerts leftMotorStickyFaultsAlert = new DeviceFaultAlerts(new Alert("Superstructure/Pivot/Alerts", "Left Motor has sticky faults: ", AlertType.kWarning), FaultType.ForwardSoftLimit, FaultType.ReverseSoftLimit, FaultType.StatorCurrentLimit, FaultType.SupplyCurrentLimit);
    private final DeviceFaultAlerts rightMotorActiveFaultsAlert = new DeviceFaultAlerts(new Alert("Superstructure/Pivot/Alerts", "Right Motor has active faults: ", AlertType.kError));
    private final DeviceFaultAlerts rightMotorStickyFaultsAlert = new DeviceFaultAlerts(new Alert("Superstructure/Pivot/Alerts", "Right Motor has sticky faults: ", AlertType.kWarning), FaultType.ForwardSoftLimit, FaultType.ReverseSoftLimit, FaultType.StatorCurrentLimit, FaultType.SupplyCurrentLimit);
    private final DeviceFaultAlerts encoderActiveFaultsAlert = new DeviceFaultAlerts(new Alert("Superstructure/Pivot/Alerts", "Encoder has active faults: ", AlertType.kError));
    private final DeviceFaultAlerts encoderStickyFaultsAlert = new DeviceFaultAlerts(new Alert("Superstructure/Pivot/Alerts", "Encoder has sticky faults: ", AlertType.kWarning));
    private final DeviceFaultClearer leftMotorStickyFaultClearer = new DeviceFaultClearer("Superstructure/Pivot/Left Motor Sticky Faults");
    private final DeviceFaultClearer rightMotorStickyFaultClearer = new DeviceFaultClearer("Superstructure/Pivot/Right Motor Sticky Faults");
    private final DeviceFaultClearer encoderStickyFaultClearer = new DeviceFaultClearer("Superstructure/Pivot/Encoder Sticky Faults");

    public Pivot(PivotIO io) {
        System.out.println("[Init Pivot] Instantiating Pivot with " + io.getClass().getSimpleName());
        this.io = io;

        ffConsts.get().update(this.feedforward);
        this.io.configPID(pidConsts.get());
    }

    public void periodic() {
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Superstructure/Pivot/Before");
        this.io.updateInputs(this.inputs);
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Superstructure/Pivot/Update Inputs");
        Logger.processInputs("Inputs/Superstructure/Pivot", this.inputs);
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Superstructure/Pivot/Process Inputs");

        this.angleRads = PivotConstants.sensorToMechanism.applyUnsigned(this.inputs.encoder.getPositionRads());
        this.velocityRadsPerSec = PivotConstants.sensorToMechanism.applyUnsigned(this.inputs.encoder.getVelocityRadsPerSec());

        this.measuredState.position = this.getAngleRads();
        this.measuredState.velocity = this.getVelocityRadsPerSec();

        this.mech.setRads(this.getAngleRads());

        Logger.recordOutput("Superstructure/Pivot/Angle/Measured", this.getAngleRads());
        Logger.recordOutput("Superstructure/Pivot/Velocity/Measured", this.getVelocityRadsPerSec());

        if (profileConsts.hasChanged(hashCode())) {
            this.motionProfile = new TrapezoidProfile(profileConsts.get());
        }
        if (ffConsts.hasChanged(hashCode())) {
            ffConsts.get().update(this.feedforward);
        }
        if (pidConsts.hasChanged(hashCode())) {
            this.io.configPID(pidConsts.get());
        }

        // this.leftMotorActiveFaultsAlert.updateFrom(this.inputs.leftMotorFaults.activeFaults);
        // this.leftMotorStickyFaultsAlert.updateFrom(this.inputs.leftMotorFaults.stickyFaults);
        // this.rightMotorActiveFaultsAlert.updateFrom(this.inputs.rightMotorFaults.activeFaults);
        // this.rightMotorStickyFaultsAlert.updateFrom(this.inputs.rightMotorFaults.stickyFaults);
        // this.encoderActiveFaultsAlert.updateFrom(this.inputs.encoderFaults.activeFaults);
        // this.encoderStickyFaultsAlert.updateFrom(this.inputs.encoderFaults.stickyFaults);
        // this.leftMotorStickyFaultClearer.clear(this.inputs.leftMotorFaults.stickyFaults, this.io::clearLeftMotorStickyFaults, DeviceFaults.allMask);
        // this.rightMotorStickyFaultClearer.clear(this.inputs.rightMotorFaults.stickyFaults, this.io::clearRightMotorStickyFaults, DeviceFaults.allMask);
        // this.encoderStickyFaultClearer.clear(this.inputs.encoderFaults.stickyFaults, this.io::clearEncoderStickyFaults, DeviceFaults.allMask);
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Superstructure/Pivot/Periodic");
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Superstructure/Pivot");
    }

    public double getAngleRads() {
        return this.angleRads;
    }
    public double getVelocityRadsPerSec() {
        return this.velocityRadsPerSec;
    }
    public double getAppliedVolts() {
        return this.inputs.leftMotor.motor.getAppliedVolts();
    }

    public void setVolts(double volts) {
        this.motionProfiling = false;
        this.io.setVolts(volts);
    }
    public void stop(Optional<NeutralMode> neutralMode) {
        this.motionProfiling = false;
        this.io.stop(neutralMode);
    }

    public void setAngleGoalRads(double angleRads) {
        this.goalState.position = angleRads;
        this.goalState.velocity = 0.0;
        if (!this.motionProfiling) {
            this.setpointState.position = this.measuredState.position;
            this.setpointState.velocity = this.measuredState.velocity;
            this.motionProfiling = true;
        }
        var newSetpointState = this.motionProfile.calculate(RobotConstants.rioUpdatePeriodSecs, this.setpointState, this.goalState);
        var ffout = this.feedforward.calculateWithVelocities(this.setpointState.position, this.setpointState.velocity, newSetpointState.velocity);
        this.setpointState.position = newSetpointState.position;
        this.setpointState.velocity = newSetpointState.velocity;
        this.io.setPosition(
            this.setpointState.position,
            this.setpointState.velocity,
            ffout
        );
        Logger.recordOutput("Superstructure/Pivot/FF/FF Out", ffout);
        Logger.recordOutput("Superstructure/Pivot/Angle/Setpoint", this.setpointState.position);
        Logger.recordOutput("Superstructure/Pivot/Velocity/Setpoint", this.setpointState.velocity);
        Logger.recordOutput("Superstructure/Pivot/Angle/Goal", this.goalState.position);
        Logger.recordOutput("Superstructure/Pivot/Velocity/Goal", this.goalState.velocity);
    }
}
