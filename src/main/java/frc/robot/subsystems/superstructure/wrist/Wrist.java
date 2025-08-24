package frc.robot.subsystems.superstructure.wrist;

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

public class Wrist {
    private final WristIO io;
    private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

    private static final LoggedTunable<TrapezoidProfile.Constraints> profileConsts = LoggedTunable.from(
        "Superstructure/Wrist/Profile",
        new TrapezoidProfile.Constraints(
            RadiansPerSecond.convertFrom(1080, DegreesPerSecond),
            RadiansPerSecondPerSecond.convertFrom(2160, DegreesPerSecondPerSecond)
        )
    );
    private static final LoggedTunable<FFConstants> ffConsts = LoggedTunable.from(
        "Superstructure/Wrist/FF",
        new FFConstants(
            0,
            0,
            5 /2/Math.PI,
            0
        )
    );
    private static final LoggedTunable<PIDConstants> pidConsts = LoggedTunable.from(
        "Superstructure/Wrist/PID",
        new PIDConstants(
            50,
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

    public final ArmMech mech = new ArmMech(WristConstants.wristBase);

    private final DeviceFaultAlerts motorActiveFaultsAlert = new DeviceFaultAlerts(new Alert("Superstructure/Wrist/Alerts", "Motor has active faults: ", AlertType.kError));
    private final DeviceFaultAlerts motorStickyFaultsAlert = new DeviceFaultAlerts(new Alert("Superstructure/Wrist/Alerts", "Motor has sticky faults: ", AlertType.kWarning), FaultType.ForwardSoftLimit, FaultType.ReverseSoftLimit, FaultType.StatorCurrentLimit, FaultType.SupplyCurrentLimit);
    private final DeviceFaultAlerts encoderActiveFaultsAlert = new DeviceFaultAlerts(new Alert("Superstructure/Wrist/Alerts", "Encoder has active faults: ", AlertType.kError));
    private final DeviceFaultAlerts encoderStickyFaultsAlert = new DeviceFaultAlerts(new Alert("Superstructure/Wrist/Alerts", "Encoder has sticky faults: ", AlertType.kWarning));
    private final DeviceFaultClearer motorStickyFaultClearer = new DeviceFaultClearer("Superstructure/Wrist/Motor Sticky Faults");
    private final DeviceFaultClearer encoderStickyFaultClearer = new DeviceFaultClearer("Superstructure/Wrist/Encoder Sticky Faults");

    public Wrist(WristIO io) {
        System.out.println("[Init Wrist] Instantiating Wrist with " + io.getClass().getSimpleName());
        this.io = io;
        
        ffConsts.get().update(this.feedforward);
        this.io.configPID(pidConsts.get());
    }

    public void periodic() {
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Superstructure/Wrist/Before");
        this.io.updateInputs(this.inputs);
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Superstructure/Wrist/Update Inputs");
        Logger.processInputs("Inputs/Superstructure/Wrist", this.inputs);
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Superstructure/Wrist/Process Inputs");
        
        this.angleRads = WristConstants.sensorToMechanism.applyUnsigned(this.inputs.encoder.getPositionRads());
        this.velocityRadsPerSec = WristConstants.sensorToMechanism.applyUnsigned(this.inputs.encoder.getVelocityRadsPerSec());

        this.measuredState.position = this.getAngleRads();
        this.measuredState.velocity = this.getVelocityRadsPerSec();
        
        Logger.recordOutput("Superstructure/Wrist/Angle/Measured", this.getAngleRads());
        Logger.recordOutput("Superstructure/Wrist/Velocity/Measured", this.getVelocityRadsPerSec());
        
        this.mech.setRads(this.getAngleRads());
        
        if (profileConsts.hasChanged(hashCode())) {
            this.motionProfile = new TrapezoidProfile(profileConsts.get());
        }
        if (ffConsts.hasChanged(hashCode())) {
            ffConsts.get().update(this.feedforward);
        }
        if (pidConsts.hasChanged(hashCode())) {
            this.io.configPID(pidConsts.get());
        }
        
        // this.motorActiveFaultsAlert.updateFrom(this.inputs.motorFaults.activeFaults);
        // this.motorStickyFaultsAlert.updateFrom(this.inputs.motorFaults.stickyFaults);
        // this.encoderActiveFaultsAlert.updateFrom(this.inputs.encoderFaults.activeFaults);
        // this.encoderStickyFaultsAlert.updateFrom(this.inputs.encoderFaults.stickyFaults);
        // this.motorStickyFaultClearer.clear(this.inputs.motorFaults.stickyFaults, this.io::clearMotorStickyFaults, DeviceFaults.allMask);
        // this.encoderStickyFaultClearer.clear(this.inputs.encoderFaults.stickyFaults, this.io::clearEncoderStickyFaults, DeviceFaults.allMask);
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Superstructure/Wrist/Periodic");
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Superstructure/Wrist");
    }

    public double getAngleRads() {
        return this.angleRads;
    }
    public double getVelocityRadsPerSec() {
        return this.velocityRadsPerSec;
    }
    public double getAppliedVolts() {
        return this.inputs.motor.motor.getAppliedVolts();
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
        Logger.recordOutput("Superstructure/Wrist/FF/FF Out", ffout);
        Logger.recordOutput("Superstructure/Wrist/Angle/Setpoint", this.setpointState.position);
        Logger.recordOutput("Superstructure/Wrist/Velocity/Setpoint", this.setpointState.velocity);
        Logger.recordOutput("Superstructure/Wrist/Angle/Goal", this.goalState.position);
        Logger.recordOutput("Superstructure/Wrist/Velocity/Goal", this.goalState.velocity);
    }
}
