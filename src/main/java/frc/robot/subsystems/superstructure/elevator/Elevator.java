package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ElevatorFeedforward;
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
import frc.util.robotStructure.linear.ExtenderMech;

public class Elevator {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private static final LoggedTunable<TrapezoidProfile.Constraints> profileConsts = LoggedTunable.fromDashboardUnits(
        "Superstructure/Elevator/Profile",
        InchesPerSecond,
        InchesPerSecond.per(Second),
        MetersPerSecond,
        MetersPerSecondPerSecond,
        new TrapezoidProfile.Constraints(
            80,
            240
        )
    );
    private static final LoggedTunable<FFConstants> ffConsts = LoggedTunable.from(
        "Superstructure/Elevator/FF",
        new FFConstants(
            0.2,
            0.3,
            2,
            0
        )
    );
    private static final LoggedTunable<PIDConstants> pidConsts = LoggedTunable.from(
        "Superstructure/Elevator/PID",
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
    private final ElevatorFeedforward feedforward = new ElevatorFeedforward(0,0,0,0);

    private double lengthMeters = 0.0;
    private double velocityMetersPerSec = 0.0;

    public final ExtenderMech stage2Mech = new ExtenderMech(ElevatorConstants.stage2Base);
    public final ExtenderMech stage3Mech = new ExtenderMech(ElevatorConstants.stage3Base);
    public final ExtenderMech stage4Mech = new ExtenderMech(ElevatorConstants.stage4Base);

    private final DeviceFaultAlerts motorActiveFaultsAlert = new DeviceFaultAlerts(new Alert("Superstructure/Elevator/Alerts", "Motor has active faults: ", AlertType.kError));
    private final DeviceFaultAlerts motorStickyFaultsAlert = new DeviceFaultAlerts(new Alert("Superstructure/Elevator/Alerts", "Motor has sticky faults: ", AlertType.kWarning), FaultType.ForwardSoftLimit, FaultType.ReverseSoftLimit, FaultType.StatorCurrentLimit, FaultType.SupplyCurrentLimit);
    private final DeviceFaultAlerts encoderActiveFaultsAlert = new DeviceFaultAlerts(new Alert("Superstructure/Elevator/Alerts", "Encoder has active faults: ", AlertType.kError));
    private final DeviceFaultAlerts encoderStickyFaultsAlert = new DeviceFaultAlerts(new Alert("Superstructure/Elevator/Alerts", "Encoder has sticky faults: ", AlertType.kWarning));
    private final DeviceFaultClearer motorStickyFaultClearer = new DeviceFaultClearer("Superstructure/Elevator/Motor Sticky Faults");
    private final DeviceFaultClearer encoderStickyFaultClearer = new DeviceFaultClearer("Superstructure/Elevator/Encoder Sticky Faults");

    public Elevator(ElevatorIO io) {
        System.out.println("[Init Elevator] Instantiating Elevator with " + io.getClass().getSimpleName());
        this.io = io;

        ffConsts.get().update(this.feedforward);
        this.io.configPID(pidConsts.get());
    }

    public void periodic() {
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Superstructure/Elevator/Before");
        this.io.updateInputs(this.inputs);
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Superstructure/Elevator/Update Inputs");
        Logger.processInputs("Inputs/Superstructure/Elevator", this.inputs);
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Superstructure/Elevator/Process Inputs");

        var stageDistMeters = ElevatorConstants.stage1LinearRelation.radiansToMeters(ElevatorConstants.sensorToMechanism.applyUnsigned(this.inputs.encoder.getPositionRads()));

        this.lengthMeters = stageDistMeters * ElevatorConstants.movingStageCount;
        this.velocityMetersPerSec = ElevatorConstants.stage1LinearRelation.radiansToMeters(ElevatorConstants.sensorToMechanism.applyUnsigned(this.inputs.encoder.getVelocityRadsPerSec())) * ElevatorConstants.movingStageCount;

        this.measuredState.position = this.getLengthMeters();
        this.measuredState.velocity = this.getVelocityMetersPerSec();

        Logger.recordOutput("Superstructure/Elevator/Length/Measured", this.getLengthMeters());
        Logger.recordOutput("Superstructure/Elevator/Velocity/Measured", this.getVelocityMetersPerSec());

        this.stage2Mech.setMeters(stageDistMeters);
        this.stage3Mech.setMeters(stageDistMeters);
        this.stage4Mech.setMeters(stageDistMeters);

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
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Superstructure/Elevator/Periodic");
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Superstructure/Elevator");
    }

    public double getLengthMeters() {
        return this.lengthMeters;
    }
    public double getVelocityMetersPerSec() {
        return this.velocityMetersPerSec;
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
    
    public void setLengthGoalMeters(double lengthMeters) {
        this.goalState.position = lengthMeters;
        this.goalState.velocity = 0.0;
        if (!this.motionProfiling) {
            this.setpointState.position = this.measuredState.position;
            this.setpointState.velocity = this.measuredState.velocity;
            this.motionProfiling = true;
        }
        var newSetpointState = this.motionProfile.calculate(RobotConstants.rioUpdatePeriodSecs, this.setpointState, this.goalState);
        var ffout = this.feedforward.calculateWithVelocities(this.setpointState.velocity, newSetpointState.velocity);
        this.setpointState.position = newSetpointState.position;
        this.setpointState.velocity = newSetpointState.velocity;
        this.io.setPosition(
            ElevatorConstants.stage1LinearRelation.metersToRadians(this.setpointState.position / ElevatorConstants.movingStageCount),
            ElevatorConstants.stage1LinearRelation.metersToRadians(this.setpointState.velocity / ElevatorConstants.movingStageCount),
            ffout
        );
        Logger.recordOutput("Superstructure/Elevator/FF/FF Out", ffout);
        Logger.recordOutput("Superstructure/Elevator/Length/Setpoint", this.setpointState.position);
        Logger.recordOutput("Superstructure/Elevator/Velocity/Setpoint", this.setpointState.velocity);
        Logger.recordOutput("Superstructure/Elevator/Length/Goal", goalState.position);
        Logger.recordOutput("Superstructure/Elevator/Velocity/Goal", goalState.velocity);
    }
}
