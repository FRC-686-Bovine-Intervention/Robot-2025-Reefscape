package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.constants.RobotConstants;
import frc.util.NeutralMode;
import frc.util.faults.DeviceFaultAlerts;
import frc.util.faults.DeviceFaultClearer;
import frc.util.faults.DeviceFaults;
import frc.util.faults.DeviceFaults.FaultType;
import frc.util.loggerUtil.tunables.LoggedTunableFF;
import frc.util.loggerUtil.tunables.LoggedTunableLinearProfile;
import frc.util.loggerUtil.tunables.LoggedTunablePID;
import frc.util.robotStructure.linear.ExtenderMech;

public class Elevator {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private final LoggedTunableLinearProfile profileConsts = new LoggedTunableLinearProfile(
        "Superstructure/Elevator/Profile",
        InchesPerSecond.of(80),
        InchesPerSecond.per(Second).of(240)
    );
    private final LoggedTunableFF ffConsts = new LoggedTunableFF(
        "Superstructure/Elevator/FF",
        0.2,
        0.3,
        2,
        0
    );
    private final LoggedTunablePID pidConsts = new LoggedTunablePID(
        "Superstructure/Elevator/PID",
        50,
        0,
        0
    );

    private TrapezoidProfile motionProfile = profileConsts.getTrapezoidProfile();
    private State setpointState = null;
    private final ElevatorFeedforward feedforward = new ElevatorFeedforward(0,0,0,0);

    private final MutDistance length = Meters.mutable(0);
    private final MutLinearVelocity velocity = MetersPerSecond.mutable(0);

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

        ffConsts.update(this.feedforward);
        this.io.configPID(pidConsts.getConstants());
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Superstructure/Elevator", inputs);

        this.length.mut_replace(ElevatorConstants.stage1LinearRelation.angleToDistance(ElevatorConstants.sensorToMechanism.applyUnsigned(inputs.encoder.position)).times(ElevatorConstants.movingStageCount));
        this.velocity.mut_replace(ElevatorConstants.stage1LinearRelation.angularVelocityToLinearVelocity(ElevatorConstants.sensorToMechanism.applyUnsigned(inputs.encoder.velocity)).times(ElevatorConstants.movingStageCount));

        Logger.recordOutput("Superstructure/Elevator/Length/Measured", this.getLength());
        Logger.recordOutput("Superstructure/Elevator/Velocity/Measured", this.getVelocity());

        var stageDist = this.getLength().div(ElevatorConstants.movingStageCount);

        this.stage2Mech.set(stageDist);
        this.stage3Mech.set(stageDist);
        this.stage4Mech.set(stageDist);

        if (profileConsts.hasChanged(hashCode())) {
            this.motionProfile = profileConsts.getTrapezoidProfile();
        }
        if (ffConsts.hasChanged(hashCode())) {
            ffConsts.update(this.feedforward);
        }
        if (pidConsts.hasChanged(hashCode())) {
            io.configPID(pidConsts.getConstants());
        }

        this.motorActiveFaultsAlert.updateFrom(this.inputs.motorFaults.activeFaults);
        this.motorStickyFaultsAlert.updateFrom(this.inputs.motorFaults.stickyFaults);
        this.encoderActiveFaultsAlert.updateFrom(this.inputs.encoderFaults.activeFaults);
        this.encoderStickyFaultsAlert.updateFrom(this.inputs.encoderFaults.stickyFaults);
        this.motorStickyFaultClearer.clear(this.inputs.motorFaults.stickyFaults, this.io::clearMotorStickyFaults, DeviceFaults.allMask);
        this.encoderStickyFaultClearer.clear(this.inputs.encoderFaults.stickyFaults, this.io::clearEncoderStickyFaults, DeviceFaults.allMask);
    }

    public Distance getLength() {
        return length;
    }
    public LinearVelocity getVelocity() {
        return velocity;
    }
    public Voltage getVoltage() {
        return inputs.motor.motor.appliedVoltage;
    }

    public void setVoltage(Measure<VoltageUnit> voltage) {
        this.setpointState = null;
        io.setVoltage(voltage);
    }
    public void stop(Optional<NeutralMode> neutralMode) {
        this.setpointState = null;
        this.io.stop(neutralMode);
    }
    
    public void setLengthGoal(Measure<DistanceUnit> length) {
        if (this.setpointState == null) {
            this.setpointState = new State(this.getLength().in(Meters), this.getVelocity().in(MetersPerSecond));
        }
        var goalState = new State(length.in(Meters), 0);
        var newSetpointState = motionProfile.calculate(RobotConstants.rioUpdatePeriodSecs, this.setpointState, goalState);
        var ffout = feedforward.calculateWithVelocities(this.setpointState.velocity, newSetpointState.velocity);
        this.setpointState = newSetpointState;
        io.setPosition(
            Radians.of(this.setpointState.position / ElevatorConstants.movingStageCount / ElevatorConstants.stage1LinearRelation.effectiveRadius().in(Meters)),
            RadiansPerSecond.of(this.setpointState.velocity / ElevatorConstants.movingStageCount / ElevatorConstants.stage1LinearRelation.effectiveRadius().in(Meters)),
            Volts.of(ffout)
        );
        Logger.recordOutput("Superstructure/Elevator/Length/Setpoint", this.setpointState.position);
        Logger.recordOutput("Superstructure/Elevator/Velocity/Setpoint", this.setpointState.velocity);
        Logger.recordOutput("Superstructure/Elevator/Length/Goal", goalState.position);
        Logger.recordOutput("Superstructure/Elevator/Velocity/Goal", goalState.velocity);
    }
}
