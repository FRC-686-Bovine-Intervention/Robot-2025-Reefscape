package frc.robot.subsystems.superstructure.pivot;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.constants.RobotConstants;
import frc.util.LoggedTracer;
import frc.util.NeutralMode;
import frc.util.faults.DeviceFaultAlerts;
import frc.util.faults.DeviceFaultClearer;
import frc.util.faults.DeviceFaults;
import frc.util.faults.DeviceFaults.FaultType;
import frc.util.loggerUtil.tunables.LoggedTunableAngularProfile;
import frc.util.loggerUtil.tunables.LoggedTunableFF;
import frc.util.loggerUtil.tunables.LoggedTunablePID;
import frc.util.robotStructure.angle.ArmMech;

public class Pivot {
    private final PivotIO io;
    private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

    private static final LoggedTunableAngularProfile profileConsts = new LoggedTunableAngularProfile(
        "Superstructure/Pivot/Profile",
        DegreesPerSecond.of(225),
        DegreesPerSecondPerSecond.of(450)
    );
    private static final LoggedTunableFF ffConsts = new LoggedTunableFF(
        "Superstructure/Pivot/FF",
        0,
        0,
        17 /2/Math.PI,
        0
    );
    private static final LoggedTunablePID pidConsts = new LoggedTunablePID(
        "Superstructure/Pivot/PID",
        150,
        0,
        0
    );

    private TrapezoidProfile motionProfile = profileConsts.getTrapezoidProfile();
    private State setpointState = null;
    private final ArmFeedforward feedforward = new ArmFeedforward(0,0,0,0);

    private final MutAngle angle = Radians.mutable(0);
    private final MutAngularVelocity velocity = RadiansPerSecond.mutable(0);

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

        ffConsts.update(this.feedforward);
        this.io.configPID(pidConsts.getConstants());
    }

    public void periodic() {
        this.io.updateInputs(this.inputs);
        Logger.processInputs("Inputs/Superstructure/Pivot", this.inputs);
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Superstructure/Pivot/Process Inputs");

        this.angle.mut_replace(PivotConstants.sensorToMechanism.applyUnsigned(this.inputs.encoder.position));
        this.velocity.mut_replace(PivotConstants.sensorToMechanism.applyUnsigned(this.inputs.encoder.velocity));

        this.mech.set(this.getAngle());

        Logger.recordOutput("Superstructure/Pivot/Angle/Measured", this.getAngle());
        Logger.recordOutput("Superstructure/Pivot/Velocity/Measured", this.getVelocity());

        if (profileConsts.hasChanged(hashCode())) {
            this.motionProfile = profileConsts.getTrapezoidProfile();
        }
        if (ffConsts.hasChanged(hashCode())) {
            ffConsts.update(this.feedforward);
        }
        if (pidConsts.hasChanged(hashCode())) {
            this.io.configPID(pidConsts.getConstants());
        }

        this.leftMotorActiveFaultsAlert.updateFrom(this.inputs.leftMotorFaults.activeFaults);
        this.leftMotorStickyFaultsAlert.updateFrom(this.inputs.leftMotorFaults.stickyFaults);
        this.rightMotorActiveFaultsAlert.updateFrom(this.inputs.rightMotorFaults.activeFaults);
        this.rightMotorStickyFaultsAlert.updateFrom(this.inputs.rightMotorFaults.stickyFaults);
        this.encoderActiveFaultsAlert.updateFrom(this.inputs.encoderFaults.activeFaults);
        this.encoderStickyFaultsAlert.updateFrom(this.inputs.encoderFaults.stickyFaults);
        this.leftMotorStickyFaultClearer.clear(this.inputs.leftMotorFaults.stickyFaults, this.io::clearLeftMotorStickyFaults, DeviceFaults.allMask);
        this.rightMotorStickyFaultClearer.clear(this.inputs.rightMotorFaults.stickyFaults, this.io::clearRightMotorStickyFaults, DeviceFaults.allMask);
        this.encoderStickyFaultClearer.clear(this.inputs.encoderFaults.stickyFaults, this.io::clearEncoderStickyFaults, DeviceFaults.allMask);
    }

    public Angle getAngle() {
        return this.angle;
    }
    public AngularVelocity getVelocity() {
        return this.velocity;
    }
    public Voltage getVoltage() {
        return this.inputs.leftMotor.motor.appliedVoltage;
    }

    public void setVoltage(Measure<VoltageUnit> voltage) {
        this.setpointState = null;
        this.io.setVoltage(voltage);
    }
    public void stop(Optional<NeutralMode> neutralMode) {
        this.setpointState = null;
        this.io.stop(neutralMode);
    }

    public void setAngleGoal(Measure<AngleUnit> angle) {
        if (this.setpointState == null) {
            this.setpointState = new State(this.getAngle().in(Radians), this.getVelocity().in(RadiansPerSecond));
        }
        var goalState = new State(angle.in(Radians), 0);
        var newSetpointState = this.motionProfile.calculate(RobotConstants.rioUpdatePeriodSecs, this.setpointState, goalState);
        var ffout = this.feedforward.calculateWithVelocities(this.setpointState.position, this.setpointState.velocity, newSetpointState.velocity);
        Logger.recordOutput("Superstructure/Pivot/FF/FF Out", ffout);
        this.setpointState = newSetpointState;
        this.io.setPosition(
            Radians.of(this.setpointState.position),
            RadiansPerSecond.of(this.setpointState.velocity),
            Volts.of(ffout)
        );
        Logger.recordOutput("Superstructure/Pivot/Angle/Setpoint", this.setpointState.position);
        Logger.recordOutput("Superstructure/Pivot/Velocity/Setpoint", this.setpointState.velocity);
        Logger.recordOutput("Superstructure/Pivot/Angle/Goal", goalState.position);
        Logger.recordOutput("Superstructure/Pivot/Velocity/Goal", goalState.velocity);
    }
}
