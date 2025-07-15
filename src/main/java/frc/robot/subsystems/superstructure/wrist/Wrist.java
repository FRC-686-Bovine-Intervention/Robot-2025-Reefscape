package frc.robot.subsystems.superstructure.wrist;

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
import frc.robot.constants.RobotConstants;
import frc.util.NeutralMode;
import frc.util.loggerUtil.tunables.LoggedTunableAngularProfile;
import frc.util.loggerUtil.tunables.LoggedTunableFF;
import frc.util.loggerUtil.tunables.LoggedTunablePID;
import frc.util.robotStructure.angle.ArmMech;

public class Wrist {
    private final WristIO io;
    private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

    private static final LoggedTunableAngularProfile profileConsts = new LoggedTunableAngularProfile(
        "Superstructure/Wrist/Profile",
        DegreesPerSecond.of(720),
        DegreesPerSecondPerSecond.of(1080)
    );
    private static final LoggedTunableFF ffConsts = new LoggedTunableFF(
        "Superstructure/Wrist/FF",
        0,
        0,
        5,
        0
    );
    private static final LoggedTunablePID pidConsts = new LoggedTunablePID(
        "Superstructure/Wrist/PID",
        50,
        0,
        0
    );

    private TrapezoidProfile motionProfile = profileConsts.getTrapezoidProfile();
    private State setpointState = null;
    private final ArmFeedforward feedforward = new ArmFeedforward(0,0,0,0);
    
    private final MutAngle angle = Radians.mutable(0);
    private final MutAngularVelocity velocity = RadiansPerSecond.mutable(0);

    public final ArmMech mech = new ArmMech(WristConstants.wristBase);

    public Wrist(WristIO io) {
        System.out.println("[Init Wrist] Instantiating Wrist with " + io.getClass().getSimpleName());
        this.io = io;
        
        ffConsts.update(this.feedforward);
        this.io.configPID(pidConsts.getConstants());
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Superstructure/Wrist", inputs);

        this.angle.mut_replace(WristConstants.sensorToMechanism.applyUnsigned(this.inputs.encoder.position));
        this.velocity.mut_replace(WristConstants.sensorToMechanism.applyUnsigned(this.inputs.encoder.velocity));

        this.mech.set(this.getAngle());

        Logger.recordOutput("Superstructure/Wrist/Measured/Angle", this.getAngle());
        Logger.recordOutput("Superstructure/Wrist/Measured/Velocity", this.getVelocity());

        if (profileConsts.hasChanged(hashCode())) {
            this.motionProfile = profileConsts.getTrapezoidProfile();
        }
        if (ffConsts.hasChanged(hashCode())) {
            ffConsts.update(this.feedforward);
        }
        if (pidConsts.hasChanged(hashCode())) {
            this.io.configPID(pidConsts.getConstants());
        }
    }

    public Angle getAngle() {
        return this.angle;
    }
    public AngularVelocity getVelocity() {
        return this.velocity;
    }
    public Voltage getVoltage() {
        return this.inputs.motor.motor.appliedVoltage;
    }
    
    public void setVoltage(Measure<VoltageUnit> voltage) {
        this.setpointState = null;
        io.setVoltage(voltage);
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
        this.setpointState = newSetpointState;
        this.io.setPosition(
            Radians.of(this.setpointState.position),
            RadiansPerSecond.of(this.setpointState.velocity),
            Volts.of(ffout)
        );
        Logger.recordOutput("Superstructure/Wrist/Setpoint/Angle", this.setpointState.position);
        Logger.recordOutput("Superstructure/Wrist/Setpoint/Velocity", this.setpointState.velocity);
        Logger.recordOutput("Superstructure/Wrist/Goal/Angle", goalState.position);
        Logger.recordOutput("Superstructure/Wrist/Goal/Velocity", goalState.velocity);
    }
}
