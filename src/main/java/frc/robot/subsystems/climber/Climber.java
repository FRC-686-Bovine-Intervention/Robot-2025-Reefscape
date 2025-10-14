package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.leds.Leds;
import frc.util.EdgeDetector;
import frc.util.LoggedTracer;
import frc.util.NeutralMode;
import frc.util.loggerUtil.tunables.LoggedTunable;
import frc.util.robotStructure.angle.AngularMech;

public class Climber extends SubsystemBase {
    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    private static final LoggedTunable<Voltage> idleVoltage = LoggedTunable.from("Climber/Idle Voltage", Volts::of, -1);
    private static final LoggedTunable<Angle> ratchetEngageAngle = LoggedTunable.from("Climber/Ratchet/Engage Angle", Degrees::of, 55);
    private static final LoggedTunable<Angle> ratchetDisengageAngle = LoggedTunable.from("Climber/Ratchet/Disengage Angle", Degrees::of, 105);
    private static final LoggedTunable<Angle> deployAngle = LoggedTunable.from("Climber/Deploy Angle", Rotations::of, 5.5);
    private static final LoggedTunable<Angle> climbAngle = LoggedTunable.from("Climber/Climb Angle", Rotations::of, 2.65);
    private static final LoggedTunable<Angle> climbTolerance = LoggedTunable.from("Climber/Climb Tolerance", Rotations::of, 0.05);
    private static final LoggedTunable<Time> climbTime = LoggedTunable.from("Climber/Climb Time", Seconds::of, 1);
    private static final LoggedTunable<Time> ratchetTime = LoggedTunable.from("Climber/Ratchet Time", Seconds::of, 0.25);

    private double angleRads = 0.0;
    private double velocityRadsPerSec = 0.0;

    public final AngularMech mech = new AngularMech(ClimberConstants.climberBase, VecBuilder.fill(0,1,0));

    // private final DeviceFaultAlerts motorActiveFaultsAlert = new DeviceFaultAlerts(new Alert("Climber/Alerts", "Motor has active faults: ", AlertType.kError));
    // private final DeviceFaultAlerts motorStickyFaultsAlert = new DeviceFaultAlerts(new Alert("Climber/Alerts", "Motor has sticky faults: ", AlertType.kWarning), FaultType.StatorCurrentLimit, FaultType.SupplyCurrentLimit);
    // private final DeviceFaultClearer motorStickyFaultClearer = new DeviceFaultClearer("Climber/Motor Sticky Faults");

    private final Alert motorDisconnectedAlert = new Alert("Climber/Alerts", "Motor Disconnected", AlertType.kError);
    private final Alert motorDisconnectedGlobalAlert = new Alert("Climber Motor Disconnected!", AlertType.kError);

    private final EdgeDetector zeroSensorEdgeDetector = new EdgeDetector();

    private boolean ratchetEngaged = true;

    public Climber(ClimberIO io) {
        System.out.println("[Init Climber] Instantiating Climber with " + io.getClass().getSimpleName());
        this.io = io;
    }

    @Override
    public void periodic() {
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Climber/Before");
        this.io.updateInputs(this.inputs);
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Climber/Update Inputs");
        Logger.processInputs("Inputs/Climber", this.inputs);
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Climber/Process Inputs");

        this.angleRads = ClimberConstants.sensorToMechanismRatio.applyUnsigned(this.inputs.motor.encoder.getPositionRads());
        this.velocityRadsPerSec = ClimberConstants.sensorToMechanismRatio.applyUnsigned(this.inputs.motor.encoder.getVelocityRadsPerSec());

        this.zeroSensorEdgeDetector.update(this.inputs.sensor);

        Logger.recordOutput("Climber/Position", this.getAngleRads());
        Logger.recordOutput("Climber/Ratchet Engaged", this.ratchetEngaged);
        Logger.recordOutput("Climber/Zero Sensor", this.inputs.sensor);

        if (this.zeroSensorEdgeDetector.risingEdge()) {
            this.io.setMotorEncoderPosRads(ClimberConstants.climberMinimumAngle.in(Radians));
        }

        var percentToDeploy = this.getAngleRads() / deployAngle.get().in(Radians);
        this.mech.setRads(percentToDeploy * ClimberConstants.climberMaxAngle.in(Radians));

        Leds.getInstance().climbing.setPos(this.getAngleRads() / climbAngle.get().in(Radians));

        // this.motorActiveFaultsAlert.updateFrom(this.inputs.motorFaults.activeFaults);
        // this.motorStickyFaultsAlert.updateFrom(this.inputs.motorFaults.stickyFaults);
        // this.motorStickyFaultClearer.clear(this.inputs.motorFaults.stickyFaults, this.io::clearMotorStickyFaults, DeviceFaults.allMask);

        this.motorDisconnectedAlert.set(!this.inputs.motorConnected);
        this.motorDisconnectedGlobalAlert.set(!this.inputs.motorConnected);

        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Climber/Periodic");
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Climber");
    }

    public double getAngleRads() {
        return this.angleRads;
    }

    public double getVelocityRadsPerSec() {
        return this.velocityRadsPerSec;
    }

    public Command idle() {
        final var climber = this;
        return new Command() {
            private final Timer ratchetTimer = new Timer();
            {
                this.addRequirements(climber);
                this.setName("Idle");
            }
            
            @Override
            public void initialize() {
                Leds.getInstance().climbing.setFlag(false);
                Leds.getInstance().climbingComplete.setFlag(false);
            }

            @Override
            public void execute() {
                climber.io.setRatchetServoAngle(ratchetDisengageAngle.get().in(Radians));
                if (climber.ratchetEngaged) {
                    climber.io.stop(NeutralMode.COAST);
                    this.ratchetTimer.start();
                    if (this.ratchetTimer.hasElapsed(ratchetTime.get().in(Seconds))) {
                        this.ratchetTimer.stop();
                        this.ratchetTimer.reset();
                        climber.ratchetEngaged = false;
                    }
                } else {
                    climber.io.setVolts(idleVoltage.get().in(Volts));
                }
            }

            @Override
            public void end(boolean interrupted) {
                this.ratchetTimer.stop();
                this.ratchetTimer.reset();
                climber.io.stop(NeutralMode.COAST);
            }
        };
    }
    
    public Command prepareClimb() {
        final var climber = this;
        return new Command() {
            private final Timer ratchetTimer = new Timer();
            {
                this.addRequirements(climber);
                this.setName("Prepare Climb");
            }

            @Override
            public void initialize() {
                Leds.getInstance().prepareClimbing.setFlag(true);
                Leds.getInstance().climbing.setFlag(false);
                Leds.getInstance().climbingComplete.setFlag(false);
            }

            @Override
            public void execute() {
                climber.io.setRatchetServoAngle(ratchetDisengageAngle.get().in(Radians));
                if (climber.ratchetEngaged) {
                    climber.io.stop(NeutralMode.COAST);
                    this.ratchetTimer.start();
                    if (this.ratchetTimer.hasElapsed(ratchetTime.get().in(Seconds))) {
                        this.ratchetTimer.stop();
                        this.ratchetTimer.reset();
                        climber.ratchetEngaged = false;
                    }
                } else {
                    climber.io.setNonClimbingAngle(deployAngle.get().in(Radians));
                }
            }

            @Override
            public void end(boolean interrupted) {
                Leds.getInstance().prepareClimbing.setFlag(false);
                this.ratchetTimer.stop();
                this.ratchetTimer.reset();
            }
        };
    }

    public Command climb() {
        final var climber = this;
        return new Command() {
            private final Timer climbTimer = new Timer();
            {
                this.addRequirements(climber);
                this.setName("Climb");
            }

            @Override
            public void initialize() {
                this.climbTimer.reset();
                Leds.getInstance().climbing.setFlag(true);
                Leds.getInstance().climbingComplete.setFlag(false);
            }

            @Override
            public void execute() {
                climber.io.setRatchetServoAngle(ratchetEngageAngle.get().in(Radians));
                climber.ratchetEngaged = true;
                if (MathUtil.isNear(climbAngle.get().in(Radians), climber.getAngleRads(), climbTolerance.get().in(Radians))) {
                    this.climbTimer.start();
                }
                if (this.climbTimer.hasElapsed(climbTime.get().in(Seconds))) {
                    this.climbTimer.stop();
                    climber.io.stop(NeutralMode.BRAKE);
                    Leds.getInstance().climbingComplete.setFlag(true);
                } else {
                    climber.io.setClimbingAngle(climbAngle.get().in(Radians));
                }
            }

            @Override
            public void end(boolean interrupted) {
                this.climbTimer.stop();
                climber.io.stop(NeutralMode.BRAKE);
            }
        };
    }
}