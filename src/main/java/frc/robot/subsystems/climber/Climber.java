package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.leds.Leds;
import frc.util.LoggedTracer;
import frc.util.faults.DeviceFaultAlerts;
import frc.util.faults.DeviceFaultClearer;
import frc.util.faults.DeviceFaults.FaultType;
import frc.util.loggerUtil.tunables.LoggedTunableMeasure;
import frc.util.misc.MeasureUtil;
import frc.util.robotStructure.angle.AngularMech;

public class Climber extends SubsystemBase {
    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    private static final LoggedTunableMeasure<VoltageUnit> idleVoltage = new LoggedTunableMeasure<>("Climber/Idle Voltage", Volts.of(-1));
    private static final LoggedTunableMeasure<AngleUnit> ratchetEngageAngle = new LoggedTunableMeasure<>("Climber/Ratchet/Engage Angle", Degrees.of(55));
    private static final LoggedTunableMeasure<AngleUnit> ratchetDisengageAngle = new LoggedTunableMeasure<>("Climber/Ratchet/Disengage Angle", Degrees.of(100));
    private static final LoggedTunableMeasure<AngleUnit> deployAngle = new LoggedTunableMeasure<>("Climber/Deploy Angle", Rotations.of(5.5));
    private static final LoggedTunableMeasure<AngleUnit> climbAngle = new LoggedTunableMeasure<>("Climber/Climb Angle", Rotations.of(2.65));
    private static final LoggedTunableMeasure<AngleUnit> climbTolerance = new LoggedTunableMeasure<>("Climber/Climb Tolerance", Rotations.of(0.05));
    private static final LoggedTunableMeasure<TimeUnit> climbTime = new LoggedTunableMeasure<>("Climber/Climb Time", Seconds.of(1));
    private static final LoggedTunableMeasure<TimeUnit> ratchetTime = new LoggedTunableMeasure<>("Climber/Ratchet Time", Seconds.of(0.25));

    private final MutAngle angle = Radians.mutable(0);

    public final AngularMech mech = new AngularMech(ClimberConstants.climberBase, VecBuilder.fill(0,1,0));

    private final DeviceFaultAlerts motorActiveFaultsAlert = new DeviceFaultAlerts(new Alert("Climber/Alerts", "Motor has active faults: ", AlertType.kError));
    private final DeviceFaultAlerts motorStickyFaultsAlert = new DeviceFaultAlerts(new Alert("Climber/Alerts", "Motor has sticky faults: ", AlertType.kWarning), FaultType.StatorCurrentLimit, FaultType.SupplyCurrentLimit);
    private final DeviceFaultClearer motorStickyFaultClearer = new DeviceFaultClearer("Climber/Motor Sticky Faults");

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

        this.angle.mut_replace(ClimberConstants.sensorToMechanismRatio.applyUnsigned(inputs.motor.encoder.getPositionRads()), Radians);

        Logger.recordOutput("Climber/Position", this.getAngle());
        Logger.recordOutput("Climber/Ratchet Engaged", ratchetEngaged);

        var percentToDeploy = this.angle.baseUnitMagnitude() / deployAngle.get().baseUnitMagnitude();
        this.mech.set(ClimberConstants.climberMaxAngle.times(percentToDeploy));

        Leds.getInstance().climbing.setPos(this.angle.baseUnitMagnitude() / climbAngle.get().baseUnitMagnitude());

        // this.motorActiveFaultsAlert.updateFrom(this.inputs.motorFaults.activeFaults);
        // this.motorStickyFaultsAlert.updateFrom(this.inputs.motorFaults.stickyFaults);
        // this.motorStickyFaultClearer.clear(this.inputs.motorFaults.stickyFaults, this.io::clearMotorStickyFaults, DeviceFaults.allMask);
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Climber/Periodic");
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Climber");
    }

    public Angle getAngle() {
        return this.angle;
    }

    public Command idle() {
        var subsystem = this;
        return new Command() {
            private final Timer ratchetTimer = new Timer();
            {
                addRequirements(subsystem);
                setName("Idle");
            }
            @Override
            public void initialize() {               
            }

            @Override
            public void execute() {
                io.setRatchetServoAngle(ratchetDisengageAngle.get());
                if (ratchetEngaged) {
                    io.setVoltage(Volts.zero(), false);
                    ratchetTimer.start();
                    if(ratchetTimer.hasElapsed(ratchetTime.get().in(Seconds))){
                        ratchetEngaged = false;
                    }
                } else {
                    ratchetTimer.stop();
                    ratchetTimer.reset();
                    io.setVoltage(idleVoltage.get(), false);
                }
            }
            @Override
            public void end(boolean interrupted) {
                
            }
        };
    }
    
    public Command prepareClimb() {
        var subsystem = this;
        return new Command() {
            private final Timer ratchetTimer = new Timer();
            {
                addRequirements(subsystem);
                setName("Prepare Climb");
            }
            @Override
            public void initialize() {
                Leds.getInstance().prepareClimbing.setFlag(true);
            }

            @Override
            public void execute() {
                io.setRatchetServoAngle(ratchetDisengageAngle.get());
                if(ratchetEngaged){
                    ratchetTimer.start();
                    if(ratchetTimer.hasElapsed(ratchetTime.get().in(Seconds))){
                        ratchetEngaged = false;
                    }
                } else {
                    ratchetTimer.stop();
                    ratchetTimer.reset();
                    io.setNonClimbingAngle(deployAngle.get());
                }
            }
            @Override
            public void end(boolean interrupted) {
                Leds.getInstance().prepareClimbing.setFlag(false);
            }
        };
    }

    public Command climb() {
        var subsystem = this;
        return new Command() {
            private final Timer climbTimer = new Timer();
            {
                addRequirements(subsystem);
                setName("Climb");
            }
            @Override
            public void initialize() {
                climbTimer.reset();
            }

            @Override
            public void execute() {
                io.setRatchetServoAngle(ratchetEngageAngle.get());
                ratchetEngaged = true;
                if (MeasureUtil.isNear(climbAngle.get(), getAngle(), climbTolerance.get())) {
                    climbTimer.start();
                }
                if (climbTimer.hasElapsed(climbTime.get().in(Seconds))) {
                    io.setVoltage(Volts.zero(), true);
                    climbTimer.stop();
                } else {
                    io.setClimbingAngle(climbAngle.get());
                }
            }
            @Override
            public void end(boolean interrupted) {
                
            }
        };
    }

    public Command testEngageRatchet() {
        var subsystem = this;
        return new Command() {
            {
                addRequirements(subsystem);
                setName("Engage Ratchet");
            }
            @Override
            public void initialize() {   

            }

            @Override
            public void execute() {
                io.setRatchetServoAngle(ratchetEngageAngle.get());
                ratchetEngaged = true;
                io.setVoltage(Volts.zero(), false);
            }
            @Override
            public void end(boolean interrupted) {
                
            }
        };
    }
    public Command testDisengageRatchet() {
        var subsystem = this;
        return new Command() {
            {
                addRequirements(subsystem);
                setName("Disengage Ratchet");
            }
            @Override
            public void initialize() {   

            }

            @Override
            public void execute() {
                io.setRatchetServoAngle(ratchetDisengageAngle.get());
                ratchetEngaged = false;
                io.setVoltage(Volts.zero(), false);
            }
            @Override
            public void end(boolean interrupted) {
                
            }
        };
    }
}