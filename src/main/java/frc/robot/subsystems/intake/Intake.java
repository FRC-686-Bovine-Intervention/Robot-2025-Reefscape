package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.util.LoggedTracer;
import frc.util.loggerUtil.tunables.LoggedTunable;
import frc.util.robotStructure.GamepiecePose;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public static final LoggedTunable<Voltage> intakeVoltage = LoggedTunable.from("Intake/Voltages/Intake", Volts::of, 6);
    public static final LoggedTunable<Voltage> ejectVoltage = LoggedTunable.from("Intake/Voltages/Eject", Volts::of, -4);
    public static final LoggedTunable<Voltage> ejectLevel1Voltage = LoggedTunable.from("Intake/Voltages/Eject Level 1", Volts::of, -2);
    public static final LoggedTunable<Voltage> ejectAlgaeVoltage = LoggedTunable.from("Intake/Voltages/Algae", Volts::of, -12);
    public static final LoggedTunable<Voltage> coralHoldVoltage = LoggedTunable.from("Intake/Voltages/Hold Coral", Volts::of, 0.3);
    public static final LoggedTunable<Voltage> algaeHoldVoltage = LoggedTunable.from("Intake/Voltages/Hold Algae", Volts::of, 1);
    public static final LoggedTunable<Current> gamepieceDetectCurrent = LoggedTunable.from("Intake/Gamepiece Detect Current", Amps::of, 35);
    public static final LoggedTunable<Time> gamepieceDetectTime = LoggedTunable.from("Intake/Gamepiece Detect Time", Seconds::of, 2);

    public final GamepiecePose coralPose = new GamepiecePose(IntakeConstants.coralPose);
    public final GamepiecePose algaePose = new GamepiecePose(IntakeConstants.algaePose);
    
    private boolean hasGamepiece = false;
    private boolean grabbingCoral = true;
    public final Trigger hasCoral = new Trigger(() -> hasGamepiece && grabbingCoral);
    public final Trigger hasAlgae = new Trigger(() -> hasGamepiece && !grabbingCoral);

    // private final DeviceFaultAlerts motorActiveFaultsAlert = new DeviceFaultAlerts(new Alert("Intake/Alerts", "Motor has active faults: ", AlertType.kError));
    // private final DeviceFaultAlerts motorStickyFaultsAlert = new DeviceFaultAlerts(new Alert("Intake/Alerts", "Motor has sticky faults: ", AlertType.kWarning), FaultType.StatorCurrentLimit, FaultType.SupplyCurrentLimit);
    // private final DeviceFaultClearer motorStickyFaultClearer = new DeviceFaultClearer("Intake/Motor Sticky Faults");

    private final Alert motorDisconnectedAlert = new Alert("Intake/Alerts", "Motor Disconnected", AlertType.kError);
    private final Alert motorDisconnectedGlobalAlert = new Alert("Intake Motor Disconnected!", AlertType.kError);

    public Intake(IntakeIO io) {
        System.out.println("[Init Intake] Instantiated Intake with " + io.getClass().getSimpleName());
        this.io = io;
        SmartDashboard.putData("Subsystems/Intake", this);
    }

    private final Debouncer debouncer = new Debouncer(1, DebounceType.kRising);
    @Override
    public void periodic() {
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Intake/Before");
        this.io.updateInputs(this.inputs);
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Intake/Update Inputs");
        Logger.processInputs("Inputs/Intake", this.inputs);
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Intake/Process Inputs");

        if (gamepieceDetectTime.hasChanged(hashCode())) {
            this.debouncer.setDebounceTime(gamepieceDetectTime.get().in(Seconds));
        }
        var second = this.debouncer.calculate(this.inputs.coralSensor);
        if (this.inputs.coralSensor) {
            if (second || this.inputs.motor.getStatorCurrentAmps() > gamepieceDetectCurrent.get().in(Amps)) {
                hasGamepiece = true;
            }
        } else {
            hasGamepiece = false;
        }

        Logger.recordOutput("Intake/hasgamepiece", hasGamepiece);

        // this.motorActiveFaultsAlert.updateFrom(this.inputs.motorFaults.activeFaults);
        // this.motorStickyFaultsAlert.updateFrom(this.inputs.motorFaults.stickyFaults);
        // this.motorStickyFaultClearer.clear(this.inputs.motorFaults.stickyFaults, this.io::clearMotorStickyFaults, DeviceFaults.allMask);

        this.motorDisconnectedAlert.set(this.inputs.motorConnected);
        this.motorDisconnectedGlobalAlert.set(this.inputs.motorConnected);
        
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Intake/Periodic");
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Intake");
    }

    private Command genCommand(
        String name,
        Supplier<Measure<VoltageUnit>> voltage
    ) {
        var subsystem = this;
        return new Command() {
            {
                setName(name);
                addRequirements(subsystem);
            }

            @Override
            public void initialize() {

            }

            @Override
            public void execute() {
                io.setMotorVoltage(voltage.get());
            }

            @Override
            public void end(boolean interrupted) {
                io.setMotorVoltage(Volts.zero());
            }
        };
    }

    public Command stop(){
        return genCommand(
            "Stop", 
            Volts::zero
        );
    }

    public Command idle() {
        return genCommand(
            "Idle",
            () -> {
                if (hasAlgae.getAsBoolean()) {
                    return algaeHoldVoltage.get();
                } else if (hasCoral.getAsBoolean()) {
                    return coralHoldVoltage.get();
                } else {
                    return Volts.zero();
                }
            }
        );
    }

    public Command eject() {
        return genCommand(
            "Eject",
            ejectVoltage::get
        );
    }
    public Command ejectLevel1() {
        return genCommand(
            "Eject Level 1",
            ejectLevel1Voltage::get
        );
    }
    public Command ejectAlgae() {
        return genCommand(
            "Eject Algae",
            ejectAlgaeVoltage::get
        );
    }

    public Command intakeCoral() {
        var subsystem = this;
        return new Command() {
            {
                addRequirements(subsystem);
                setName("Intake Coral");
            }
            @Override
            public void initialize() {
                grabbingCoral = true;
                io.setMotorVoltage(intakeVoltage.get());
            }
            
        };
    }
    public Command intakeAlgae() {
        var subsystem = this;
        return new Command() {
            {
                addRequirements(subsystem);
                setName("Intake Algae");
            }
            @Override
            public void initialize() {
                grabbingCoral = false;
                io.setMotorVoltage(intakeVoltage.get());
            }
            
        };
    }
}