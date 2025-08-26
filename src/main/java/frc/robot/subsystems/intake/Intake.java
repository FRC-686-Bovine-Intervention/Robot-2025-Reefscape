package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.LoggedTracer;
import frc.util.loggerUtil.tunables.LoggedTunable;
import frc.util.robotStructure.GamepiecePose;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public static final LoggedTunable<Voltage> intakeCoralVoltage =     LoggedTunable.from("Intake/Voltages/Intake Coral",    Volts::of,   +6);
    public static final LoggedTunable<Voltage> intakeAlgaeVoltage =     LoggedTunable.from("Intake/Voltages/Intake Algae",    Volts::of,   +6);
    public static final LoggedTunable<Voltage> ejectVoltage =           LoggedTunable.from("Intake/Voltages/Eject",           Volts::of,   -4);
    public static final LoggedTunable<Voltage> ejectL1Voltage =         LoggedTunable.from("Intake/Voltages/Eject Level 1",   Volts::of,   -2);
    public static final LoggedTunable<Voltage> ejectAlgaeVoltage =      LoggedTunable.from("Intake/Voltages/Algae",           Volts::of,   -12);
    public static final LoggedTunable<Voltage> coralHoldVoltage =       LoggedTunable.from("Intake/Voltages/Hold Coral",      Volts::of,   +0.3);
    public static final LoggedTunable<Voltage> algaeHoldVoltage =       LoggedTunable.from("Intake/Voltages/Hold Algae",      Volts::of,   +1);
    public static final LoggedTunable<Current> gamepieceDetectCurrent = LoggedTunable.from("Intake/Gamepiece Detect Current", Amps::of,    +35);
    public static final LoggedTunable<Time>    gamepieceDetectTime =    LoggedTunable.from("Intake/Gamepiece Detect Time",    Seconds::of, +2);

    public final GamepiecePose coralPose = new GamepiecePose(IntakeConstants.coralPose);
    public final GamepiecePose algaePose = new GamepiecePose(IntakeConstants.algaePose);
    
    private boolean hasGamepiece = false;
    private boolean grabbingCoral = true;

    private final Debouncer gamepieceSensorDebouncer = new Debouncer(gamepieceDetectTime.get().in(Seconds), DebounceType.kRising);

    // private final DeviceFaultAlerts motorActiveFaultsAlert = new DeviceFaultAlerts(new Alert("Intake/Alerts", "Motor has active faults: ", AlertType.kError));
    // private final DeviceFaultAlerts motorStickyFaultsAlert = new DeviceFaultAlerts(new Alert("Intake/Alerts", "Motor has sticky faults: ", AlertType.kWarning), FaultType.StatorCurrentLimit, FaultType.SupplyCurrentLimit);
    // private final DeviceFaultClearer motorStickyFaultClearer = new DeviceFaultClearer("Intake/Motor Sticky Faults");

    private final Alert motorDisconnectedAlert = new Alert("Intake/Alerts", "Motor Disconnected", AlertType.kError);
    private final Alert motorDisconnectedGlobalAlert = new Alert("Intake Motor Disconnected!", AlertType.kError);

    public Intake(IntakeIO io) {
        System.out.println("[Init Intake] Instantiated Intake with " + io.getClass().getSimpleName());
        this.io = io;
    }
    
    @Override
    public void periodic() {
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Intake/Before");
        this.io.updateInputs(this.inputs);
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Intake/Update Inputs");
        Logger.processInputs("Inputs/Intake", this.inputs);
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Intake/Process Inputs");

        if (gamepieceDetectTime.hasChanged(this.hashCode())) {
            this.gamepieceSensorDebouncer.setDebounceTime(gamepieceDetectTime.get().in(Seconds));
        }

        var second = this.gamepieceSensorDebouncer.calculate(this.inputs.coralSensor);

        if (this.inputs.coralSensor) {
            if (second || this.inputs.motor.getStatorCurrentAmps() > gamepieceDetectCurrent.get().in(Amps)) {
                this.hasGamepiece = true;
            }
        } else {
            this.hasGamepiece = false;
        }

        Logger.recordOutput("Intake/hasgamepiece", this.hasGamepiece);

        // this.motorActiveFaultsAlert.updateFrom(this.inputs.motorFaults.activeFaults);
        // this.motorStickyFaultsAlert.updateFrom(this.inputs.motorFaults.stickyFaults);
        // this.motorStickyFaultClearer.clear(this.inputs.motorFaults.stickyFaults, this.io::clearMotorStickyFaults, DeviceFaults.allMask);

        this.motorDisconnectedAlert.set(this.inputs.motorConnected);
        this.motorDisconnectedGlobalAlert.set(this.inputs.motorConnected);
        
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Intake/Periodic");
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Intake");
    }

    public boolean hasCoral() {
        return this.hasGamepiece && this.grabbingCoral;
    }

    public boolean hasAlgae() {
        return this.hasGamepiece && !this.grabbingCoral;
    }

    private Command genVoltageCommand(
        String name,
        DoubleSupplier voltsSupplier
    ) {
        final var intake = this;
        return new Command() {
            {
                this.addRequirements(intake);
                this.setName(name);
            }

            @Override
            public void initialize() {

            }

            @Override
            public void execute() {
                intake.io.setVolts(voltsSupplier.getAsDouble());
            }

            @Override
            public void end(boolean interrupted) {
                intake.io.setVolts(0);
            }
        };
    }

    public Command stop() {
        return genVoltageCommand(
            "Stop", 
            () -> 0.0
        );
    }

    public Command idle() {
        return genVoltageCommand(
            "Idle",
            () -> {
                if (this.hasAlgae()) {
                    return algaeHoldVoltage.get().in(Volts);
                } else if (this.hasCoral()) {
                    return coralHoldVoltage.get().in(Volts);
                } else {
                    return 0.0;
                }
            }
        );
    }

    public Command eject() {
        return genVoltageCommand(
            "Eject",
            () -> ejectVoltage.get().in(Volts)
        );
    }
    public Command ejectL1() {
        return genVoltageCommand(
            "Eject L1",
            () -> ejectL1Voltage.get().in(Volts)
        );
    }
    public Command ejectAlgae() {
        return genVoltageCommand(
            "Eject Algae",
            () -> ejectAlgaeVoltage.get().in(Volts)
        );
    }

    public Command intakeCoral() {
        final var intake = this;
        return new Command() {
            {
                this.addRequirements(intake);
                this.setName("Intake Coral");
            }

            @Override
            public void initialize() {
                grabbingCoral = true;
            }

            @Override
            public void execute() {
                intake.io.setVolts(intakeCoralVoltage.get().in(Volts));
            }

            @Override
            public void end(boolean interrupted) {
                intake.io.setVolts(0.0);
            }
        };
    }

    public Command intakeAlgae() {
        final var intake = this;
        return new Command() {
            {
                this.addRequirements(intake);
                this.setName("Intake Algae");
            }

            @Override
            public void initialize() {
                grabbingCoral = false;
            }

            @Override
            public void execute() {
                intake.io.setVolts(intakeAlgaeVoltage.get().in(Volts));
            }

            @Override
            public void end(boolean interrupted) {
                intake.io.setVolts(0.0);
            }
        };
    }
}