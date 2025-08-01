package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.util.faults.DeviceFaultAlerts;
import frc.util.faults.DeviceFaults.FaultType;
import frc.util.loggerUtil.tunables.LoggedTunableMeasure;
import frc.util.robotStructure.GamepiecePose;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public static final LoggedTunableMeasure<VoltageUnit> intakeVoltage = new LoggedTunableMeasure<>("Intake/Voltages/Intake", Volts.of(6));
    public static final LoggedTunableMeasure<VoltageUnit> ejectVoltage = new LoggedTunableMeasure<>("Intake/Voltages/Eject", Volts.of(4).unaryMinus());
    public static final LoggedTunableMeasure<VoltageUnit> ejectLevel1Voltage = new LoggedTunableMeasure<>("Intake/Voltages/Eject Level 1", Volts.of(2).unaryMinus());
    public static final LoggedTunableMeasure<VoltageUnit> ejectAlgaeVoltage = new LoggedTunableMeasure<>("Intake/Voltages/Algae", Volts.of(12).unaryMinus());
    public static final LoggedTunableMeasure<VoltageUnit> coralHoldVoltage = new LoggedTunableMeasure<>("Intake/Voltages/Hold Coral", Volts.of(0.3));
    public static final LoggedTunableMeasure<VoltageUnit> algaeHoldVoltage = new LoggedTunableMeasure<>("Intake/Voltages/Hold Algae", Volts.of(1));
    public static final LoggedTunableMeasure<CurrentUnit> gamepieceDetectCurrent = new LoggedTunableMeasure<>("Intake/Gamepiece Detect Current", Amps.of(50));
    public static final LoggedTunableMeasure<TimeUnit> gamepieceDetectTime = new LoggedTunableMeasure<>("Intake/Gamepiece Detect Time", Seconds.of(2));

    public final GamepiecePose coralPose = new GamepiecePose(IntakeConstants.coralPose);
    public final GamepiecePose algaePose = new GamepiecePose(IntakeConstants.algaePose);
    
    private boolean hasGamepiece = false;
    private boolean grabbingCoral = true;
    public final Trigger hasCoral = new Trigger(() -> hasGamepiece && grabbingCoral);
    public final Trigger hasAlgae = new Trigger(() -> hasGamepiece && !grabbingCoral);

    private final DeviceFaultAlerts motorActiveFaultsAlert = new DeviceFaultAlerts(new Alert("Intake/Alerts", "Motor has active faults: ", AlertType.kError));
    private final DeviceFaultAlerts motorStickyFaultsAlert = new DeviceFaultAlerts(new Alert("Intake/Alerts", "Motor has sticky faults: ", AlertType.kWarning), FaultType.StatorCurrentLimit, FaultType.SupplyCurrentLimit);

    public Intake(IntakeIO io) {
        System.out.println("[Init Intake] Instantiated Intake with " + io.getClass().getSimpleName());
        this.io = io;
        SmartDashboard.putData("Subsystems/Intake", this);
    }

    private final Debouncer debouncer = new Debouncer(1, DebounceType.kRising);
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Intake", inputs);

        if (gamepieceDetectTime.hasChanged(hashCode())) {
            debouncer.setDebounceTime(gamepieceDetectTime.get().in(Seconds));
        }
        var second = debouncer.calculate(inputs.coralSensor);
        if (inputs.coralSensor) {
            if (inputs.motor.statorCurrent.gt(gamepieceDetectCurrent.get()) || second) {
                hasGamepiece = true;
            }
        } else {
            hasGamepiece = false;
        }

        Logger.recordOutput("Intake/hasgamepiece", hasGamepiece);

        Logger.recordOutput("Gamepiece/Coral",
            (hasCoral.getAsBoolean()) ? (
                new Pose3d[]{
                    coralPose.getFieldRelative()
                }
            ) : (
                new Pose3d[]{}
            )
        );
        Logger.recordOutput("Gamepiece/Algae",
            (hasAlgae.getAsBoolean()) ? (
                new Pose3d[]{
                    algaePose.getFieldRelative()
                }
            ) : (
                new Pose3d[]{}
            )
        );

        this.motorActiveFaultsAlert.updateFrom(this.inputs.motorFaults.activeFaults);
        this.motorStickyFaultsAlert.updateFrom(this.inputs.motorFaults.stickyFaults);
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
            ejectVoltage
        );
    }
    public Command ejectLevel1() {
        return genCommand(
            "Eject Level 1",
            ejectLevel1Voltage
        );
    }
    public Command ejectAlgae() {
        return genCommand(
            "Eject Algae",
            ejectAlgaeVoltage
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