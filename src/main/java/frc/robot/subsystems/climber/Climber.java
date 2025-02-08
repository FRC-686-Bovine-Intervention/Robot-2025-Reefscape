package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.loggerUtil.tunables.LoggedTunableMeasure;

public class Climber extends SubsystemBase {
    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    public static final LoggedTunableMeasure<VoltageUnit> idleVoltage = new LoggedTunableMeasure<>("Climber/Voltages/Idle", Volts.of(1));
    public static final LoggedTunableMeasure<VoltageUnit> intakeVoltage = new LoggedTunableMeasure<>("Climber/Voltages/Idle", Volts.of(4));

    public Climber(ClimberIO io) {
        System.out.println("[Init Climber] Instantiated Climber with " + io.getClass().getSimpleName());
        this.io = io;
        SmartDashboard.putData(this);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Climber", inputs);
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

    public Command stop() {
        return genCommand("Stop", Volts::zero);
    }

    public Command idle() {
        return genCommand("Idle", idleVoltage);
    }

    public Command intake() {
        return genCommand("Intake", intakeVoltage);
    }
}
