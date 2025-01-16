package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.pivot.Pivot;

public class Superstructure {
    public final Pivot pivot;
    public final Elevator elevator;

    public Superstructure(Pivot pivot, Elevator elevator) {
        this.pivot = pivot;
        this.elevator = elevator;
    }

    public Command pivotVoltage(DoubleSupplier voltage) {
        return new Command() {
            {
                addRequirements(pivot, elevator);
                setName("Pivot Voltage");
            }
            @Override
            public void initialize() {

            }
            @Override
            public void execute() {
                pivot.setVoltage(Volts.of(voltage.getAsDouble()));
                elevator.setVoltage(Volts.zero());
            }
        };
    }
    public Command elevatorVoltage(DoubleSupplier voltage) {
        return new Command() {
            {
                addRequirements(pivot, elevator);
                setName("Elevator Voltage");
            }
            @Override
            public void initialize() {

            }
            @Override
            public void execute() {
                pivot.setVoltage(Volts.zero());
                elevator.setVoltage(Volts.of(voltage.getAsDouble()));
            }
        };
    }
}
