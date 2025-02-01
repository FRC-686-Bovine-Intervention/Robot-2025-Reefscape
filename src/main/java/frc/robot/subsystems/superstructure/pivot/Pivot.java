package frc.robot.subsystems.superstructure.pivot;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.robotStructure.angle.ArmMech;

public class Pivot extends SubsystemBase {
    private final PivotIO io;
    private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

    public final ArmMech mech = new ArmMech(PivotConstants.pivotBase);

    public Pivot(PivotIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Pivot", inputs);

        mech.set(inputs.encoder.position);
    }

    public Angle getAngle() {
        return inputs.encoder.position;
    }

    public void setPivot(Measure<AngleUnit> angle) {
        io.setPosition(angle);
    }
    public void setVoltage(Measure<VoltageUnit> voltage) {
        io.setVoltage(voltage);
    }
    
    public Command pivotTo(Angle angle) {
        var subsystem = this;
        return new Command() {
            {
                addRequirements(subsystem);
                setName("Pivot to");
            }
            public void initialize() {
                execute();
            }
            public void execute() {
                setPivot(angle);
            }
            public void end(boolean interrupted) {
                io.stop();
            }
        };
    }
    
    public Command voltage(DoubleSupplier voltage) {
        var subsystem = this;
        return new Command() {
            {
                addRequirements(subsystem);
                setName("Voltage");
            }
            public void initialize() {
                execute();
            }
            public void execute() {
                io.setVoltage(Volts.of(voltage.getAsDouble()));
            }
            public void end(boolean interrupted) {
                io.stop();
            }
        };
    }
}
