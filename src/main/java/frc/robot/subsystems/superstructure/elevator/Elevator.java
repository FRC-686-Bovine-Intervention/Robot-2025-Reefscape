package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.robotStructure.linear.ExtenderMech;

public class Elevator extends SubsystemBase{
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    public final ExtenderMech stage2Mech = new ExtenderMech(ElevatorConstants.stage2Base);
    public final ExtenderMech stage3Mech = new ExtenderMech(ElevatorConstants.stage3Base);
    public final ExtenderMech stage4Mech = new ExtenderMech(ElevatorConstants.stage4Base);

    public Elevator(ElevatorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Elevator", inputs);

        var stageDist = ElevatorConstants.sprocketRadius.times(inputs.encoder.position.in(Radians));

        stage2Mech.set(stageDist);
        stage3Mech.set(stageDist);
        stage4Mech.set(stageDist);

        Logger.recordOutput("Elevator/Total Length", stageDist.times(ElevatorConstants.movingStages));
    }

    public Distance getLength() {
        return ElevatorConstants.sprocketRadius.times(inputs.encoder.position.in(Radians)).times(ElevatorConstants.movingStages);
    }

    public void setLength(Measure<DistanceUnit> dist) {
        io.setLength(dist);
    }
    public void setVoltage(Measure<VoltageUnit> voltage) {
        io.setVoltage(voltage);
    }

    public Command elevateTo(Measure<DistanceUnit> dist) {
        var subsystem = this;
        return new Command() {
            {
                addRequirements(subsystem);
                setName("Elevate To");
            }
            @Override
            public void initialize() {
                execute();
            }
            @Override
            public void execute() {
                setLength(dist);
            }
            @Override
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
            @Override
            public void initialize() {
                execute();
            }
            @Override
            public void execute() {
                io.setVoltage(Volts.of(voltage.getAsDouble()));
            }
            @Override
            public void end(boolean interrupted) {
                io.stop();
            }
        };
    }
}
