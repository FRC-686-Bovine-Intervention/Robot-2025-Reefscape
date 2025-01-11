package frc.robot.subsystems.superstructure.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.elevator.ElevatorIO.ElevatorIOInputs;

public class Elevator extends SubsystemBase{
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    public Elevator(ElevatorIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Elevator", inputs);
    }

    public Command elevateTo(Measure<DistanceUnit> dist) {
        var subsystem = this;
        return new Command() {
            {
                addRequirements(subsystem);
                setName("Elevate To");
            }
            public void initialize() {
                execute();
            }
            public void execute() {
                io.setPosition(dist);
            }
            public void end() {
                io.stop();
            }
        };
    }
}
