package frc.robot.subsystems.superstructure.pivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase {
    private final PivotIO io;
    private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

    public Pivot(PivotIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Pivot", inputs);
    }

    public void setPivot(Angle angle) {
        io.setPosition(angle);
    }
    
    public Command pivotTo(Angle angle) {
        var subsystem = this;
        return new Command() {
            {
                addRequirements(subsystem);
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
}
