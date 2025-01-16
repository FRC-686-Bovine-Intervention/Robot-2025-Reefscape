package frc.robot.subsystems.superstructure.pivot;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.pivot.PivotIO.PivotIOInputsAutoLogged;

public class Pivot extends SubsystemBase {

    // Declare variables
    private final PivotIO io;
    private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

    public Pivot(PivotIO io) {
        this.io = io;
    }
    
    // Get input and directly send to IO
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
                io.setPosition(angle);
            }
            public void end(boolean interrupted) {
                io.stop();
            }
        };
    }
}
