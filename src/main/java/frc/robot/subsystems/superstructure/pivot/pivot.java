package frc.robot.subsystems.superstructure.pivot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase {

    // Declare variables
    private final PivotIOFalcon pivotIO = new PivotIOFalcon();
    private int goal = 0;
    private final double zero = 0;

    public Pivot (int goal) {
        this.goal = goal;
    }

    // Get input and convert to degrees, then send to IO
    public Command pivotToDegrees (double degrees) {
        var subsystem = this;
        return new Command() {
            {
                addRequirements(subsystem);
            }
            @Override
            public void initialize () {
                execute();
            }
            public void execute () {
                double goal = Units.degreesToRadians(degrees) + zero;
                pivotIO.setPivotPosition(goal);  
            }
            public void end (boolean interrupted) {
                pivotIO.stop();
            }

        };
    }

    // Get input and directly send to IO
    public Command pivotTo (double rads) {
        var subsystem = this;
        return new Command() {
            {
                addRequirements(subsystem);
            }
            public void initialize () {
                execute();
            }
            public void execute () {
                double goal = rads;
                pivotIO.setPivotPosition(goal);
            }
            public void end () {
                pivotIO.stop();
            }
        };
    }
}