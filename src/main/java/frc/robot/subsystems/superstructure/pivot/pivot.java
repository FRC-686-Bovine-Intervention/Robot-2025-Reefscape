package frc.robot.subsystems.superstructure.pivot;

import edu.wpi.first.math.util.Units;

public class Pivot {

    // Declare variables
    private final PivotIOFalcon pivotIO = new PivotIOFalcon();
    private double goal = 0;
    private final double zero = 0;

   public Pivot (int goal) {
        this.goal = goal;
   }

   // Get input and convert to degrees, then send to IO
   public void pivotToDegrees (double degrees) {
        this.goal = Units.degreesToRadians(degrees) + zero;
        pivotIO.setPivotPosition(goal);
   }

   // Get input and directly send to IO
   public void pivotTo (double rads) {
        this.goal = rads;
        pivotIO.setPivotPosition(goal);
   }
}